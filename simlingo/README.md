# SimLingo inference scratchpad

A small Modal app for running offline SimLingo inference and checking that the
public checkpoint produces sensible waypoints + commentary on held-out validation
frames. This is a sanity-check sandbox — not a deployment.

## What is SimLingo

[SimLingo](https://arxiv.org/abs/2503.09594) (Renz et al., CVPR'25 highlight) is a
vision-language-action (VLA) model for autonomous driving:

- Backbone: [`OpenGVLab/InternVL2-1B`](https://huggingface.co/OpenGVLab/InternVL2-1B)
  (a ~1B param vision-language model) fine-tuned with LoRA on InternLM2 + the
  InternVL ViT encoder.
- Input: a single front-facing camera frame (1024×512, FOV 110°, mounted at
  `(x=-1.5, y=0, z=2)` on a CARLA vehicle, bottom 30% cropped to hide the
  bonnet), the current speed in m/s, an ego-frame target waypoint (and the next
  one), plus a natural-language prompt that bakes in the route command and asks
  either "Predict the waypoints." or "What should the ego do next?".
- Output: 11 future waypoints at 0.2 s spacing (the "speed wps"), an equally-
  spaced route prediction, and free-form language (commentary / reasoning).
- Training data: 3.3M frames collected with the privileged PDM-Lite expert in
  CARLA 0.9.15, plus VQA, commentary, and "dreamer" instruction-following
  labels. Source: [`RenzKa/simlingo` dataset](https://huggingface.co/datasets/RenzKa/simlingo).
- The released checkpoint (`epoch=013`) lives at
  [`RenzKa/simlingo`](https://huggingface.co/RenzKa/simlingo).

The full driving stack at inference time is `(image, speed, target_point,
command-prompt) → VLM → (waypoints, commentary) → PID controllers → throttle,
brake, steer`. The PID part lives in `team_code/agent_simlingo.py` of the
upstream repo and only runs inside CARLA. We're focused on the VLM head: are
the predicted waypoints in the right ballpark, and does the language make sense?

## What this folder does

1. Builds a Modal image with PyTorch 2.2, the right transformers/peft/hydra
   versions, flash-attn, and a clone of [RenzKa/simlingo](https://github.com/RenzKa/simlingo)
   on the `PYTHONPATH`.
2. Downloads two things into Modal volumes (one-time, cached):
   - The checkpoint `pytorch_model.pt` + `.hydra/config.yaml` from
     `RenzKa/simlingo` on HF.
   - One validation chunk from `RenzKa/simlingo` on HF datasets
     (`data_simlingo_validation_1_scenario_routes_validation_..._chunk_001.tar.gz`),
     plus the matching `commentary_*_chunk_001.tar.gz` so we can sanity-check
     the language head against ground-truth commentary.
3. Loads the model with Hydra (using the checkpoint's own config), then walks
   a few validation routes, and for each sampled frame:
   - Reconstructs the same `DrivingInput` the live CARLA agent would build:
     letterbox-cropped image, ego-speed, ego-frame target points, command-style
     prompt with the `<TARGET_POINT>` placeholder.
   - Runs `model(driving_input)` to get `pred_speed_wps`, `pred_route`,
     `pred_language`.
   - Computes ground-truth waypoints from each frame's `ego_matrix` (next 11
     frames at 5-frame stride → 0.25 s spacing as in training).
4. Reports per-sample and aggregate metrics, and writes a side-by-side image
   per sample: GT waypoints (green), predicted speed waypoints (red), and the
   predicted commentary text underneath.

## Metrics

For a quick "is the model alive?" check we report:

| Metric | What it measures |
| --- | --- |
| Waypoint ADE / FDE (m) | Mean and final L2 displacement between predicted and GT speed waypoints |
| Path ADE / FDE (m) | Same, but for the route prediction vs the dataset's `route_adjusted` |
| Speed @ 1 s (m/s) | Implied speed from waypoints (matches the controller's brake heuristic) |
| Brake-flag agreement | Does the implied speed cross the brake threshold when GT does? |
| Commentary BLEU / exact-match | Cheap text overlap of predicted vs GT commentary (when available) |

These are not the paper's headline metrics (Driving Score on Bench2Drive / CARLA
Leaderboard 2.0 is closed-loop in CARLA) — those require running the model
inside a CARLA server, which we're explicitly skipping here. Closed-loop
evaluation is a separate (much heavier) follow-up.

## What "evaluating on driving data" looks like long-term

The Stanford cart's sensor rig is similar in spirit to CARLA's front-cam-only
setup, but cameras, mounting heights, FOVs, and coordinate frames don't match
exactly. To use SimLingo zero-shot on cart data, we'd need to either:

1. Letterbox/resize the cart's front camera to roughly match SimLingo's
   training distribution (1024×512, FOV ≈ 110°, bottom ~30% removed), and feed
   manually-authored target points + commands in CARLA's ego frame
   (x-forward, y-right, meters). This is the cheapest path and what this
   sandbox is structured to support next.
2. Fine-tune / LoRA-adapt the model on a few minutes of cart teleop data.

Both of those depend on first establishing that the public checkpoint isn't
broken in our setup, which is what this folder is for.

## How to run

```bash
# one-time, from the repo root
uv pip install --group simlingo  # or: pip install modal
modal setup                       # if you haven't auth'd modal yet

cd simlingo
modal run modal_app.py::prepare_assets    # downloads weights + 1 val chunk
modal run modal_app.py::run --num-samples 64
```

The second command prints aggregate metrics and writes overlay images +
per-sample JSON into the `simlingo-outputs` Modal volume. Use
`modal volume get simlingo-outputs <remote-path> <local-path>` to pull the
images down.

`modal run modal_app.py::run --help` (or look at `modal_app.py`) for the rest of
the knobs (number of routes, GPU type, language eval on/off, etc.).

## Layout

```
simlingo/
├── README.md            # this file
├── pyproject.toml       # local Modal client + ruff
├── modal_app.py         # Modal image, volumes, entrypoints
├── scripts/
│   ├── __init__.py
│   ├── inference.py     # builds DrivingInput, runs model, computes metrics
│   └── viz.py           # waypoint overlays on the front-cam image
└── outputs/             # pulled-down artifacts (gitignored)
```

## Things to double-check

- The released checkpoint is a **reproduction** of the paper's model, retrained
  on the released dataset, so absolute metric numbers will be a bit different
  from the paper. The README on the upstream repo flags this.
- The dataset is CARLA 0.9.15 (Town12/13). Generalization to a real-world cart
  is not guaranteed — closed-loop CARLA eval scores are the only existing
  evidence.
- All waypoints are in the CARLA ego frame: `+x` forward, `+y` right, meters,
  origin at the rear axle (`extrinsics[:3, 3] = [-1.5, 0, 2]` in the camera
  extrinsics).

## Upstream references

- Paper: <https://arxiv.org/abs/2503.09594>
- Code: <https://github.com/RenzKa/simlingo>
- Model card: <https://huggingface.co/RenzKa/simlingo>
- Dataset card: <https://huggingface.co/datasets/RenzKa/simlingo>
- Project page: <https://www.katrinrenz.de/simlingo/>
