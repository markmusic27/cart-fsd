# Cart Controller Design

## Overview

The cart controller is a two-layer system:

- **Python host** (`cart/controller.py`) — decides *what* the cart should do
  (target positions and angles) based on input from a PS5 controller, the
  Alpamayo R1 autonomy stack, or any other source.
- **Arduino Mega firmware** (`firmware/firmware.ino`) — decides *how*
  to get there using PD closed-loop control at 100 Hz.

The Python side sends target positions over serial; the Arduino drives the
actuators to those targets autonomously.  If the Python host disconnects,
the Arduino watchdog auto-brakes the cart.

## Why PD Control (Not Adam / PID)

The user's intuition was "steps toward the target that get smaller as you
approach" — this is exactly what a **Proportional-Derivative (PD)** controller
does:

- **P term** (`Kp * error`): Output is proportional to the distance from
  target.  Far away = large PWM (fast movement).  Close = small PWM (gentle
  approach).  This gives the "shrinking step" behavior.

- **D term** (`-Kd * velocity`): Acts as a brake that increases with speed.
  Prevents overshoot by dampening the approach.

Why not Adam?  Adam's **momentum** would accumulate past corrections and
cause overshoot — exactly what we need to avoid with physical actuators.

Why not PID?  The **I term** (integral) accumulates error over time to
correct steady-state offset.  For position control with potentiometer
feedback, the P term alone drives error to zero — an I term would overshoot.

**Critically damped PD** gives the fastest possible response with zero
overshoot.

## Serial Protocol (v2)

Baud rate: **115200**.

### Host → Arduino

| Command      | Format                       | Example                   |
|--------------|------------------------------|---------------------------|
| Set targets  | `T G<0-1> B<0-1> A<angle>\n`| `T G0.50 B0.00 A-15.3\n` |
| Config       | `C <key> <value>\n`          | `C GKP 800\n`             |
| E-stop       | `E\n`                        | `E\n`                     |
| Zero encoder | `Z\n`                        | `Z\n`                     |

**Target command fields:**
- `G`: Gas pedal position, 0.0 (released) to 1.0 (max configured travel)
- `B`: Brake pedal position, 0.0 (released) to 1.0 (max configured travel)
- `A`: Steering angle in degrees at the steering column (+ = right, - = left)

The `T` command is sent at 50 Hz baseline by the background thread.  When a
target changes, it is sent immediately (send-on-change via `threading.Event`).

**Config keys:**

| Key  | Parameter          | Type  |
|------|--------------------|-------|
| GKP  | Gas Kp             | float |
| GKD  | Gas Kd             | float |
| BKP  | Brake Kp           | float |
| BKD  | Brake Kd           | float |
| SKP  | Steering Kp        | float |
| SKD  | Steering Kd        | float |
| GMAX | Gas max PWM        | int   |
| BMAX | Brake max PWM      | int   |
| SMAX | Steering max PWM   | int   |
| GDB  | Gas deadband       | float |
| BDB  | Brake deadband     | float |
| SDB  | Steering deadband  | float |

### Arduino → Host

State reports at ~50 Hz:

```
S G<pos> B<pos> A<angle> GS<0|1> BS<0|1> AS<0|1> ES<0|1>\n
```

| Field | Meaning                                        |
|-------|------------------------------------------------|
| G     | Current gas position (0–1 normalized)          |
| B     | Current brake position (0–1 normalized)        |
| A     | Current steering angle (degrees at column)     |
| GS    | Gas settled (within deadband of target)        |
| BS    | Brake settled                                  |
| AS    | Steering settled                               |
| ES    | Hardware e-stop active                         |

## Python API

```python
from cart import CartController, CartConfig

config = CartConfig(
    gas_max_position=0.67,     # cap at ~10 mph
    brake_max_position=1.0,
    gas_max_pwm=200,           # limit actuator speed
)

ctl = CartController("/dev/cu.usbmodem1201", config)

ctl.set_gas(0.5)       # 50% of capped range
ctl.set_brake(0.0)
ctl.set_steering(15.0) # 15° right

state = ctl.state      # non-blocking read
print(state.gas_position, state.gas_settled)

ctl.stop()             # software e-stop
ctl.close()
```

### CartConfig Fields

| Field               | Default | Description                              |
|---------------------|---------|------------------------------------------|
| gas_max_position    | 1.0     | Fraction of gas travel (speed cap)       |
| brake_max_position  | 1.0     | Fraction of brake travel                 |
| steering_max_angle  | 240.0   | Max steering angle (degrees)             |
| gas_max_pwm         | 255     | Max PWM for gas actuator                 |
| brake_max_pwm       | 255     | Max PWM for brake actuator               |
| steering_max_pwm    | 200     | Max PWM for steering motor               |
| gas_kp / gas_kd     | 800/50  | Gas PD gains                             |
| brake_kp / brake_kd | 600/40  | Brake PD gains                           |
| steering_kp / kd    | 5.0/0.3 | Steering PD gains                        |
| gas_deadband        | 0.008   | Gas position tolerance                   |
| brake_deadband      | 0.02    | Brake position tolerance                 |
| steering_deadband   | 2.0     | Steering angle tolerance (degrees)       |

### Speed Cap Example

The cart tops out at ~15 mph with the gas actuator at full travel.

| Desired max speed | gas_max_position | Pot target at gas=1.0               |
|-------------------|------------------|--------------------------------------|
| 15 mph (no cap)   | 1.0              | 0.05 + 1.0 × 0.67 = 0.72           |
| 10 mph            | 0.67             | 0.05 + 0.67 × 0.67 = 0.50          |
| 5 mph             | 0.33             | 0.05 + 0.33 × 0.67 = 0.27          |

## Pedal Calibration

Calibrated potentiometer readings (normalized ADC / 1023):

| Pedal | POS_MIN (retracted) | POS_MAX (fully pressed) | Range |
|-------|---------------------|-------------------------|-------|
| Gas   | 0.05                | 0.72                    | 0.67  |
| Brake | 0.47                | 0.90                    | 0.43  |

These are baked into the Arduino firmware.  The Python side works in
normalized 0–1 space; the Arduino maps to the physical range:

```
pot_target = POS_MIN + user_input × (POS_MAX - POS_MIN)
```

## PD Controller Implementation

On the Arduino, every 10 ms (100 Hz loop):

```c
float error = target - current_pos;
float velocity = (current_pos - prev_pos) / dt;
float output = kp * error - kd * velocity;

output = constrain(output, -max_pwm, max_pwm);

if (abs(error) < deadband)       → disable motor (settled)
else if (abs(output) < MIN_PWM)  → apply MIN_PWM (overcome static friction)
else                              → drive motor at output
```

The **derivative is on position (velocity), not on error**.  This avoids
"derivative kick" when the target changes suddenly — only the approach
velocity is dampened, not the new setpoint.

## Responsiveness

| Component              | Latency                     |
|------------------------|-----------------------------|
| Arduino PD loop        | 10 ms (100 Hz)              |
| Python send-on-change  | ~1 ms (immediate wake)      |
| Python baseline tick   | 20 ms (50 Hz)               |
| Serial transmission    | ~2 ms (25 bytes @ 115200)   |
| **Worst case total**   | **~25 ms**                  |
| **Typical total**      | **~12 ms**                  |
