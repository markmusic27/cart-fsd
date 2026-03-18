"""Serial protocol v2 — encode commands and decode state reports.

Host → Arduino
--------------
  T G<0-1> B<0-1> A<angle> [S<-1..1>]\\n
                                Set pedal targets and optional open-loop steering command
  C <key> <value>\\n             Config update
  E\\n                           Software emergency stop
  Z\\n                           Zero encoder

Arduino → Host
--------------
  S G<pos> B<pos> A<angle> GS<0|1> BS<0|1> AS<0|1> ES<0|1>\\n
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from cart.config import CartConfig, CartState


# ---------------------------------------------------------------------------
# Encoding — Python → Arduino
# ---------------------------------------------------------------------------

def encode_targets(
    gas: float,
    brake: float,
    steering_angle: float,
    steering_command: float | None = None,
) -> bytes:
    """Build a target command string.

    `steering_angle` is for closed-loop steering targets.
    `steering_command` is an optional normalized open-loop motor command in
    [-1.0, 1.0]. When present, firmware treats steering as open-loop.
    """
    command = f"T G{gas:.3f} B{brake:.3f} A{steering_angle:.1f}"
    if steering_command is not None:
        command += f" S{steering_command:.3f}"
    return f"{command}\n".encode()


def encode_estop() -> bytes:
    return b"E\n"


def encode_zero_encoder() -> bytes:
    return b"Z\n"


def encode_config(key: str, value: float | int) -> bytes:
    """Build a single config command.  *key* is e.g. ``GKP``, ``BKD``."""
    if isinstance(value, float):
        return f"C {key} {value:.4f}\n".encode()
    return f"C {key} {value}\n".encode()


# Config key mapping: CartConfig field → Arduino config key
_CONFIG_KEYS: list[tuple[str, str]] = [
    ("safety", "SAFE"),
    ("gas_kp", "GKP"),
    ("gas_kd", "GKD"),
    ("brake_kp", "BKP"),
    ("brake_kd", "BKD"),
    ("steering_kp", "SKP"),
    ("steering_kd", "SKD"),
    ("gas_max_pwm", "GMAX"),
    ("brake_max_pwm", "BMAX"),
    ("steering_max_pwm", "SMAX"),
    ("gas_deadband", "GDB"),
    ("brake_deadband", "BDB"),
    ("steering_deadband", "SDB"),
]


def encode_full_config(config: CartConfig) -> list[bytes]:
    """Return a list of config command bytes for every tunable parameter."""
    cmds: list[bytes] = []
    for attr, key in _CONFIG_KEYS:
        value = getattr(config, attr)
        if isinstance(value, bool):
            value = int(value)
        cmds.append(encode_config(key, value))
    return cmds


# ---------------------------------------------------------------------------
# Decoding — Arduino → Python
# ---------------------------------------------------------------------------

def decode_state(line: str) -> CartState | None:
    """Parse a state report line into a CartState, or *None* on failure.

    Expected format:
        S G<pos> B<pos> A<angle> GS<0|1> BS<0|1> AS<0|1> ES<0|1>
    """
    line = line.strip()
    if not line.startswith("S "):
        return None

    try:
        parts = line.split()
        values: dict[str, str] = {}
        for part in parts[1:]:
            # Two-char prefix keys: GS, BS, AS, ES  or single-char: G, B, A
            if len(part) >= 3 and part[:2] in ("GS", "BS", "AS", "ES"):
                values[part[:2]] = part[2:]
            elif len(part) >= 2 and part[0] in "GBA":
                values[part[0]] = part[1:]

        return CartState(
            gas_position=float(values.get("G", "0")),
            brake_position=float(values.get("B", "0")),
            steering_angle=float(values.get("A", "0")),
            gas_settled=values.get("GS", "0") == "1",
            brake_settled=values.get("BS", "0") == "1",
            steering_settled=values.get("AS", "0") == "1",
            e_stop_active=values.get("ES", "0") == "1",
            timestamp=time.monotonic(),
        )
    except (ValueError, KeyError):
        return None
