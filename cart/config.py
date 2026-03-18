"""Hardware constants and configuration dataclasses for the cart controller."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


# ---------------------------------------------------------------------------
# Hardware constants (baked into Arduino firmware — do not change at runtime)
# ---------------------------------------------------------------------------

# Potentiometer calibration: normalized ADC readings (0.0–1.0) at mechanical
# endpoints.  Measured from the old caddy_drive.ino firmware.
GAS_POS_MIN: float = 0.05
GAS_POS_MAX: float = 0.72
BRAKE_POS_MIN: float = 0.47
BRAKE_POS_MAX: float = 0.90

GAS_POS_RANGE: float = GAS_POS_MAX - GAS_POS_MIN
BRAKE_POS_RANGE: float = BRAKE_POS_MAX - BRAKE_POS_MIN

# Encoder
ENCODER_PPR: float = 600.0
ENCODER_WHEEL_DIA_MM: float = 30.8
STEERING_COL_DIA_MM: float = 90.0
ENCODER_GEAR_RATIO: float = ENCODER_WHEEL_DIA_MM / STEERING_COL_DIA_MM

# Sprocket chain
MOTOR_SPROCKET_TEETH: int = 25
STEERING_SPROCKET_TEETH: int = 36
SPROCKET_RATIO: float = MOTOR_SPROCKET_TEETH / STEERING_SPROCKET_TEETH

# Steering limits (measured from steering_calibration.json)
STEERING_MAX_RIGHT_DEG: float = 250.0
STEERING_MAX_LEFT_DEG: float = -250

# Cart speed reference (approximate)
CART_TOP_SPEED_MPH: float = 15.0

# Serial
DEFAULT_BAUD_RATE: int = 115200

# Timing
SEND_INTERVAL_S: float = 0.010  # 100 Hz baseline tick
WATCHDOG_TIMEOUT_MS: int = 250

# Motor PWM limits (Arduino-side)
PWM_MIN: int = 0
PWM_MAX: int = 255
MIN_PWM_THRESHOLD: int = 60  # below this the motor can't overcome static friction

# Control limits settings file
REPO_ROOT = Path(__file__).resolve().parent.parent
CONTROL_LIMITS_PATH = REPO_ROOT / "settings" / "control_limits.json"


def load_control_limits() -> dict[str, int | float]:
    """Load editable control limits from disk.

    The file is intentionally simple so it can be edited quickly:

        {
          "gas_limit_percent": 100,
          "steering_max_right_deg": 220,
          "steering_max_left_deg": -220
        }
    """
    default_limits: dict[str, int | float] = {
        "gas_limit_percent": 100,
        "steering_max_right_deg": 220.0,
        "steering_max_left_deg": -220.0,
    }
    try:
        data = json.loads(CONTROL_LIMITS_PATH.read_text())
        gas_limit_percent = int(data.get("gas_limit_percent", 100))
        steering_max_right_deg = float(data.get("steering_max_right_deg", 220.0))
        steering_max_left_deg = float(data.get("steering_max_left_deg", -220.0))
        default_limits["gas_limit_percent"] = max(0, min(100, gas_limit_percent))
        default_limits["steering_max_right_deg"] = max(0.0, steering_max_right_deg)
        default_limits["steering_max_left_deg"] = min(0.0, steering_max_left_deg)
    except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError, ValueError):
        pass
    return default_limits


def default_gas_limit_percent() -> int:
    return load_control_limits()["gas_limit_percent"]


def default_gas_limit_fraction() -> float:
    return default_gas_limit_percent() / 100.0


def default_steering_max_right_deg() -> float:
    return float(load_control_limits()["steering_max_right_deg"])


def default_steering_max_left_deg() -> float:
    return float(load_control_limits()["steering_max_left_deg"])


# ---------------------------------------------------------------------------
# Runtime configuration — passed to CartController, sent to Arduino at init
# ---------------------------------------------------------------------------

@dataclass
class CartConfig:
    """Tunable parameters for the cart controller.

    Displacement caps are fractions of the calibrated potentiometer range
    (0.0–1.0).  For example ``gas_max_position=0.7`` limits the gas actuator
    to 70 % of its full travel, which roughly caps the cart at ~10 mph.

    The Arduino maps a user input *u* (0–1) to a pot target:
        pot_target = POS_MIN + u * (POS_MAX - POS_MIN)
    The Python-side cap clips *u* before sending:
        u_clamped = min(u, gas_max_position)
    """

    # --- displacement caps (0.0–1.0) ---
    gas_max_position: float = field(default_factory=default_gas_limit_fraction)
    brake_max_position: float = 1.0
    steering_max_angle: float = 240.0  # symmetric legacy fallback
    steering_max_right_angle: float = field(default_factory=default_steering_max_right_deg)
    steering_max_left_angle: float = field(default_factory=default_steering_max_left_deg)
    safety: bool = True

    # --- actuator speed caps (max PWM 0–255) ---
    gas_max_pwm: int = 255
    brake_max_pwm: int = 255
    steering_max_pwm: int = 255

    # --- PD gains (sent to Arduino via config commands) ---
    gas_kp: float = 1200.0
    gas_kd: float = 70.0
    brake_kp: float = 1000.0
    brake_kd: float = 60.0
    steering_kp: float = 2.0
    steering_kd: float = 0.9

    # --- deadbands ---
    gas_deadband: float = 0.05
    brake_deadband: float = 0.05
    steering_deadband: float = 10.0  # about 5% of a 180-200 deg steering range


# ---------------------------------------------------------------------------
# Live state reported by the Arduino
# ---------------------------------------------------------------------------

@dataclass
class CartState:
    """Snapshot of the cart's current sensor readings."""

    gas_position: float = 0.0       # 0–1 within calibrated range
    brake_position: float = 0.0     # 0–1
    steering_angle: float = 0.0     # degrees at steering column
    gas_settled: bool = False
    brake_settled: bool = False
    steering_settled: bool = False
    e_stop_active: bool = False
    timestamp: float = 0.0          # time.monotonic() when parsed
