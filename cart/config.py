"""Hardware constants and configuration dataclasses for the cart controller."""

from __future__ import annotations

from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Hardware constants (baked into Arduino firmware — do not change at runtime)
# ---------------------------------------------------------------------------

# Potentiometer calibration: normalized ADC readings (0.0–1.0) at mechanical
# endpoints.  Measured from the old caddy_drive.ino firmware.
GAS_POS_MIN: float = 0.05
GAS_POS_MAX: float = 0.72
BRAKE_POS_MIN: float = 0.42
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
STEERING_MAX_RIGHT_DEG: float = 272.9
STEERING_MAX_LEFT_DEG: float = -276.8

# Cart speed reference (approximate)
CART_TOP_SPEED_MPH: float = 15.0

# Serial
DEFAULT_BAUD_RATE: int = 115200

# Timing
SEND_INTERVAL_S: float = 0.020  # 50 Hz baseline tick
WATCHDOG_TIMEOUT_MS: int = 250

# Motor PWM limits (Arduino-side)
PWM_MIN: int = 0
PWM_MAX: int = 255
MIN_PWM_THRESHOLD: int = 60  # below this the motor can't overcome static friction


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
    gas_max_position: float = 1.0
    brake_max_position: float = 1.0
    steering_max_angle: float = 240.0  # degrees at the steering column

    # --- actuator speed caps (max PWM 0–255) ---
    gas_max_pwm: int = 255
    brake_max_pwm: int = 255
    steering_max_pwm: int = 200

    # --- PD gains (sent to Arduino via config commands) ---
    gas_kp: float = 800.0
    gas_kd: float = 50.0
    brake_kp: float = 600.0
    brake_kd: float = 40.0
    steering_kp: float = 5.0
    steering_kd: float = 0.3

    # --- deadbands ---
    gas_deadband: float = 0.008
    brake_deadband: float = 0.02
    steering_deadband: float = 2.0  # degrees


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
