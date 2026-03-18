# Caddy Cart FSD — Hardware Context

## Project Overview

Self-driving golf cart platform. A Python application (this repo) handles
high-level autonomy (perception, planning, control) and communicates with an
**Arduino Mega 2560** that drives all low-level actuators and reads all sensors.

The cart's stock steering, throttle and brake are actuated mechanically — nothing
in the cart's original electrical system is modified.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Golf Cart                            │
│                                                         │
│   Cart Battery ──► 6-Way Fuse Box (5 A per channel)    │
│        │                                                │
│        ├──► Buck Converter → 12 V  ──► Linear Actuators│
│        │                           ──► Arduino Mega     │
│        └──► Buck Converter → 24 V  ──► Steering Motor  │
│                                                         │
│   Arduino Mega 2560                                     │
│     ├── 3× BTS7960 H-Bridge  (steering, brake, gas)    │
│     ├── 2× Linear Potentiometer  (brake pos, gas pos)  │
│     └── 1× Rotary Encoder  (steering angle)            │
│                                                         │
│   Host Computer  ◄──serial──►  Arduino Mega             │
└─────────────────────────────────────────────────────────┘
```

---

## Power System

| Rail    | Source             | Voltage | Consumers                              |
|---------|--------------------|---------|----------------------------------------|
| Battery | Golf cart battery  | ~36-48V | Input to both buck converters           |
| 12 V    | Buck converter #1  | 12 V    | Brake actuator, gas actuator, Arduino   |
| 24 V    | Buck converter #2  | 24 V    | Steering worm gear motor               |

All motor channels pass through a **Blue Sea Systems 5025 ST Blade Fuse Block**
(6-way, 100 A bus) with **ATC 5 A blade fuses** per motor circuit.  The 5 A
rating is just above normal operating current for all motors, so fuses will
**blow on a sustained stall** before motors or H-bridges are damaged.  Software
stall detection is still recommended for faster response (avoids blown fuses).

---

## Actuators

### Steering Motor

| Parameter       | Value                                                       |
|-----------------|-------------------------------------------------------------|
| Model           | uxcell JCF63R DC24V 80W 100RPM 8 N·m worm gear motor       |
| Voltage         | 24 V DC                                                     |
| Power           | 80 W                                                        |
| Output speed    | 100 RPM (no load)                                           |
| Output torque   | 8 N·m                                                       |
| Shaft diameter  | 10 mm (keyed)                                               |
| Reversible      | Yes (reverse polarity via H-bridge)                         |
| Configuration   | Right-angle worm gearbox                                    |
| Self-locking    | Yes — worm gear prevents back-driving when unpowered        |

**Drive train to steering column:**

- Motor sprocket: ANSI #25, **25 teeth**, ¼″ pitch, 10 mm bore w/ keyway
- Steering-column sprocket: ANSI #25, **36 teeth**, same pitch
- Chain: ANSI #25 roller chain
- **Sprocket ratio:** 25 : 36 → motor turns **1.44 rev** per 1 rev of the
  steering column (i.e. the steering column is geared down from the motor)
- Effective steering column speed ≈ 100 × (25/36) ≈ **69.4 RPM** (no load)

### Linear Actuators (×2 — brake & gas)

| Parameter       | Value                                                   |
|-----------------|---------------------------------------------------------|
| Voltage         | 12 V DC                                                 |
| Stroke          | 4 inches (101.6 mm)                                     |
| Force           | 66 lbs / 300 N                                          |
| Speed           | 1.77 in/s (no load)                                     |
| Full-stroke time| ~2.26 s (no load)                                       |
| Rating          | IP54                                                    |
| Mounting        | Included brackets                                       |

Each actuator presses down on one pedal (gas or brake). A separate linear
potentiometer is mounted alongside each actuator to measure displacement.

---

## Sensors

### Linear Potentiometers (×2)

Standard linear potentiometers wired as voltage dividers (5 V reference from
the Arduino).  The wiper voltage is read by an Arduino analog input (10-bit
ADC, 0–1023).

**Calibration values (voltage at the analog pin):**

| Pedal | Analog Pin | Min (retracted) | Max (extended) |
|-------|------------|-----------------|----------------|
| Brake | A0         | 0.485 V         | 0.885 V        |
| Gas   | A4         | 0.07 V          | 0.726 V        |

> These correspond roughly to ADC readings of **99–181** (brake) and
> **14–148** (gas) on a 5 V / 10-bit ADC.  Confirm with live calibration.

### Rotary Encoder (steering angle)

An incremental rotary encoder whose wheel is driven by a **3D-printed gear**
meshed with the steering wheel.

| Parameter                  | Value                                |
|----------------------------|--------------------------------------|
| Steering wheel gear ⌀      | 90 mm                                |
| Encoder wheel gear ⌀       | 30.8 mm                              |
| Gear ratio (steer → enc)  | 90 / 30.8 ≈ **2.922 : 1**           |
| Encoder revs per steer rev| ≈ 2.922                              |

The encoder has **three signal wires** connected to interrupt-capable pins on the
Arduino Mega:

| Wire colour | Arduino Pin | Mega Interrupt | Likely signal |
|-------------|-------------|----------------|---------------|
| White       | 18          | INT5           | Channel A     |
| Green       | 19          | INT4           | Channel B     |
| Yellow      | 20          | INT3           | Index / Z     |

Quadrature decoding on channels A + B gives direction and position.  The index
pulse (if present) provides a once-per-revolution reference.

---

## Motor Drivers — BTS7960 H-Bridges (×3)

Each BTS7960 module has four control lines from the Arduino:

| Line  | Function                                      |
|-------|-----------------------------------------------|
| R_EN  | Right-side enable (HIGH to enable)            |
| R_PWM | Right-side PWM — drives motor in one direction|
| L_EN  | Left-side enable (HIGH to enable)             |
| L_PWM | Left-side PWM — drives motor in other direction|

To drive a motor:

1. Set **both** enable pins HIGH.
2. Write a PWM duty cycle (0–255) to **one** PWM pin while keeping the other at 0.
3. Swap which PWM pin is active to reverse direction.
4. Set enables LOW to coast/stop.

---

## Arduino Mega 2560 — Complete Pin Map

### Steering Motor (BTS7960 #1 — 24 V rail)

| Signal | Pin | PWM? | Interrupt? |
|--------|-----|------|------------|
| R_EN   | 8   | Yes  | —          |
| R_PWM  | 2   | Yes  | INT0       |
| L_EN   | 9   | Yes  | —          |
| L_PWM  | 3   | Yes  | INT1       |

### Brake Actuator (BTS7960 #2 — 12 V rail)

| Signal | Pin | PWM? | Interrupt? |
|--------|-----|------|------------|
| R_EN   | 12  | Yes  | —          |
| R_PWM  | 10  | Yes  | —          |
| L_EN   | 13  | Yes  | —          |
| L_PWM  | 11  | Yes  | —          |

### Gas Actuator (BTS7960 #3 — 12 V rail)

| Signal | Pin | PWM? | Interrupt? |
|--------|-----|------|------------|
| R_EN   | 6   | Yes  | —          |
| R_PWM  | 4   | Yes  | —          |
| L_EN   | 7   | Yes  | —          |
| L_PWM  | 5   | Yes  | —          |

### Potentiometers

| Sensor    | Pin | Type   |
|-----------|-----|--------|
| Brake pot | A0  | Analog |
| Gas pot   | A4  | Analog |

### Rotary Encoder

| Wire   | Pin | Type             |
|--------|-----|------------------|
| White  | 18  | Digital (INT5)   |
| Green  | 19  | Digital (INT4)   |
| Yellow | 20  | Digital (INT3)   |

### Pin usage summary (digital 0–53)

```
 0  —  (RX0, reserved for USB serial)
 1  —  (TX0, reserved for USB serial)
 2  ■  Steering R_PWM          (PWM, INT0)
 3  ■  Steering L_PWM          (PWM, INT1)
 4  ■  Gas R_PWM               (PWM)
 5  ■  Gas L_PWM               (PWM)
 6  ■  Gas R_EN                (PWM)
 7  ■  Gas L_EN                (PWM)
 8  ■  Steering R_EN           (PWM)
 9  ■  Steering L_EN           (PWM)
10  ■  Brake R_PWM             (PWM)
11  ■  Brake L_PWM             (PWM)
12  ■  Brake R_EN              (PWM)
13  ■  Brake L_EN              (PWM)
14–17  (free)
18  ■  Encoder White / Ch A    (INT5)
19  ■  Encoder Green / Ch B    (INT4)
20  ■  Encoder Yellow / Index  (INT3)
21  —  (INT2, free)
22–53  (free)
```

### Analog pins

```
A0  ■  Brake potentiometer
A1     (free)
A2     (free)
A3     (free)
A4  ■  Gas potentiometer
A5–A15 (free)
```

---

## Derived Constants (for firmware / control code)

```
# Sprocket chain
MOTOR_SPROCKET_TEETH     = 25
STEERING_SPROCKET_TEETH  = 36
SPROCKET_RATIO           = 25.0 / 36.0   # motor → steering column

# Encoder-to-steering gear
STEERING_WHEEL_GEAR_DIA  = 90.0   # mm
ENCODER_WHEEL_GEAR_DIA   = 30.8   # mm
ENCODER_GEAR_RATIO       = 90.0 / 30.8   # ≈ 2.922 encoder revs per steering rev

# Potentiometer calibration (volts at analog pin, 5 V ref)
BRAKE_POT_MIN_V = 0.485
BRAKE_POT_MAX_V = 0.885
GAS_POT_MIN_V   = 0.07
GAS_POT_MAX_V   = 0.726

# ADC equivalents (10-bit, 5 V ref)
BRAKE_POT_MIN_ADC = int(0.485 / 5.0 * 1023)   # ≈ 99
BRAKE_POT_MAX_ADC = int(0.885 / 5.0 * 1023)   # ≈ 181
GAS_POT_MIN_ADC   = int(0.07  / 5.0 * 1023)   # ≈ 14
GAS_POT_MAX_ADC   = int(0.726 / 5.0 * 1023)   # ≈ 148

# Actuator physical limits
ACTUATOR_STROKE_IN       = 4.0    # inches
ACTUATOR_SPEED_IN_S      = 1.77   # inches/sec (no load)
ACTUATOR_FORCE_LBS       = 66     # lbs
ACTUATOR_FULL_STROKE_SEC = ACTUATOR_STROKE_IN / ACTUATOR_SPEED_IN_S  # ≈ 2.26 s

# Steering motor
STEERING_MOTOR_RPM       = 100    # no-load output RPM
STEERING_MOTOR_TORQUE_NM = 8.0
STEERING_COLUMN_RPM      = STEERING_MOTOR_RPM * SPROCKET_RATIO  # ≈ 69.4

# BTS7960 PWM range
PWM_MIN = 0
PWM_MAX = 255

# Fuse box — Blue Sea 5025, ATC 5 A blade fuses per motor channel
FUSE_RATING_AMPS = 5   # will blow on sustained stall (all motors stall above 5 A)
```

---

## Important Notes

- **Steering motor is self-locking** — the worm gear prevents the steering
  wheel from back-driving the motor when power is removed. This is a safety
  feature: if the system loses power, steering holds its last position.
- **Linear actuators have no built-in position feedback** — position is inferred
  entirely from the external linear potentiometers mounted alongside them.
- **Encoder needs a known home position** — the incremental encoder only tracks
  relative movement; a homing routine (e.g. steer to a hard stop or index
  pulse) is required on startup to establish absolute steering angle.
- **5 A fuses protect against motor stall damage.** The steering motor draws
  ~3.3 A nominal (stall likely 8–15 A) and each linear actuator draws ~2–5 A
  nominal (stall likely 8–10 A).  The ATC 5 A fuses will blow on a sustained
  stall before the motors overheat.  Software stall detection is still
  recommended to cut power faster and avoid having to replace blown fuses.
- **PWM frequency matters** — the BTS7960 supports up to 25 kHz PWM. Arduino
  Mega default PWM frequencies vary by timer (490 Hz or 980 Hz). Adjusting
  timer registers can reduce audible motor whine if needed.
- **Pins 2 and 3 are shared** between steering motor PWM and external
  interrupts INT0/INT1. This is fine as long as those interrupts aren't
  needed for another purpose (the encoder uses pins 18–20 / INT3–INT5).
- **Pins 20/21 conflict with I2C** — pin 20 is used for the encoder and
  therefore I2C (SDA=20, SCL=21) is **not available** on the Mega's default
  I2C bus. Use `Wire1` (SDA=SDA1/pin 44, SCL=SCL1/pin 45) if I2C peripherals
  are added later.

---

## Old Codebase Reference (caddy-arduino/caddy_drive)

The working (but messy) codebase lives at `caddy-arduino/caddy_drive/`. It
contains the Arduino firmware and three Python host-side controllers. This
section documents the proven control approach so it can be cleaned up and
rewritten properly.

### Files

| File                     | Purpose                                              |
|--------------------------|------------------------------------------------------|
| `caddy_drive.ino`        | Arduino Mega firmware — all motor/sensor logic       |
| `caddy_controller.py`    | PS controller → manual drive (joystick + triggers)   |
| `caddy_wasd.py`          | WASD keyboard → manual drive (headless terminal)     |
| `sunny_controller.py`    | PS controller + sunnypilot ONNX AI steering/throttle |
| `steering_state.json`    | Persisted encoder angle across runs                  |
| `steering_calibration.json` | Measured max left/right steering limits            |
| `models/`                | Sunnypilot ONNX models (vision, policy, dmonitoring) |

### Serial Protocol (Host ↔ Arduino)

**Baud rate:** 115200 (controller scripts) / 9600 (WASD script)

**Host → Arduino commands:**

| Command                  | Effect                                       |
|--------------------------|----------------------------------------------|
| `G<0-1> B<0-1> S<-1\|0\|1>\n` | Set gas (0–1), brake (0–1), steer direction |
| `STOP\n`                 | Kill all motors immediately                  |
| `STATUS\n`               | Report all sensor readings                   |
| `z\n`                    | Zero the encoder count                       |

Examples: `G0.50 B0.00 S1\n` = 50% gas, no brake, steer right (CW).

**Arduino → Host responses:**

Every loop (~20 Hz at 50 ms delay):

```
E<encoder_count> A<steering_angle_degrees>
```

Example: `E1200 A45.3` = encoder at count 1200, steering angle 45.3°.

Special responses: `CADDY READY`, `ALL STOPPED`, `ZEROED`,
`GAS:<pos> BRK:<pos> STR:<dir> ENC:<count> ANG:<angle>` (STATUS).

### Arduino Firmware Architecture (caddy_drive.ino)

**Encoder:**
- Model: BMQ-A38L6, **600 PPR**
- Only Channel A (pin 19/INT4) is used with `attachInterrupt(RISING)`
- Channel B (pin 18/INT5) read in ISR to determine direction
- Pin 20 (yellow) is declared but unused in firmware — likely an index channel
- Steering angle formula: `(encoderCount / 600.0) * 360.0 * (30.8 / 90.0)`
  - Note: the old code uses `ENCODER_WHEEL_DIA / STEERING_COL_DIA` as the gear
    ratio, which gives ~0.342. This is the **inverse** of what you'd expect —
    it converts encoder revolutions to steering wheel revolutions (correct).

**Gas/Brake — Closed-loop position control:**
- Potentiometer positions are read as normalized ADC values (0.0–1.0)
- User input (0.0–1.0) is mapped to a target pot position via linear
  interpolation between calibrated min/max
- A bang-bang controller with variable-speed PWM drives the actuator:
  - Error = target_position − current_position
  - If |error| < deadband → disable motor (target reached, set to −1 inactive)
  - Otherwise → extend or retract at a speed proportional to error magnitude
  - Speed is mapped: `constrain(map(|error|*1000, 0, 200, MIN_SPD, MAX_SPD))`
  - MIN_SPD = 90, MAX_SPD = 255 (PWM values)
- The `gasTarget`/`brkTarget` are set to −1 when inactive (motor disabled)

**Calibration values used in firmware (slightly different from raw pin specs):**

| Pedal | POS_MIN | POS_MAX | Deadband |
|-------|---------|---------|----------|
| Gas   | 0.05    | 0.72    | 0.008    |
| Brake | 0.42    | 0.90    | 0.025    |

**Steering — Open-loop direction control:**
- The Arduino only receives a direction: −1 (CCW/left), 0 (stop), +1 (CW/right)
- Fixed PWM speed of **200** (out of 255) when moving
- All angle tracking and closed-loop control happens in the Python host
- The Python host clamps steering at ±220° (controller) or ±240° (sunny)

**Measured steering limits** (from `steering_calibration.json`):
- Max right: +272.9°
- Max left: −276.8°

### Python Host Control Architecture

**Manual mode (caddy_controller.py):**
- PlayStation DualSense/DualShock 4 via pygame
- R2 trigger → gas (0–1, proportional, with deadzone + EMA smoothing)
- L2 trigger → brake (0–1, proportional, with deadzone + EMA smoothing)
- Left stick X → steer direction (−1/0/+1 after deadzone, no proportional)
- Gas is capped by `MAX_SPEED_MPH / CART_TOP_SPEED_MPH` ratio
- Triangle zeros encoder, Circle quits
- Controller disconnect → auto full-brake
- Steering angle persisted to `steering_state.json` across runs
- Commands sent at **20 Hz**

**AI mode (sunny_controller.py):**
- Runs sunnypilot vision + policy ONNX models on camera frames
- Extracts planned trajectory, computes desired steering angle from trajectory
  heading at ~8th waypoint: `heading_deg * AI_STEER_GAIN (20.0)`
- Closed-loop: compares desired angle to actual encoder angle, outputs −1/0/+1
  with a **5° deadband**
- AI gas/brake from model's predicted velocity
- Manual input immediately overrides AI
- EMA smoothing on AI steering (α=0.20) and gas/brake (α=0.15)

### Key Design Decisions to Preserve

1. **Steering is open-loop at the Arduino level.** The Arduino just applies
   a fixed PWM in a direction. All position control is done in Python using
   encoder feedback. This keeps the Arduino firmware simple.

2. **Gas/brake are closed-loop at the Arduino level.** The Arduino drives
   actuators to a target pot position autonomously. The host just says
   "go to 50% gas" and the Arduino handles it.

3. **The encoder is zeroed on startup** via `z\n` command. The Python host
   tracks absolute angle by adding a persisted offset.

4. **Safety: controller disconnect = full brake.** This is implemented in
   both the pygame event loop and as a serial fallback.

5. **50 ms loop delay** in the Arduino. This gives ~20 Hz control rate,
   matching the Python-side send rate.
