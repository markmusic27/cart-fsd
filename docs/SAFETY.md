# Safety Systems

This document describes the three independent safety layers that can bring the
cart to a controlled stop.  Any one layer acting alone is sufficient.

## Safety Layers Overview

| Layer | Trigger | Reaction time | Depends on |
|-------|---------|---------------|------------|
| 1. Hardware E-Stop | Physical button pressed | Microseconds (ISR) | Arduino power + firmware |
| 2. Watchdog Timeout | No serial command for 250 ms | ≤250 ms | Arduino firmware |
| 3. Software E-Stop | Host sends `E\n` command | ~12–25 ms | Serial link + Arduino |

All three layers converge to the same **safe state**:
- Gas target → 0 (retract actuator, release throttle)
- Brake target → 1.0 (extend actuator, full brake)
- Steering → hold current angle (don't jerk)

---

## Layer 1: Hardware E-Stop Button

### Component

Mushroom-head latching normally-closed (NC) emergency stop button.
Standard industrial type — push to stop (button latches down), twist to
release (button pops back up, contacts close).

### Wiring

**Two wires only.  No external resistors, capacitors, or power supply needed.**

```
Arduino Mega Pin 21 (INT2) ────── NC E-Stop Button ────── Arduino GND
```

The Arduino's internal pull-up resistor (enabled via `INPUT_PULLUP` in
firmware) provides the reference voltage.  Current draw is negligible
(< 0.5 mA through the internal ~20 kΩ pull-up).

### Circuit Schematic Description

For generating a schematic in KiCad, Fritzing, or similar:

```
Component: Arduino Mega 2560
    Pin 21 (Digital, INT2) ──── wire ──── Terminal 1 of E-Stop Switch
    GND                    ──── wire ──── Terminal 2 of E-Stop Switch

Component: E-Stop Switch
    Type: SPST, Normally Closed (NC), Mushroom-Head Latching
    Terminal 1 ──── Arduino Pin 21
    Terminal 2 ──── Arduino GND

Arduino firmware configuration:
    pinMode(21, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(21), eStopISR, RISING);
```

### Electrical States

| Physical state | Button contacts | Pin 21 voltage | Pin 21 reading | System state |
|----------------|-----------------|----------------|----------------|--------------|
| Normal operation | Closed (NC) | ~0 V (grounded through button) | LOW | Normal |
| Button pressed | Open (latched) | ~5 V (internal pull-up) | HIGH | **E-STOP** |
| Wire broken | Open circuit | ~5 V (internal pull-up) | HIGH | **E-STOP** |
| Wire shorted | Closed | ~0 V | LOW | Normal |
| Connector loose | Open circuit | ~5 V (internal pull-up) | HIGH | **E-STOP** |

This is a **fail-safe** design: any failure mode (broken wire, loose
connector, corroded contact) results in an open circuit, which triggers the
e-stop.  The only state that allows normal operation is an intact, closed
circuit.

### Firmware Behavior

1. **ISR (Interrupt Service Routine):** Fires on RISING edge of pin 21
   (LOW → HIGH = button pressed or wire broken).  Sets `eStopActive = true`
   in microseconds — no polling delay.

2. **Main loop:** When `eStopActive` is true, overrides all targets to safe
   state every loop iteration (100 Hz).  Target commands (`T`) from the host
   are ignored while e-stop is active.

3. **Recovery:** When the button is twisted to release (contacts close, pin
   returns LOW), the e-stop flag is **not** automatically cleared.  It only
   clears when a new serial command arrives AND pin 21 reads LOW.  This
   prevents accidental resumption — the operator must deliberately release
   the button AND the host must send a command.

4. **Boot check:** On startup, firmware reads pin 21.  If HIGH (button
   pressed at power-on), the system starts in e-stop mode.

### State Report

When e-stop is active, the Arduino includes `ES1` in its state reports:

```
S G0.050 B0.900 A0.0 GS1 BS1 AS1 ES1
```

The Python `CartState.e_stop_active` field exposes this to the host so the
UI can display e-stop status.

---

## Layer 2: Watchdog / Heartbeat Timeout

### How It Works

The Python `CartController` sends a target command (`T G... B... A...\n`)
every 20 ms (50 Hz) as a heartbeat, even when targets haven't changed.

The Arduino tracks `lastCommandTime = millis()` and checks every loop:

```c
if (millis() - lastCommandTime > 250) {
    // No command for 250 ms — host is presumed dead
    applySafeState();
}
```

### What Triggers the Watchdog

| Failure | Time to detect |
|---------|---------------|
| USB cable disconnected | ≤ 250 ms |
| Laptop crash / freeze | ≤ 250 ms |
| Python script crash | ≤ 250 ms |
| Laptop battery dies | ≤ 250 ms |
| Serial driver failure | ≤ 250 ms |

### Recovery

The watchdog resets automatically when a new command arrives.  No special
recovery sequence needed — just reconnect and start sending commands.

---

## Layer 3: Software E-Stop

The host can trigger an e-stop programmatically by sending `E\n` over serial
or calling `CartController.stop()`.

```python
ctl.stop()   # sends E\n, sets local targets to gas=0, brake=1, steering=0
```

The Arduino responds with `ESTOP` and applies the safe state.

Recovery: send any `T` command to resume normal operation.

---

## Safety State Machine

```
                  ┌──────────┐
       power on ──► BOOT     │
                  │ (check   │
                  │  pin 21) │
                  └────┬─────┘
                       │
            pin 21 LOW │         pin 21 HIGH
            (button    │         (button pressed
             released) ▼          or wire broken)
                  ┌──────────┐        ┌──────────┐
   commands ─────►│ NORMAL   │───────►│ E-STOP   │
   arriving       │          │ ISR    │ (safe    │
                  │ PD loops │ fires  │  state)  │
                  │ active   │        │          │
                  └────┬─────┘        └────┬─────┘
                       │                   │
         no command    │    button released │
         for 250 ms    │    + new command   │
                       ▼                   │
                  ┌──────────┐             │
                  │ WATCHDOG │             │
                  │ (safe    │◄────────────┘
                  │  state)  │  (if watchdog tripped
                  └────┬─────┘   during e-stop)
                       │
         new command   │
         arrives       │
                       ▼
                  ┌──────────┐
                  │ NORMAL   │
                  └──────────┘
```

---

## Bill of Materials (E-Stop Circuit)

| Qty | Component | Notes |
|-----|-----------|-------|
| 1 | Mushroom-head NC latching e-stop button | Any standard 22 mm panel-mount industrial e-stop (e.g., Schneider XB2-BS542) |
| 2 | Hookup wire (22 AWG recommended) | Pin 21 to button terminal 1; GND to button terminal 2 |
| 0 | Resistors / capacitors / external power | **None needed** — uses Arduino internal pull-up |

Total added cost: ~$5–10 for the button.

---

## Future Improvement: Hardware Power Cut (Layer 0)

For maximum safety, a fourth layer could physically cut motor power
independent of the Arduino firmware:

```
Cart Battery → Buck Converter → Relay (NC contacts) → Fuse Box → Motors
                                   ↑
                              Relay coil powered by
                              a separate circuit through
                              the same NC e-stop button
```

With this design, pressing the e-stop button simultaneously:
1. Triggers the Arduino ISR (Layer 1, firmware response)
2. Opens the relay contacts (Layer 0, hardware power cut)

This would stop the motors even if the Arduino firmware is completely
frozen.  Not implemented yet — the software-based e-stop is sufficient for
development and testing at low speeds.
