#!/usr/bin/env python3
"""Remote control for the golf cart using a DualSense / PS5 controller.

Controls
--------
- R2: gas (0.0 to 1.0)
- L2: brake (0.0 to 1.0)
- Left stick X: steering target angle
- Triangle: zero steering encoder
- Cross: software e-stop
- Circle: quit

Example
-------
    uv run remote.py /dev/cu.usbmodem1201
    uv run remote.py /dev/cu.usbmodem1201 --gas-cap 0.5 --max-steer-angle 180
"""

from __future__ import annotations

import argparse
import sys
import time

import pygame

from cart import CartConfig, CartController


SEND_HZ = 60
TRIGGER_DEADZONE = 0.02
STEER_DEADZONE = 0.08

# SDL / pygame mappings commonly seen for DualSense on macOS / Bluetooth.
AXIS_LEFT_X = 0
AXIS_L2 = 4
AXIS_R2 = 5

BTN_CROSS = 1
BTN_CIRCLE = 2
BTN_TRIANGLE = 3


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def trigger_to_unit(raw: float) -> float:
    """Map trigger axis from [-1, 1] to [0, 1]."""
    value = (raw + 1.0) / 2.0
    if value < TRIGGER_DEADZONE:
        return 0.0
    return clamp((value - TRIGGER_DEADZONE) / (1.0 - TRIGGER_DEADZONE), 0.0, 1.0)


def stick_to_steering(raw: float, max_angle: float) -> float:
    """Map left-stick X from [-1, 1] to steering angle in degrees."""
    if abs(raw) < STEER_DEADZONE:
        return 0.0
    signed = (abs(raw) - STEER_DEADZONE) / (1.0 - STEER_DEADZONE)
    signed *= 1.0 if raw >= 0 else -1.0
    return clamp(signed, -1.0, 1.0) * max_angle


def print_status(ctl: CartController, gas: float, brake: float, steer: float) -> None:
    state = ctl.state
    estop = " ESTOP" if state.e_stop_active else ""
    print(
        f"\r"
        f"cmd G:{gas:0.2f} B:{brake:0.2f} A:{steer:+06.1f}  "
        f"fb G:{state.gas_position:0.2f} B:{state.brake_position:0.2f} A:{state.steering_angle:+06.1f}"
        f"{estop}    ",
        end="",
        flush=True,
    )


def connect_controller() -> pygame.joystick.Joystick:
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No game controller found. Pair the PS5 controller first.")

    js = pygame.joystick.Joystick(0)
    js.init()
    return js


def main() -> int:
    parser = argparse.ArgumentParser(description="PS5 Bluetooth remote for the golf cart")
    parser.add_argument("port", help="Arduino serial port, e.g. /dev/cu.usbmodem1201")
    parser.add_argument("--gas-cap", type=float, default=1.0, help="Cap gas travel from 0.0 to 1.0")
    parser.add_argument("--brake-cap", type=float, default=1.0, help="Cap brake travel from 0.0 to 1.0")
    parser.add_argument(
        "--max-steer-angle",
        type=float,
        default=180.0,
        help="Max steering target angle in degrees for full stick deflection",
    )
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()

    try:
        joystick = connect_controller()
    except RuntimeError as exc:
        print(exc)
        return 1

    connected = True

    print(f"Controller: {joystick.get_name()}")
    print("Triangle = zero encoder | Cross = e-stop | Circle = quit")

    config = CartConfig(
        gas_max_position=clamp(args.gas_cap, 0.0, 1.0),
        brake_max_position=clamp(args.brake_cap, 0.0, 1.0),
        steering_max_angle=max(0.0, args.max_steer_angle),
    )

    ctl = CartController(args.port, config)
    gas = 0.0
    brake = 0.0
    steer = 0.0

    try:
        clock = pygame.time.Clock()
        running = True

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_CIRCLE:
                        running = False
                    elif event.button == BTN_TRIANGLE:
                        ctl.zero_encoder()
                        print("\nSteering encoder zeroed.")
                    elif event.button == BTN_CROSS:
                        ctl.stop()
                        print("\nSoftware e-stop sent.")
                elif event.type == pygame.JOYDEVICEREMOVED:
                    connected = False
                    gas = 0.0
                    brake = 1.0
                    steer = 0.0
                    ctl.set_gas(gas)
                    ctl.set_brake(brake)
                    ctl.set_steering(steer)
                    print("\nController disconnected. Applying full brake.")
                elif event.type == pygame.JOYDEVICEADDED:
                    if not connected:
                        joystick = connect_controller()
                        connected = True
                        print(f"\nController reconnected: {joystick.get_name()}")

            pygame.event.pump()

            if connected:
                raw_gas = joystick.get_axis(AXIS_R2)
                raw_brake = joystick.get_axis(AXIS_L2)
                raw_steer = joystick.get_axis(AXIS_LEFT_X)

                gas = trigger_to_unit(raw_gas)
                brake = trigger_to_unit(raw_brake)
                steer = stick_to_steering(raw_steer, config.steering_max_angle)

                ctl.set_gas(gas)
                ctl.set_brake(brake)
                ctl.set_steering(steer)

            print_status(ctl, gas, brake, steer)
            clock.tick(SEND_HZ)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ctl.stop()
        time.sleep(0.1)
        ctl.close()
        pygame.quit()
        print("\nDisconnected.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
