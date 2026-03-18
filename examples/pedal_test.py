#!/usr/bin/env python3
"""Pedal test — verify gas and brake actuators respond to targets.

Ramps gas from 0 to 50 %, holds, then returns to 0.  Repeats for brake.
Prints live state from the Arduino the whole time.

Usage:
    python examples/pedal_test.py /dev/cu.usbmodem1201
    python examples/pedal_test.py /dev/cu.usbmodem1201 --gas-cap 0.5
"""

from __future__ import annotations

import argparse
import sys
import time

from cart import CartConfig, CartController


def print_state(ctl: CartController) -> None:
    s = ctl.state
    es = " *** E-STOP ***" if s.e_stop_active else ""
    print(
        f"\r  GAS {s.gas_position:5.3f} {'✓' if s.gas_settled else '…'}  "
        f"BRK {s.brake_position:5.3f} {'✓' if s.brake_settled else '…'}  "
        f"STR {s.steering_angle:+6.1f}°{es}   ",
        end="",
        flush=True,
    )


def ramp_and_hold(
    ctl: CartController,
    label: str,
    set_fn,
    target: float,
    ramp_time: float = 1.0,
    hold_time: float = 2.0,
    return_time: float = 1.0,
) -> None:
    """Ramp a pedal to *target*, hold, then return to 0."""
    steps = int(ramp_time / 0.05)
    print(f"\n--- {label}: ramp to {target:.2f} ---")

    for i in range(1, steps + 1):
        set_fn(target * i / steps)
        time.sleep(0.05)
        print_state(ctl)

    print(f"\n--- {label}: hold at {target:.2f} for {hold_time}s ---")
    t0 = time.monotonic()
    while time.monotonic() - t0 < hold_time:
        time.sleep(0.05)
        print_state(ctl)

    print(f"\n--- {label}: return to 0 ---")
    for i in range(steps, -1, -1):
        set_fn(target * i / steps)
        time.sleep(0.05)
        print_state(ctl)

    time.sleep(0.5)
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description="Pedal test")
    parser.add_argument("port", help="Serial port, e.g. /dev/cu.usbmodem1201")
    parser.add_argument(
        "--gas-cap", type=float, default=0.5,
        help="Max gas position (0-1). Default 0.5 for safety.",
    )
    parser.add_argument(
        "--brake-cap", type=float, default=0.8,
        help="Max brake position (0-1). Default 0.8.",
    )
    args = parser.parse_args()

    config = CartConfig(
        gas_max_position=args.gas_cap,
        brake_max_position=args.brake_cap,
    )

    print(f"Connecting to {args.port}...")
    print(f"Gas cap: {args.gas_cap}, Brake cap: {args.brake_cap}")
    ctl = CartController(args.port, config)

    try:
        print("\nStarting pedal test.  Press Ctrl+C to abort (will e-stop).\n")
        time.sleep(1.0)

        ramp_and_hold(ctl, "GAS", ctl.set_gas, target=1.0)
        ramp_and_hold(ctl, "BRAKE", ctl.set_brake, target=1.0)

        print("\n=== Test complete ===")
    except KeyboardInterrupt:
        print("\n\nAborted!")
    finally:
        ctl.stop()
        time.sleep(0.5)
        ctl.close()
        print("Done.")


if __name__ == "__main__":
    main()
