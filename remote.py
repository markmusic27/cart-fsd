#!/usr/bin/env python3
"""Remote control for the golf cart using a DualSense / PS5 controller.

This is intentionally tuned to feel like the legacy `caddy_controller.py`:

- R2: gas (smoothed, proportional)
- L2: brake (smoothed, proportional)
- Left stick X: steering direction (-1 / 0 / +1 open-loop)
- Triangle: zero steering encoder
- Cross: software e-stop
- Circle: quit

Example
-------
    uv run remote.py /dev/cu.usbmodem20401
    uv run remote.py /dev/cu.usbmodem20401 --skip-upload
"""

from __future__ import annotations

import argparse
import time

import pygame

from cart import CartConfig, CartController
from cart.config import (
    default_gas_limit_percent,
    default_steering_max_left_deg,
    default_steering_max_right_deg,
)
from firmware.upload_firmware import upload_firmware


SEND_HZ = 20
DEADZONE = 0.15
STEER_DEADZONE = 0.15
STEER_SENSITIVITY = 1.0
STEER_SPEED_SCALE = 0.8
SMOOTHING = 0.3
STEER_SMOOTHING = 1.0
WINDOW_WIDTH = 900
WINDOW_HEIGHT = 220

# SDL / pygame mappings commonly seen for DualSense on macOS / Bluetooth.
AXIS_LEFT_X = 0
AXIS_L2 = 4
AXIS_R2 = 5

BTN_CROSS = 1
BTN_CIRCLE = 2
BTN_TRIANGLE = 3


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def lerp(current: float, target: float, alpha: float) -> float:
    return current + alpha * (target - current)


def draw_bar(
    surface: pygame.Surface,
    font: pygame.font.Font,
    x: int,
    y: int,
    w: int,
    h: int,
    value: float,
    color: tuple[int, int, int],
    label: str,
    suffix: str = "",
) -> None:
    pygame.draw.rect(surface, (60, 60, 60), (x, y, w, h))
    fill_w = int(w * max(0.0, min(1.0, value)))
    pygame.draw.rect(surface, color, (x, y, fill_w, h))
    pygame.draw.rect(surface, (200, 200, 200), (x, y, w, h), 1)
    text = font.render(f"{label} {suffix}", True, (255, 255, 255))
    surface.blit(text, (x, y - 26))


def draw_steering(
    surface: pygame.Surface,
    font: pygame.font.Font,
    x: int,
    y: int,
    w: int,
    h: int,
    steer_norm: float,
    wheel_angle: float,
) -> None:
    pygame.draw.rect(surface, (60, 60, 60), (x, y, w, h))
    center_x = x + w // 2
    indicator_w = max(4, int(w * 0.06))
    pos_x = center_x + int(steer_norm * (w // 2 - indicator_w))
    pygame.draw.rect(surface, (0, 180, 255), (pos_x, y, indicator_w, h))
    pygame.draw.line(surface, (200, 200, 200), (center_x, y), (center_x, y + h), 1)
    pygame.draw.rect(surface, (200, 200, 200), (x, y, w, h), 1)
    text = font.render(f"STEER {wheel_angle:+.1f} deg", True, (255, 255, 255))
    surface.blit(text, (x, y - 26))


def render_hud(
    screen: pygame.Surface,
    font: pygame.font.Font,
    status_font: pygame.font.Font,
    gas: float,
    brake: float,
    steer: int,
    wheel_angle: float,
    gas_cap: float,
    connected: bool,
    e_stop: bool,
) -> None:
    screen.fill((20, 20, 20))
    pad = 24
    bar_w = 240
    bar_h = 28
    row_y = 96

    mph = gas / gas_cap * 15.0 if gas_cap > 0 and gas > 0 else 0.0
    draw_bar(screen, font, pad, row_y, bar_w, bar_h, gas / gas_cap if gas_cap else 0.0, (0, 200, 80), "GAS", f"{mph:.0f} mph")
    draw_bar(screen, font, pad + bar_w + pad, row_y, bar_w, bar_h, brake, (220, 60, 60), "BRK", f"{brake:.0%}")
    draw_steering(screen, font, pad + 2 * (bar_w + pad), row_y, bar_w, bar_h, float(steer), wheel_angle)

    if e_stop:
        txt = status_font.render("E-STOP", True, (255, 50, 50))
        screen.blit(txt, (WINDOW_WIDTH - txt.get_width() - 24, 24))
    elif not connected:
        txt = status_font.render("NO CTRL", True, (255, 180, 50))
        screen.blit(txt, (WINDOW_WIDTH - txt.get_width() - 24, 24))

    pygame.display.flip()


def connect_controller() -> pygame.joystick.Joystick:
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No game controller found. Pair the PS5 controller first.")

    js = pygame.joystick.Joystick(0)
    js.init()
    return js


def main() -> int:
    parser = argparse.ArgumentParser(description="PS5 Bluetooth remote for the golf cart")
    parser.add_argument("port", help="Arduino serial port, e.g. /dev/cu.usbmodem1201")
    parser.add_argument(
        "--gas-limit",
        type=int,
        default=None,
        help="Override gas limit percent (0-100). If omitted, reads settings/control_limits.json.",
    )
    parser.add_argument("--brake-cap", type=float, default=1.0, help="Cap brake travel from 0.0 to 1.0")
    parser.add_argument(
        "--steer-right-limit",
        type=float,
        default=None,
        help="Override max right steering angle in degrees. If omitted, reads settings/control_limits.json.",
    )
    parser.add_argument(
        "--steer-left-limit",
        type=float,
        default=None,
        help="Override max left steering angle in degrees (negative). If omitted, reads settings/control_limits.json.",
    )
    parser.add_argument(
        "--skip-upload",
        action="store_true",
        help="Skip firmware compile/upload before connecting",
    )
    parser.add_argument(
        "--safety",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable brake-on-disconnect and brake-on-exit behavior (default: true)",
    )
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Cart Remote")
    font = pygame.font.SysFont("monospace", 18)
    status_font = pygame.font.SysFont("monospace", 24, bold=True)

    try:
        joystick = connect_controller()
    except RuntimeError as exc:
        print(exc)
        return 1

    connected = True

    gas_limit_percent = default_gas_limit_percent() if args.gas_limit is None else max(0, min(100, args.gas_limit))
    gas_cap = gas_limit_percent / 100.0
    steer_right_limit = default_steering_max_right_deg() if args.steer_right_limit is None else max(0.0, args.steer_right_limit)
    steer_left_limit = default_steering_max_left_deg() if args.steer_left_limit is None else min(0.0, args.steer_left_limit)

    print(f"Controller: {joystick.get_name()}")
    print("Triangle = zero encoder | Cross = e-stop | Circle = quit")
    print(f"Safety: {'ON' if args.safety else 'OFF'}")
    print(f"Gas limit: {gas_limit_percent}%")
    print(f"Steering limits: left {steer_left_limit:.1f} deg, right {steer_right_limit:.1f} deg")

    config = CartConfig(
        gas_max_position=gas_cap,
        brake_max_position=clamp(args.brake_cap, 0.0, 1.0),
        steering_max_angle=max(abs(steer_left_limit), abs(steer_right_limit)),
        steering_max_right_angle=steer_right_limit,
        steering_max_left_angle=steer_left_limit,
        safety=args.safety,
    )

    if not args.skip_upload:
        try:
            upload_firmware(args.port)
            time.sleep(2.0)
        except Exception as exc:
            print(f"Firmware upload failed: {exc}")
            return 1

    ctl = CartController(args.port, config)
    gas = 0.0
    brake = 0.0
    steer = 0
    smooth_lx = 0.0
    smooth_gas = 0.0
    smooth_brake = 0.0

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
                    brake = 1.0 if config.safety else 0.0
                    steer = 0
                    smooth_lx = 0.0
                    smooth_gas = 0.0
                    smooth_brake = 0.0
                    ctl.set_gas(gas)
                    ctl.set_brake(brake)
                    ctl.set_steering_open_loop(0.0)
                    if config.safety:
                        print("\nController disconnected. Applying full brake.")
                    else:
                        print("\nController disconnected. Safety disabled; no brake applied.")
                elif event.type == pygame.JOYDEVICEADDED:
                    if not connected:
                        joystick = connect_controller()
                        connected = True
                        gas = 0.0
                        brake = 0.0
                        steer = 0
                        smooth_lx = 0.0
                        smooth_gas = 0.0
                        smooth_brake = 0.0
                        print(f"\nController reconnected: {joystick.get_name()}")

            if connected:
                raw_gas = joystick.get_axis(AXIS_R2)
                raw_brake = joystick.get_axis(AXIS_L2)
                raw_lx = joystick.get_axis(AXIS_LEFT_X) * STEER_SENSITIVITY

                smooth_gas = lerp(smooth_gas, (raw_gas + 1.0) / 2.0, SMOOTHING)
                smooth_brake = lerp(smooth_brake, (raw_brake + 1.0) / 2.0, SMOOTHING)
                smooth_lx = lerp(smooth_lx, raw_lx, STEER_SMOOTHING)

                if smooth_gas > DEADZONE:
                    gas = min(1.0, (smooth_gas - DEADZONE) / (1.0 - DEADZONE)) * config.gas_max_position
                else:
                    gas = 0.0

                if smooth_brake > DEADZONE:
                    brake = min(1.0, (smooth_brake - DEADZONE) / (1.0 - DEADZONE)) * config.brake_max_position
                else:
                    brake = 0.0

                if smooth_lx < -STEER_DEADZONE:
                    steer = -1
                elif smooth_lx > STEER_DEADZONE:
                    steer = 1
                else:
                    steer = 0

                wheel_angle = ctl.state.steering_angle
                if wheel_angle >= config.steering_max_right_angle and steer > 0:
                    steer = 0
                elif wheel_angle <= config.steering_max_left_angle and steer < 0:
                    steer = 0

                ctl.set_gas(gas)
                ctl.set_brake(brake)
                ctl.set_steering_open_loop(float(steer) * STEER_SPEED_SCALE)

            wheel_angle = ctl.state.steering_angle
            render_hud(
                screen,
                font,
                status_font,
                gas,
                brake,
                steer,
                wheel_angle,
                config.gas_max_position,
                connected,
                ctl.state.e_stop_active,
            )
            clock.tick(SEND_HZ)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if config.safety:
            ctl.stop()
            time.sleep(0.1)
        else:
            ctl.set_gas(0.0)
            ctl.set_brake(0.0)
            ctl.set_steering_open_loop(0.0)
            time.sleep(0.05)
        ctl.close()
        pygame.quit()
        print("\nDisconnected.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
