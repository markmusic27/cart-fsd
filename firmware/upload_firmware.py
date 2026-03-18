#!/usr/bin/env python3
"""Compile and upload the Arduino firmware sketch with arduino-cli.

Examples
--------
    uv run firmware/upload_firmware.py /dev/cu.usbmodem20401
    uv run firmware/upload_firmware.py /dev/cu.usbmodem20401 --fqbn arduino:avr:mega
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


DEFAULT_FQBN = "arduino:avr:mega"
SKETCH_DIR = Path(__file__).resolve().parent


def run_checked(command: list[str]) -> None:
    subprocess.run(command, check=True)


def upload_firmware(port: str, fqbn: str = DEFAULT_FQBN) -> None:
    if shutil.which("arduino-cli") is None:
        raise RuntimeError("arduino-cli is not installed or not on PATH")

    if not SKETCH_DIR.exists():
        raise RuntimeError(f"Firmware sketch directory not found: {SKETCH_DIR}")

    print(f"Compiling firmware from {SKETCH_DIR} for {fqbn}...")
    run_checked(
        [
            "arduino-cli",
            "compile",
            "--fqbn",
            fqbn,
            str(SKETCH_DIR),
        ]
    )

    print(f"Uploading firmware to {port}...")
    run_checked(
        [
            "arduino-cli",
            "upload",
            "-p",
            port,
            "--fqbn",
            fqbn,
            str(SKETCH_DIR),
        ]
    )

    print("Firmware upload complete.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Compile and upload cart firmware")
    parser.add_argument("port", help="Arduino serial port, e.g. /dev/cu.usbmodem20401")
    parser.add_argument(
        "--fqbn",
        default=DEFAULT_FQBN,
        help=f"Board FQBN (default: {DEFAULT_FQBN})",
    )
    args = parser.parse_args()

    try:
        upload_firmware(args.port, args.fqbn)
    except subprocess.CalledProcessError as exc:
        print(f"Firmware upload failed with exit code {exc.returncode}.", file=sys.stderr)
        return exc.returncode
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
