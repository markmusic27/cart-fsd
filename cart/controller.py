"""High-level cart controller — the single entry-point for driving the cart.

Usage::

    from cart import CartController, CartConfig

    cfg = CartConfig(gas_max_position=0.67)          # cap at ~10 mph
    ctl = CartController("/dev/cu.usbmodem1201", cfg)

    ctl.set_gas(0.5)          # 50 % of capped range
    ctl.set_brake(0.0)
    print(ctl.state)          # latest CartState

    ctl.stop()                # software e-stop
    ctl.close()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable

import serial

from cart.config import (
    CartConfig,
    CartState,
    DEFAULT_BAUD_RATE,
    SEND_INTERVAL_S,
)
from cart.protocol import (
    decode_state,
    encode_estop,
    encode_full_config,
    encode_targets,
    encode_zero_encoder,
)

logger = logging.getLogger(__name__)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class CartController:
    """Thread-safe interface to the Arduino cart controller.

    A background thread continuously sends the current target command at
    ``SEND_INTERVAL_S`` (50 Hz) as a heartbeat, and reads incoming state
    reports from the Arduino.  Calling any ``set_*`` method wakes the
    thread immediately via a ``threading.Event`` so the new target is
    transmitted without waiting for the next tick.
    """

    def __init__(
        self,
        port: str,
        config: CartConfig | None = None,
        baud: int = DEFAULT_BAUD_RATE,
        on_state: Callable[[CartState], None] | None = None,
    ) -> None:
        self._config = config or CartConfig()
        self._on_state = on_state

        # Current targets (protected by _lock)
        self._lock = threading.Lock()
        self._gas_target: float = 0.0
        self._brake_target: float = 0.0
        self._steering_target: float = 0.0
        self._steering_command: float | None = None

        # Latest state from Arduino
        self._state = CartState()
        self._state_lock = threading.Lock()

        # Send-on-change event
        self._change_event = threading.Event()

        # Serial connection
        logger.info("Connecting to %s at %d baud", port, baud)
        self._ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)  # wait for Arduino reset after serial open
        self._drain_startup()
        self._send_initial_config()

        # Background IO thread
        self._running = True
        self._thread = threading.Thread(
            target=self._io_loop, name="cart-io", daemon=True
        )
        self._thread.start()
        logger.info("CartController ready")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_gas(self, position: float) -> None:
        """Set gas pedal target (0.0 = released, 1.0 = max configured travel)."""
        with self._lock:
            self._gas_target = _clamp(position, 0.0, self._config.gas_max_position)
        self._change_event.set()

    def set_brake(self, position: float) -> None:
        """Set brake pedal target (0.0 = released, 1.0 = max configured travel)."""
        with self._lock:
            self._brake_target = _clamp(position, 0.0, self._config.brake_max_position)
        self._change_event.set()

    def set_steering(self, angle: float) -> None:
        """Set steering target in degrees at the steering column.

        Positive = right, negative = left.
        """
        limit = self._config.steering_max_angle
        with self._lock:
            self._steering_target = _clamp(angle, -limit, limit)
            self._steering_command = None
        self._change_event.set()

    def set_steering_open_loop(self, command: float) -> None:
        """Set normalized open-loop steering motor command in [-1.0, 1.0].

        -1.0 = full left motor command
         0.0 = stop steering motor
        +1.0 = full right motor command
        """
        with self._lock:
            self._steering_command = _clamp(command, -1.0, 1.0)
        self._change_event.set()

    def stop(self) -> None:
        """Software emergency stop — immediately commands Arduino to safe state."""
        with self._lock:
            self._gas_target = 0.0
            self._brake_target = 1.0
            self._steering_target = 0.0
            self._steering_command = 0.0
        self._safe_write(encode_estop())
        self._change_event.set()

    def zero_encoder(self) -> None:
        """Zero the steering encoder on the Arduino."""
        self._safe_write(encode_zero_encoder())

    @property
    def state(self) -> CartState:
        """Most recent state snapshot (non-blocking read)."""
        with self._state_lock:
            return self._state

    @property
    def config(self) -> CartConfig:
        return self._config

    def update_config(self, config: CartConfig) -> None:
        """Send a new configuration to the Arduino."""
        self._config = config
        self._send_config(config)

    def close(self) -> None:
        """Stop the IO thread and close the serial port."""
        self._running = False
        self._change_event.set()  # unblock the thread if sleeping
        self._thread.join(timeout=2.0)
        try:
            if self._config.safety:
                self._safe_write(encode_estop())
            else:
                self._safe_write(encode_targets(0.0, 0.0, 0.0))
                time.sleep(0.02)
            self._ser.close()
        except Exception:
            pass
        logger.info("CartController closed")

    # ------------------------------------------------------------------
    # Background IO loop
    # ------------------------------------------------------------------

    def _io_loop(self) -> None:
        while self._running:
            self._send_current_targets()
            self._read_state()
            self._change_event.wait(timeout=SEND_INTERVAL_S)
            self._change_event.clear()

    def _send_current_targets(self) -> None:
        with self._lock:
            gas = self._gas_target
            brake = self._brake_target
            angle = self._steering_target
            command = self._steering_command
        self._safe_write(encode_targets(gas, brake, angle, command))

    def _read_state(self) -> None:
        try:
            while self._ser.in_waiting:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                parsed = decode_state(line)
                if parsed is not None:
                    with self._state_lock:
                        self._state = parsed
                    if self._on_state is not None:
                        self._on_state(parsed)
                elif line:
                    logger.debug("Arduino: %s", line)
        except OSError as exc:
            logger.warning("Serial read error: %s", exc)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _drain_startup(self) -> None:
        """Read and log any boot messages from the Arduino."""
        try:
            while self._ser.in_waiting:
                line = self._ser.readline().decode(errors="replace").strip()
                if line:
                    logger.info("Arduino startup: %s", line)
        except OSError:
            pass

    def _send_initial_config(self) -> None:
        """Send full config + zero encoder on first connect."""
        self._safe_write(encode_zero_encoder())
        time.sleep(0.05)
        self._send_config(self._config)

    def _send_config(self, config: CartConfig) -> None:
        for cmd in encode_full_config(config):
            self._safe_write(cmd)
            time.sleep(0.005)  # small gap so Arduino can parse each line

    def _safe_write(self, data: bytes) -> bool:
        try:
            self._ser.write(data)
            return True
        except OSError as exc:
            logger.warning("Serial write error: %s", exc)
            return False
