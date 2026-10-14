"""
serial_interface.py - Serial communication module for the autonomous waste robot.

Provides an abstraction for serial communications with the Arduino Uno
driving a 4WD L298N chassis:
- Real Mode: Opens hardware serial port, performs startup handshake ('READY'),
  sends motor commands (F/B/TL/TR/S/V) and queries ultrasonic distance (Q -> R,<cm>).
- Mock Mode: Simulates serial interaction without physical hardware for testing.
  Mock distance shrinks with each forward burst to simulate approach.
"""

import glob
import sys
import time
from typing import Optional

try:
    import serial
except ImportError:
    serial = None


def find_arduino_port(timeout: float = 30.0) -> Optional[str]:
    """
    Auto-detect the Arduino serial port.

    Searches (in order):
      1. /dev/serial/by-id/*  (most reliable — udev symlinks)
      2. /dev/ttyACM*         (CDC ACM devices, typical for Uno)
      3. /dev/ttyUSB*         (USB-serial adapters)

    Retries every 2 s for up to `timeout` seconds.
    Returns the first matching port, or None if nothing is found.
    """
    patterns = [
        "/dev/serial/by-id/*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    deadline = time.time() + timeout
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        for pattern in patterns:
            ports = sorted(glob.glob(pattern))
            if ports:
                print(f"[SERIAL] Auto-detected port: {ports[0]} (attempt {attempt})")
                return ports[0]
        remaining = deadline - time.time()
        if remaining > 0:
            print(f"[SERIAL] No Arduino port found yet, retrying... ({remaining:.0f}s left)")
            time.sleep(min(2.0, remaining))
    return None


class ArduinoSerialInterface:
    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 115200,
        timeout: float = 2.0,
        mock: bool = False,
        mock_distance: float = 80.0,
    ):
        """
        Initialize the serial interface.

        Args:
            port: Serial port path. If None, auto-detect is used.
            baudrate: Communication baud rate (default 115200).
            timeout: Read timeout in seconds for command acknowledgements.
            mock: If True, simulate communication without opening hardware serial.
            mock_distance: Initial simulated ultrasonic distance (cm) in mock mode.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.mock = mock
        self.mock_distance = mock_distance
        self._serial: Optional["serial.Serial"] = None
        self._mock_commands: list[str] = []  # Recorded commands in mock mode

    def connect(self, auto_find_timeout: float = 30.0) -> bool:
        """
        Open the connection and wait for the Arduino 'READY' handshake.

        In real mode, if self.port is None, auto-detect the port first.
        Returns True if connection and handshake succeed.
        """
        if self.mock:
            port_str = self.port or "/dev/ttyMOCK"
            print(f"[MOCK SERIAL] Connected in MOCK mode (Port: {port_str}, Baud: {self.baudrate})")
            print("[MOCK SERIAL] Received Handshake: READY")
            return True

        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. Please run 'pip install pyserial' or use --mock."
            )

        # Auto-find the port if not specified
        if self.port is None:
            print(f"[SERIAL] Auto-detecting Arduino port (timeout {auto_find_timeout}s)...")
            self.port = find_arduino_port(timeout=auto_find_timeout)
            if self.port is None:
                raise RuntimeError(
                    "Could not auto-detect Arduino serial port. "
                    "Is the Arduino connected via USB? Try specifying --port manually."
                )

        try:
            print(f"[SERIAL] Connecting to Arduino on {self.port} at {self.baudrate} baud...")
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

            # Reset Arduino line (DTR toggle often resets Uno)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            # Wait for READY handshake from Arduino setup()
            print("[SERIAL] Waiting for Arduino 'READY' handshake...")
            start_time = time.time()
            handshake_received = False

            while time.time() - start_time < 10.0:
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        print(f"[SERIAL RX] {line}")
                    if "READY" in line:
                        handshake_received = True
                        break
                time.sleep(0.05)

            if handshake_received:
                print("[SERIAL] Handshake successful! Arduino is ready.")
                return True
            else:
                print("[SERIAL WARNING] No 'READY' handshake received within timeout; continuing anyway.")
                return True

        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Failed to connect to {self.port}: {e}", file=sys.stderr)
            raise

    # ---------------------------------------------------------------
    # Motion Commands — each sends a command string and waits for 'A'
    # ---------------------------------------------------------------

    def _send_motion(self, cmd: str) -> bool:
        """
        Send a motion command and wait for the 'A' acknowledgement.

        Returns True if 'A' was received within the timeout, False otherwise.
        """
        if self.mock:
            self._mock_commands.append(cmd)
            # Simulate distance shrinking on forward bursts
            if cmd.startswith("F,"):
                ms = int(cmd.split(",")[1])
                shrink = (ms / 300.0) * 5.0  # ~5 cm per 300 ms burst
                self.mock_distance = max(2.0, self.mock_distance - shrink)
            print(f"[MOCK SERIAL TX] {cmd}")
            print("[MOCK SERIAL RX] A")
            return True

        if not self._serial or not self._serial.is_open:
            print("[SERIAL ERROR] Cannot send command: Serial port is not open.", file=sys.stderr)
            return False

        try:
            self._serial.reset_input_buffer()
            self._serial.write(f"{cmd}\n".encode("utf-8"))
            self._serial.flush()

            # Wait for 'A' acknowledgement
            start_time = time.time()
            while time.time() - start_time < self.timeout:
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                    if line == "A":
                        return True
                    # Might receive R,<cm> from a Q that was in flight — ignore
                time.sleep(0.01)

            print(f"[SERIAL WARNING] No 'A' acknowledgement for '{cmd}' within {self.timeout}s.")
            return False

        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Error sending '{cmd}': {e}", file=sys.stderr)
            return False

    def send_forward(self, ms: int) -> bool:
        """Drive forward for `ms` milliseconds."""
        return self._send_motion(f"F,{int(ms)}")

    def send_backward(self, ms: int) -> bool:
        """Drive backward for `ms` milliseconds."""
        return self._send_motion(f"B,{int(ms)}")

    def send_turn_left(self, ms: int) -> bool:
        """Pivot left for `ms` milliseconds."""
        return self._send_motion(f"TL,{int(ms)}")

    def send_turn_right(self, ms: int) -> bool:
        """Pivot right for `ms` milliseconds."""
        return self._send_motion(f"TR,{int(ms)}")

    def send_stop(self) -> bool:
        """
        Send immediate stop command. No 'A' reply expected.
        """
        if self.mock:
            self._mock_commands.append("S")
            print("[MOCK SERIAL TX] S")
            return True

        if not self._serial or not self._serial.is_open:
            print("[SERIAL ERROR] Cannot send stop: Serial port is not open.", file=sys.stderr)
            return False

        try:
            self._serial.write(b"S\n")
            self._serial.flush()
            return True
        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Error sending stop: {e}", file=sys.stderr)
            return False

    def send_speed(self, value: int) -> bool:
        """
        Set motor PWM speed (0–255). No 'A' reply expected.
        """
        value = max(0, min(255, int(value)))

        if self.mock:
            self._mock_commands.append(f"V,{value}")
            print(f"[MOCK SERIAL TX] V,{value}")
            return True

        if not self._serial or not self._serial.is_open:
            print("[SERIAL ERROR] Cannot send speed: Serial port is not open.", file=sys.stderr)
            return False

        try:
            self._serial.write(f"V,{value}\n".encode("utf-8"))
            self._serial.flush()
            return True
        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Error sending speed: {e}", file=sys.stderr)
            return False

    # ---------------------------------------------------------------
    # Ultrasonic Query
    # ---------------------------------------------------------------

    def query_distance(self) -> float:
        """
        Query distance by sending 'Q' and waiting for 'R,<cm>'.

        Returns:
            Distance in cm as float if valid, or -1.0 on timeout/out of range.
        """
        if self.mock:
            print(f"[MOCK SERIAL TX] Q (Query distance)")
            print(f"[MOCK SERIAL RX] R,{self.mock_distance:.1f}")
            return self.mock_distance

        if not self._serial or not self._serial.is_open:
            print("[SERIAL ERROR] Cannot query: Serial port is not open.", file=sys.stderr)
            return -1.0

        try:
            # Clear stale bytes
            self._serial.reset_input_buffer()

            # Send Query command
            self._serial.write(b"Q\n")
            self._serial.flush()

            # Wait for response line R,<cm>
            start_time = time.time()
            while time.time() - start_time < self.timeout:
                if self._serial.in_waiting > 0:
                    line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    if line.startswith("R,"):
                        val_str = line[2:].strip()
                        try:
                            dist = float(val_str)
                            return dist
                        except ValueError:
                            print(f"[SERIAL WARNING] Malformed distance response: '{line}'")
                            return -1.0
                time.sleep(0.01)

            print("[SERIAL WARNING] Distance query timed out with no response.")
            return -1.0

        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Error querying distance: {e}", file=sys.stderr)
            return -1.0

    # ---------------------------------------------------------------
    # Lifecycle
    # ---------------------------------------------------------------

    def get_mock_commands(self) -> list[str]:
        """Return list of all commands sent in mock mode (for testing)."""
        return list(self._mock_commands)

    def close(self):
        """Close the serial port."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
                print("[SERIAL] Port closed.")
            except Exception:
                pass
        self._serial = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
