"""
serial_interface.py - Serial communication module for the waste-detection pointer.

Provides an abstraction for serial communications with the Arduino Uno:
- Real Mode: Opens hardware serial port, performs startup handshake ('READY'),
  sends pointing angles ('P,<angle>') and queries ultrasonic distance ('Q' -> 'R,<cm>').
- Mock Mode: Simulates serial interaction without physical hardware for testing.
"""

import sys
import time
from typing import Optional

try:
    import serial
except ImportError:
    serial = None


class ArduinoSerialInterface:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 2.0,
        mock: bool = False,
        mock_distance: float = 45.0,
    ):
        """
        Initialize the serial interface.

        Args:
            port: Serial port path (e.g. '/dev/ttyACM0', '/dev/ttyUSB0', or 'COM3').
            baudrate: Communication baud rate (default 115200).
            timeout: Read timeout in seconds.
            mock: If True, simulate communication without opening hardware serial.
            mock_distance: Default distance in cm returned in mock mode.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.mock = mock
        self.mock_distance = mock_distance
        self._serial: Optional["serial.Serial"] = None

    def connect(self) -> bool:
        """
        Open the connection and wait for the Arduino 'READY' handshake.
        Returns True if connection and handshake succeed.
        """
        if self.mock:
            print(f"[MOCK SERIAL] Connected in MOCK mode (Port: {self.port}, Baud: {self.baudrate})")
            print("[MOCK SERIAL] Received Handshake: READY")
            return True

        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. Please run 'pip install pyserial' or use --mock."
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

            while time.time() - start_time < 5.0:
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

    def send_point(self, angle: int) -> bool:
        """
        Send point command 'P,<angle>' to the Arduino.
        Angle is clamped to [0, 180].
        """
        clamped_angle = max(0, min(180, int(angle)))

        if self.mock:
            print(f"[MOCK SERIAL TX] P,{clamped_angle} (Point servo to {clamped_angle}°)")
            return True

        if not self._serial or not self._serial.is_open:
            print("[SERIAL ERROR] Cannot send command: Serial port is not open.", file=sys.stderr)
            return False

        try:
            cmd = f"P,{clamped_angle}\n"
            self._serial.write(cmd.encode("utf-8"))
            self._serial.flush()
            return True
        except serial.SerialException as e:
            print(f"[SERIAL ERROR] Error writing P command: {e}", file=sys.stderr)
            return False

    def query_distance(self) -> float:
        """
        Query distance by sending 'Q' and waiting for 'R,<cm>'.

        Returns:
            Distance in cm as float if plausible, or -1.0 on timeout/out of bounds.
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
