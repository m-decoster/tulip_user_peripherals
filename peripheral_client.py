"""Client to interface with peripherals connected to the Teensy microcontroller."""

import time
from enum import Enum

import serial


class StatusLed(Enum):
    POWER = 0
    BATTERY = 1
    WARNING = 2
    ARM = 3
    WHEELS_ENABLED = 4


class PeripheralClient:
    """Client to interface with peripherals connected to the Teensy microcontroller."""

    def __init__(self, port, baud_rate):
        """Initialize the client and connect to the server."""
        self._ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)
        if not self.ping():
            raise RuntimeError(
                "Could not establish connection to peripheral server. It the microcontroller connected?"
            )

    def _transceive(self, command):
        """Send a command to the server and return the response."""
        # Send command to server
        self._ser.write((command + "\n").encode("utf-8"))

        # Read response from server
        start_time = time.time()
        while True:
            if self._ser.in_waiting > 0:
                response = self._ser.readline().decode("utf-8").strip()
                if response:
                    return response
            else:
                if time.time() - start_time > 2.0:
                    return "__ERROR__"
                time.sleep(0.01)

    def ping(self):
        """Check if the server is reachable."""
        res = self._transceive("PING")
        return res == "PONG"

    def get_flow(self):
        """Get the flow sensor readings."""
        res = self._transceive("FLOW")
        tokens = res.split(",")
        x1 = int(tokens[0])
        y1 = int(tokens[1])
        x2 = int(tokens[2])
        y2 = int(tokens[3])
        return x1, y1, x2, y2

    def get_orientation(self):
        res = self._transceive("BNO")
        tokens = res.split(",")
        x = float(tokens[0])
        y = float(tokens[1])
        z = float(tokens[2])
        return x, y, z

    def set_leds_boot(self):
        """Set the LEDs to boot mode."""
        res = self._transceive("LED BOOT")
        return res == "OK"

    def set_leds_idle(self):
        """Set the LEDs to idle mode."""
        res = self._transceive("LED IDLE")
        return res == "OK"

    def set_leds_active(self, angle: float, velocity: float):
        """Set the LEDs to active mode, depending on the current angle and velocity."""
        angle = angle % (2 * 3.1415)
        res = self._transceive(f"LED ACTIVE {angle} {velocity}")
        return res == "OK"

    def set_leds_error(self):
        """Set the LEDs to error mode."""
        res = self._transceive("LED ERROR")
        return res == "OK"

    def set_status_led(self, led: StatusLed, state: bool):
        """Set the status of a specific LED."""
        res = self._transceive(f"LED STATUS {led.value} {int(state)}")
        return res == "OK"