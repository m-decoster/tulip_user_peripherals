import math
import time
from threading import Thread
from typing import List

import numpy as np
from airo_robots.drives.hardware.kelo_robile import KELORobile
from airo_typing import Vector3DType
from loguru import logger

from peripheral_client import PeripheralClient



class KELORobilePeripherals(KELORobile):
    def __init__(self, robot_ip: str):
        super().__init__(robot_ip)
        self._peripheral_client = PeripheralClient("/dev/ttyACM0", 115200)

        self._flow = None  # Relative offset data.
        self._orientation = None  # Relative orientation data.
        self._orientation_start = None  # To obtain relative orientation changes.
        self._time_last_update = None

        self._total_flow = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)  # Total flow accumulated over time.
        self._pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # Pose of the robot in the world frame.
        # Start a thread that calls update_peripherals at 20Hz.
        thread = Thread(
            target=self._periodic_update_peripherals,
            args=(),
            daemon=True,
        )
        thread.start()

        while self._flow is None or self._orientation is None:
            logger.info("Waiting for initial flow and orientation data...")
            time.sleep(0.5)

    def _periodic_update_peripherals(self):
        """Periodically update the peripheral data at 20Hz."""
        while True:
            self.update_peripherals()
            time.sleep(0.05)

    def update_peripherals(self):
        """Get updated information from the peripherals. Should be called periodically, e.g., at 20Hz."""

        # Flow is relative data.
        self._flow = np.array(self._peripheral_client.get_flow(), dtype=np.float64)
        self._flow /= 12750.0  # conversion from dimensionless to meters. this is specific to our setup.

        # Orientation is absolute data: subtract start orientation.
        self._orientation = np.deg2rad(np.array(self._peripheral_client.get_orientation(), dtype=np.float64))
        if self._orientation_start is None:
            self._orientation_start = self._orientation.copy()
        self._orientation -= self._orientation_start

        self._total_flow += self._flow

        [flow_x_1, flow_y_1, flow_x_2, flow_y_2] = self._flow
        if self._time_last_update is None:
            self._time_last_update = time.time()
            return np.array([0.0, 0.0, 0.0])

        delta_t = time.time() - self._time_last_update
        self._time_last_update = time.time()

        v_x_1 = (flow_x_1 - flow_y_1) * np.sqrt(2) / 2 / delta_t
        v_y_1 = (-flow_x_1 - flow_y_1) * np.sqrt(2) / 2 / delta_t
        v_x_2 = (-flow_x_2 + flow_y_2) * np.sqrt(2) / 2 / delta_t
        v_y_2 = (flow_x_2 + flow_y_2) * np.sqrt(2) / 2 / delta_t

        v_x = (v_x_1 + v_x_2) / 2
        v_y = (v_y_1 + v_y_2) / 2
        p_a = 0.0

        self._pose[0] += (v_x * np.cos(p_a) - v_y * np.sin(p_a)) * delta_t
        self._pose[1] += (v_x * np.sin(p_a) + v_y * np.cos(p_a)) * delta_t
        self._pose[2] = p_a

        print(f"Flow: {self._flow}, Total flow: {self._total_flow}, Orientation: {self._orientation}")
        print(f"Velocity: ({v_x:.3f}, {v_y:.3f}), Pose: {self._pose}")


def test_movement(mobi: KELORobile):
    logger.info(f"Starting odometry test with {mobi.__class__.__name__}.")

    input("Press enter to drive 0.5m forward in open loop.")

    mobi.set_platform_velocity_target(0.1, 0.0, 0.0, timeout=5).wait()


if __name__ == "__main__":
    mobi_peripherals = KELORobilePeripherals("10.42.0.1")
    test_movement(mobi_peripherals)
