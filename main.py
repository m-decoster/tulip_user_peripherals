import math
import time
from threading import Thread
from typing import List

import numpy as np
from airo_robots.drives.hardware.kelo_robile import KELORobile
from airo_typing import Vector3DType
from loguru import logger

from peripheral_client import PeripheralClient


class PlatformPoseEstimatorPeripherals:
    def __init__(self):
        self._time_last_update = None
        self._pose = np.array([0.0, 0.0, 0.0])

    def _calculate_velocities(self, delta_t: float, raw_flow: List[float]):
        [flow_x_1, flow_y_1, flow_x_2, flow_y_2] = raw_flow

        T_X = 0.348  # mounting position of the flow sensor on robot
        T_Y = 0.232  # mounting position of the flow sensor on robot
        R = np.sqrt(T_X ** 2 + T_Y ** 2)
        beta = np.arctan2(T_Y, T_X)

        v_x_1 = (flow_x_1 - flow_y_1) * np.sqrt(2) / 2 / delta_t
        v_y_1 = (-flow_x_1 - flow_y_1) * np.sqrt(2) / 2 / delta_t
        v_a_1 = (-flow_x_1 * np.cos(beta) - flow_y_1 * np.sin(beta)) / R / delta_t
        v_x_2 = (-flow_x_2 + flow_y_2) * np.sqrt(2) / 2 / delta_t
        v_y_2 = (flow_x_2 + flow_y_2) * np.sqrt(2) / 2 / delta_t
        v_a_2 = (-flow_x_2 * np.cos(beta) - flow_y_2 * np.sin(beta)) / R / delta_t

        v_x = (v_x_1 + v_x_2) / 2
        v_y = (v_y_1 + v_y_2) / 2
        v_a = (v_a_1 + v_a_2) / 2

        return v_x, v_y, v_a

    def _update_pose(self, delta_t: float, v_x, v_y, p_a):
        self._pose[0] += (v_x * np.cos(p_a) - v_y * np.sin(p_a)) * delta_t
        self._pose[1] += (v_x * np.sin(p_a) + v_y * np.cos(p_a)) * delta_t
        self._pose[2] = p_a

    def get_pose(self, raw_flow: List[float], raw_orientation_x: float) -> np.ndarray:
        if self._time_last_update is None:
            self._time_last_update = time.time()
            return np.array([0.0, 0.0, 0.0])

        delta_time = time.time() - self._time_last_update
        self._time_last_update = time.time()

        v_x, v_y, v_a = self._calculate_velocities(delta_time, raw_flow)
        self._update_pose(delta_time, v_x, v_y, -raw_orientation_x)

        return self._pose


class KELORobilePeripherals(KELORobile):
    def __init__(self, robot_ip: str):
        super().__init__(robot_ip)
        self._peripheral_client = PeripheralClient("/dev/ttyACM0", 115200)
        self._pose_estimator = PlatformPoseEstimatorPeripherals()

        self._flow = None  # Relative offset data.
        self._orientation = None  # Relative orientation data.
        self._orientation_start = None  # To obtain relative orientation changes.

        self._pose = np.zeros((3,))

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

        self._pose = self._pose_estimator.get_pose(self._flow.tolist(), self._orientation[0])

    def get_odometry(self) -> Vector3DType:
        return self._pose


def test_movement(mobi: KELORobile):
    logger.info(f"Starting odometry test with {mobi.__class__.__name__}.")

    x, y, a = mobi.get_odometry()
    logger.info(f"Start odometry: {x=} {y=} {a=}")

    input("Press enter to drive 1m forward in closed loop.")

    mobi.move_platform_to_pose(1.0 + x, y, a, timeout=10.0).wait()
    x, y, a = mobi.get_odometry()
    logger.info(f"End odometry: {x=} {y=} {a=}")

    input("Press enter to drive 1m backward in closed loop.")

    x, y, a = mobi.get_odometry()
    logger.info(f"Start odometry: {x=} {y=} {a=}")
    mobi.move_platform_to_pose(-1.0 + x, y, a, timeout=10.0).wait()
    x, y, a = mobi.get_odometry()
    logger.info(f"End odometry: {x=} {y=} {a=}")

    input("Press enter to drive 1m left in closed loop.")

    x, y, a = mobi.get_odometry()
    logger.info(f"Start odometry: {x=} {y=} {a=}")
    mobi.move_platform_to_pose(x, 1.0 + y, a, timeout=10.0).wait()
    x, y, a = mobi.get_odometry()
    logger.info(f"End odometry: {x=} {y=} {a=}")

    input("Press enter to drive 1m right in closed loop.")

    x, y, a = mobi.get_odometry()
    logger.info(f"Start odometry: {x=} {y=} {a=}")
    mobi.move_platform_to_pose(x, -1.0 + y, a, timeout=10.0).wait()
    x, y, a = mobi.get_odometry()
    logger.info(f"End odometry: {x=} {y=} {a=}")

    input("Press enter to rotate 360 degrees in closed loop.")

    x, y, a = mobi.get_odometry()
    logger.info(f"Start odometry: {x=} {y=} {a=}")
    mobi.move_platform_to_pose(x, y, a + np.deg2rad(360), timeout=10.0).wait()
    x, y, a = mobi.get_odometry()
    logger.info(f"End odometry: {x=} {y=} {a=}")


if __name__ == "__main__":
    mobi_encoders = KELORobile("10.42.0.1")
    test_movement(mobi_encoders)

    mobi_peripherals = KELORobilePeripherals("10.42.0.1")
    test_movement(mobi_peripherals)
