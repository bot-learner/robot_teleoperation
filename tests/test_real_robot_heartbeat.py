import time
import unittest

from pico_teleop_interface.robot_interfaces.a3_ultra.a3_ultra_real_robot import (
    A3UltraRealRobot,
)
from pico_teleop_interface.robot_interfaces.base_interface import TeleoperationMonitor


class RealRobotHeartbeatTest(unittest.TestCase):
    def test_stale_true_heartbeat_remains_ready(self):
        robot = A3UltraRealRobot.__new__(A3UltraRealRobot)
        robot._bridge_ready = True
        robot._bridge_ready_at = time.monotonic() - 10.0
        self.assertTrue(robot._control_bridge_ready())

    def test_explicit_not_ready_remains_blocking(self):
        robot = A3UltraRealRobot.__new__(A3UltraRealRobot)
        robot._bridge_ready = False
        self.assertFalse(robot._control_bridge_ready())

    def test_monitor_snapshot_contains_heartbeat_timestamp(self):
        monitor = TeleoperationMonitor(50.0, 25.0, 1.0)
        heartbeat_at = time.monotonic() - 1.5
        monitor.set_robot_feedback(time.monotonic(), 0.25, heartbeat_at, 1.0)
        self.assertEqual(monitor._snapshot()[5], heartbeat_at)


if __name__ == "__main__":
    unittest.main()
