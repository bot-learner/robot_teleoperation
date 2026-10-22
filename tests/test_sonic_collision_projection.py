import unittest

import numpy as np

from pico_teleop_interface.robot_interfaces.a3_ultra.sim import (
    _furthest_collision_free_target,
)


class _FakeIK:
    _arm_q_indices = np.array([0])
    _q = np.zeros(1)
    collision_min_distance = 0.2

    def _minimum_collision_distance(self, configuration):
        position = float(configuration[0])
        # 0.3～0.5 rad 之间模拟一段碰撞区，验证不会跳到其后的安全终点。
        distance = 0.1 if 0.3 < position < 0.5 else 0.3
        return distance, 0


class SonicCollisionProjectionTest(unittest.TestCase):
    def test_stops_before_first_collision_region(self):
        target, distance, fraction = _furthest_collision_free_target(
            _FakeIK(), np.array([0.0]), np.array([0.8]))

        self.assertGreaterEqual(distance, _FakeIK.collision_min_distance)
        self.assertLessEqual(float(target[0]), 0.3)
        self.assertLess(fraction, 1.0)

    def test_keeps_fully_safe_target(self):
        target, distance, fraction = _furthest_collision_free_target(
            _FakeIK(), np.array([0.5]), np.array([0.8]))

        np.testing.assert_allclose(target, [0.8])
        self.assertGreaterEqual(distance, _FakeIK.collision_min_distance)
        self.assertEqual(fraction, 1.0)


if __name__ == "__main__":
    unittest.main()
