import math
import unittest

from collaborative_detection.nodes.cusum_detector_node import update_cusum
from collaborative_detection.nodes.waypoint_follower_node import pure_pursuit


class CoreLogicTest(unittest.TestCase):
    def test_cusum_positive_tail_accumulates_positive_innovation(self):
        plus, minus = update_cusum(0.0, 0.0, 2.0, 0.5)
        self.assertEqual(plus, 1.5)
        self.assertEqual(minus, 0.0)

    def test_cusum_negative_tail_accumulates_negative_innovation(self):
        plus, minus = update_cusum(0.0, 0.0, -2.0, 0.5)
        self.assertEqual(plus, 0.0)
        self.assertEqual(minus, 1.5)

    def test_cusum_resets_a_tail_when_evidence_changes_sign(self):
        plus, minus = update_cusum(2.0, 1.0, -2.0, 0.5)
        self.assertEqual(plus, 0.0)
        self.assertEqual(minus, 2.5)

    def test_pure_pursuit_handles_missing_measurement(self):
        v, w, idx = pure_pursuit(
            None, None, None, [(0.0, 0.0), (5.0, 0.0)], 0,
            0.2, 0.8, 0.0, 0.5,
        )
        self.assertEqual((v, w, idx), (0.0, 0.0, 0))

    def test_pure_pursuit_drives_straight_toward_ahead_point(self):
        v, w, idx = pure_pursuit(
            0.0, 0.0, 0.0, [(0.0, 0.0), (5.0, 0.0)], 0,
            0.2, 0.8, 0.0, 0.5,
        )
        self.assertEqual(idx, 0)
        self.assertTrue(math.isclose(v, 0.2))
        self.assertTrue(math.isclose(w, 0.0, abs_tol=1e-12))

    def test_pure_pursuit_advances_when_close_to_next_waypoint(self):
        v, w, idx = pure_pursuit(
            4.8, 0.0, 0.0, [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)], 0,
            0.2, 0.8, 0.0, 0.5,
        )
        self.assertEqual(idx, 1)
        self.assertGreaterEqual(v, 0.0)
        self.assertLessEqual(abs(w), 1.5)


if __name__ == '__main__':
    unittest.main()
