"""稳定手势跟踪器单元测试（纯逻辑，注入时间）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from windows_vision.vision_state import StableGestureTracker, compute_latency_ok


class TestStableTracker(unittest.TestCase):
    def test_not_stable_before_dwell(self):
        t = StableGestureTracker(dwell_seconds=0.7)
        self.assertFalse(t.update(True, 0.0))
        self.assertFalse(t.update(True, 0.3))
        self.assertFalse(t.update(True, 0.6))

    def test_stable_after_dwell(self):
        t = StableGestureTracker(dwell_seconds=0.7)
        for tt in [0.0, 0.2, 0.4, 0.6]:
            t.update(True, tt)
        self.assertTrue(t.update(True, 0.75))

    def test_interruption_resets(self):
        t = StableGestureTracker(dwell_seconds=0.7)
        t.update(True, 0.0)
        t.update(True, 0.5)
        self.assertFalse(t.update(False, 0.6))   # 中断
        self.assertFalse(t.update(True, 0.7))    # 重新计时
        self.assertFalse(t.update(True, 1.2))
        self.assertTrue(t.update(True, 1.45))    # 从 0.7 起算满 0.7s

    def test_latency_ok(self):
        self.assertTrue(compute_latency_ok(100, 150))
        self.assertFalse(compute_latency_ok(200, 150))


if __name__ == "__main__":
    unittest.main()
