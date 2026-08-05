"""ROI 逻辑单元测试（纯逻辑，无 cv2）。"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from windows_vision.roi import Roi, point_in_roi, load_roi, save_roi


class TestRoi(unittest.TestCase):
    def test_contains_inside(self):
        r = Roi(x=0.5, y=0.3, w=0.3, h=0.4)
        self.assertTrue(point_in_roi(0.6, 0.5, r))

    def test_contains_outside(self):
        r = Roi(x=0.5, y=0.3, w=0.3, h=0.4)
        self.assertFalse(point_in_roi(0.2, 0.2, r))
        self.assertFalse(point_in_roi(0.9, 0.9, r))

    def test_boundary(self):
        r = Roi(x=0.5, y=0.3, w=0.3, h=0.4)
        self.assertTrue(point_in_roi(0.5, 0.3, r))       # 左上角
        self.assertTrue(point_in_roi(0.8, 0.7, r))       # 右下角

    def test_clamp(self):
        r = Roi(x=0.9, y=0.9, w=0.5, h=0.5).clamp()
        self.assertLessEqual(r.x + r.w, 1.0 + 1e-9)
        self.assertLessEqual(r.y + r.h, 1.0 + 1e-9)

    def test_to_pixels(self):
        r = Roi(x=0.5, y=0.5, w=0.5, h=0.5)
        self.assertEqual(r.to_pixels(640, 480), (320, 240, 320, 240))

    def test_save_and_load_roundtrip(self):
        r = Roi(x=0.11, y=0.22, w=0.33, h=0.44, label="Z")
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "roi.yaml")
            save_roi(p, r)
            r2 = load_roi(p)
            self.assertAlmostEqual(r2.x, 0.11)
            self.assertAlmostEqual(r2.y, 0.22)
            self.assertAlmostEqual(r2.w, 0.33)
            self.assertAlmostEqual(r2.h, 0.44)
            self.assertEqual(r2.label, "Z")


if __name__ == "__main__":
    unittest.main()
