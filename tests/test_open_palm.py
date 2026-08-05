"""张开手掌判定单元测试（纯逻辑，无 cv2/mediapipe）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from windows_vision.open_palm import (
    is_open_palm, count_extended_fingers, wrist_xy, finger_extended,
)


def make_open_hand():
    """构造一个“张开手”的 21 关键点（手心朝上，指尖远离手腕）。

    简化模型：wrist 在 (0.5, 0.9)；各指沿 y 向上伸展，
    tip 比 pip 离 wrist 更远。
    """
    lm = [[0.5, 0.9, 0.0] for _ in range(21)]
    # index(5..8), middle(9..12), ring(13..16), pinky(17..20)
    # 布局：mcp 近手腕，pip 更远，tip 最远
    finger_cols = {
        "index": (5, 0.35),
        "middle": (9, 0.45),
        "ring": (13, 0.55),
        "pinky": (17, 0.65),
    }
    for name, (base, x) in finger_cols.items():
        lm[base] = [x, 0.6, 0.0]      # mcp
        lm[base + 1] = [x, 0.45, 0.0] # pip
        lm[base + 2] = [x, 0.30, 0.0] # dip
        lm[base + 3] = [x, 0.15, 0.0] # tip (离 wrist 最远)
    # thumb 1..4
    lm[1] = [0.40, 0.80, 0.0]
    lm[2] = [0.34, 0.72, 0.0]
    lm[3] = [0.30, 0.66, 0.0]
    lm[4] = [0.26, 0.60, 0.0]
    return lm


def make_fist():
    """构造一个“握拳”的关键点：指尖靠近手腕（tip 比 pip 更近 wrist）。"""
    lm = [[0.5, 0.9, 0.0] for _ in range(21)]
    finger_cols = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}
    for name, base in finger_cols.items():
        x = 0.45 + 0.03 * list(finger_cols).index(name)
        lm[base] = [x, 0.70, 0.0]      # mcp
        lm[base + 1] = [x, 0.62, 0.0]  # pip (离 wrist 较远)
        lm[base + 2] = [x, 0.70, 0.0]  # dip 卷回
        lm[base + 3] = [x, 0.78, 0.0]  # tip 卷回靠近 wrist
    return lm


class TestOpenPalm(unittest.TestCase):
    def test_open_hand_detected(self):
        lm = make_open_hand()
        self.assertEqual(count_extended_fingers(lm), 4)
        self.assertTrue(is_open_palm(lm, min_extended_fingers=4))

    def test_fist_not_open(self):
        lm = make_fist()
        self.assertLess(count_extended_fingers(lm), 4)
        self.assertFalse(is_open_palm(lm, min_extended_fingers=4))

    def test_threshold_effect(self):
        lm = make_open_hand()
        # 阈值设为 5（含拇指才可能达到）时，仅四指伸直不足以判定
        self.assertFalse(is_open_palm(lm, min_extended_fingers=5, include_thumb=False))

    def test_wrist_xy(self):
        lm = make_open_hand()
        x, y = wrist_xy(lm)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 0.9)

    def test_short_landmarks_returns_false(self):
        self.assertFalse(is_open_palm([[0, 0, 0]] * 5))
        self.assertFalse(is_open_palm(None))

    def test_single_finger(self):
        lm = make_fist()
        # 只把 index 伸直
        lm[8] = [0.45, 0.10, 0.0]
        self.assertTrue(finger_extended(lm, 8, 6))


if __name__ == "__main__":
    unittest.main()
