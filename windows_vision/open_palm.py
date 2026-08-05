"""张开手掌判定 —— 纯几何逻辑，无 cv2 / mediapipe / numpy 依赖。

这样做的目的：
  - 逻辑可在 WSL 上用系统 python3 直接跑单元测试（无需装视觉库）；
  - Windows 侧 hand_landmarker 得到 21 个关键点后调用本模块判定。

MediaPipe Hand 21 关键点索引（标准定义）：
  0  WRIST
  1-4   THUMB (cmc, mcp, ip, tip)
  5-8   INDEX (mcp, pip, dip, tip)
  9-12  MIDDLE
  13-16 RING
  17-20 PINKY

关键点为归一化坐标 (x, y[, z])，本判定仅用 (x, y)。
"""

from __future__ import annotations

from math import hypot
from typing import List, Sequence, Tuple

WRIST = 0
FINGER_TIPS = {"index": 8, "middle": 12, "ring": 16, "pinky": 20}
FINGER_PIPS = {"index": 6, "middle": 10, "ring": 14, "pinky": 18}
THUMB_TIP = 4
THUMB_IP = 3

Point = Tuple[float, float]


def _xy(landmarks: Sequence[Sequence[float]], idx: int) -> Point:
    p = landmarks[idx]
    return float(p[0]), float(p[1])


def _dist(a: Point, b: Point) -> float:
    return hypot(a[0] - b[0], a[1] - b[1])


def finger_extended(landmarks: Sequence[Sequence[float]], tip_idx: int, pip_idx: int) -> bool:
    """某根手指是否伸直：tip 到 wrist 的距离 > pip 到 wrist 的距离。

    这是与朝向无关的判据（不依赖“y 更小=更高”，因此手上下颠倒也稳健）。
    """
    wrist = _xy(landmarks, WRIST)
    tip = _xy(landmarks, tip_idx)
    pip = _xy(landmarks, pip_idx)
    return _dist(tip, wrist) > _dist(pip, wrist)


def count_extended_fingers(
    landmarks: Sequence[Sequence[float]], include_thumb: bool = False
) -> int:
    """统计伸直的手指数量。"""
    count = 0
    for name, tip_idx in FINGER_TIPS.items():
        pip_idx = FINGER_PIPS[name]
        if finger_extended(landmarks, tip_idx, pip_idx):
            count += 1
    if include_thumb:
        # 拇指用 tip 到 wrist vs ip 到 wrist 近似。
        if finger_extended(landmarks, THUMB_TIP, THUMB_IP):
            count += 1
    return count


def is_open_palm(
    landmarks: Sequence[Sequence[float]],
    min_extended_fingers: int = 4,
    include_thumb: bool = False,
) -> bool:
    """伸直手指数 >= 阈值 即判定为张开手掌。"""
    if landmarks is None or len(landmarks) < 21:
        return False
    return count_extended_fingers(landmarks, include_thumb) >= min_extended_fingers


def wrist_xy(landmarks: Sequence[Sequence[float]]) -> Point:
    """返回手腕(landmark 0)的归一化 (x, y)。"""
    return _xy(landmarks, WRIST)
