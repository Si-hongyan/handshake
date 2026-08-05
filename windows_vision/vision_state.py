"""视觉状态计算 + 稳定手势跟踪器（Windows 侧）。

StableGestureTracker 为纯逻辑（时间由外部传入），可在 WSL 上单元测试。
它维护一个 dwell 时间窗，统计窗口内“好帧比例”，判断是否稳定。
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Tuple


class StableGestureTracker:
    """连续满足条件达到 dwell_seconds 才判定 stable。

    - 每次 update(cond_met, now) 记录一个样本；
    - 只保留 dwell 窗口内的样本；
    - 若窗口已覆盖 dwell_seconds 且坏帧比例 <= max_bad_frame_ratio，则 stable。
    """

    def __init__(self, dwell_seconds: float = 0.7, max_bad_frame_ratio: float = 0.2):
        self.dwell_seconds = dwell_seconds
        self.max_bad_frame_ratio = max_bad_frame_ratio
        self._samples: Deque[Tuple[float, bool]] = deque()
        self._window_start: float = 0.0

    def reset(self) -> None:
        self._samples.clear()
        self._window_start = 0.0

    def update(self, cond_met: bool, now: float) -> bool:
        if not cond_met:
            # 条件中断：清空窗口，重新计时。
            self.reset()
            return False

        if not self._samples:
            self._window_start = now
        self._samples.append((now, cond_met))

        # 丢弃 dwell 窗口之外的旧样本。
        cutoff = now - self.dwell_seconds
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

        elapsed = now - self._window_start
        if elapsed < self.dwell_seconds:
            return False

        bad = sum(1 for _, ok in self._samples if not ok)
        total = max(len(self._samples), 1)
        return (bad / total) <= self.max_bad_frame_ratio

    def elapsed(self, now: float) -> float:
        """当前已连续稳定的时长（秒），用于 UI 计时显示。"""
        if not self._samples:
            return 0.0
        return max(0.0, now - self._window_start)


def compute_latency_ok(latency_ms: float, max_latency_ms: float) -> bool:
    return latency_ms <= max_latency_ms
