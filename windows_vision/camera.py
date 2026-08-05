"""摄像头读取（Windows 侧）：独立线程抓帧，只保留最新一帧，丢弃旧帧。

设计要点（对应“只保留最新视觉帧，不要积累旧帧”）：
  - 后台线程持续 grab/read；
  - 用锁保护一个 single-slot（最新帧），新帧直接覆盖旧帧；
  - 消费者拿到的永远是最近一帧，不会积压延迟。
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple


class LatestFrameCamera:
    def __init__(self, index: int = 0, width: int = 640, height: int = 480, target_fps: int = 30):
        self.index = index
        self.width = width
        self.height = height
        self.target_fps = target_fps

        self._cap = None
        self._lock = threading.Lock()
        self._frame = None
        self._frame_t = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_grab_t = 0.0

    def open(self) -> None:
        import cv2

        self._cap = cv2.VideoCapture(self.index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        # 尽量减小驱动内部缓冲，进一步避免堆积（部分后端支持）。
        try:
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 index={self.index}")
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            now = time.time()
            if not ok:
                # 读帧失败：不覆盖最新帧，让 alive 判定通过时间戳失效。
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._frame_t = now
            self._last_grab_t = now

    def read_latest(self) -> Tuple[Optional["object"], float]:
        """返回 (最新帧, 该帧时间戳)。无帧时返回 (None, 0)。"""
        with self._lock:
            return self._frame, self._frame_t

    def is_alive(self, timeout_s: float) -> bool:
        """距最近一帧是否在 timeout_s 内。"""
        return (time.time() - self._frame_t) <= timeout_s if self._frame_t else False

    def release(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
