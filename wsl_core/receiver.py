"""UDP 接收端（WSL 侧）：只保留最新视觉状态数据报。

非阻塞 socket，每次 poll 把内核缓冲里的数据报全部读空，只留最后一个（最新）。
记录最近接收时间，供 watchdog 判断视觉链路是否超时。
"""

from __future__ import annotations

import socket
import time
from typing import Optional

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shared.protocol import VisionState  # noqa: E402


class UdpReceiver:
    def __init__(self, bind_host: str = "0.0.0.0", port: int = 51555, max_bytes: int = 2048):
        self.max_bytes = max_bytes
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_host, port))
        self._sock.setblocking(False)
        self._last_state: Optional[VisionState] = None
        self._last_recv_t: float = 0.0

    def poll(self) -> Optional[VisionState]:
        """读空内核缓冲，返回最新一帧（若本次有新数据）。无新数据返回 None。"""
        newest: Optional[bytes] = None
        while True:
            try:
                data, _ = self._sock.recvfrom(self.max_bytes)
                newest = data  # 覆盖，只留最后一个
            except BlockingIOError:
                break
            except OSError:
                break
        if newest is None:
            return None
        try:
            state = VisionState.from_bytes(newest)
        except Exception:
            return None
        self._last_state = state
        self._last_recv_t = time.time()
        return state

    @property
    def last_state(self) -> Optional[VisionState]:
        return self._last_state

    @property
    def last_recv_t(self) -> float:
        return self._last_recv_t

    def seconds_since_last(self, now: float) -> float:
        if self._last_recv_t == 0.0:
            return float("inf")
        return now - self._last_recv_t

    def close(self) -> None:
        self._sock.close()
