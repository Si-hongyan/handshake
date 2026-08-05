"""UDP 发送端（Windows 侧）：把最新视觉状态发往 WSL。

只发送最新状态，不缓冲、不重传。UDP 无连接，天然“丢旧留新”。
"""

from __future__ import annotations

import socket
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shared.protocol import VisionState  # noqa: E402


class UdpSender:
    def __init__(self, host: str = "127.0.0.1", port: int = 51555):
        self.addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, state: VisionState) -> None:
        try:
            self._sock.sendto(state.to_bytes(), self.addr)
        except OSError:
            # 发送失败不抛出（对端未就绪时不阻塞视觉线程）。
            pass

    def close(self) -> None:
        self._sock.close()
