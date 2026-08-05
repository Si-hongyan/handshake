"""共享：视觉状态的 JSON 协议定义（Windows 发送端与 WSL 接收端共用一份字段约定）。

阶段一通过 UDP 传输一个轻量 JSON。只发送“最新状态”，接收端只保留最新数据报。

字段:
    seq            int    单调递增序号（接收端可用于判断是否有新帧）
    t_send         float  发送端时间戳(秒, time.time())
    hand_detected  bool   画面中是否检测到手（且置信度达标）
    open_palm      bool   手掌是否张开
    inside_zone    bool   手腕中心点是否落入 ROI
    stable_gesture bool   发送端本地判断的稳定手势（仅供显示/参考；
                          真正的触发 dwell 计时由 WSL 状态机独立完成）
    camera_alive   bool   摄像头是否正常出帧
    latency_ok     bool   单帧处理延迟是否在阈值内
    confidence     float  手部检测置信度 [0,1]
    wrist_xy       [float,float] 手腕归一化坐标 [x,y]，仅用于可视化/调试
    latency_ms     float  最近一帧处理延迟(毫秒)
    fps            float  发送端估计帧率

“触发条件”（六项全 true）由 WSL 状态机判断，不在发送端下结论。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict, field
from typing import Optional, Tuple


PROTOCOL_VERSION = 1


@dataclass
class VisionState:
    seq: int = 0
    t_send: float = 0.0
    hand_detected: bool = False
    open_palm: bool = False
    inside_zone: bool = False
    stable_gesture: bool = False
    camera_alive: bool = False
    latency_ok: bool = False
    confidence: float = 0.0
    wrist_xy: Optional[Tuple[float, float]] = None
    latency_ms: float = 0.0
    fps: float = 0.0
    v: int = field(default=PROTOCOL_VERSION)

    def all_conditions_met(self) -> bool:
        """六项触发条件是否同时满足（供状态机做 dwell 前的瞬时判断）。"""
        return (
            self.hand_detected
            and self.open_palm
            and self.inside_zone
            and self.camera_alive
            and self.latency_ok
        )

    def to_bytes(self) -> bytes:
        return json.dumps(asdict(self), separators=(",", ":")).encode("utf-8")

    @staticmethod
    def from_bytes(data: bytes) -> "VisionState":
        obj = json.loads(data.decode("utf-8"))
        wx = obj.get("wrist_xy")
        return VisionState(
            seq=int(obj.get("seq", 0)),
            t_send=float(obj.get("t_send", 0.0)),
            hand_detected=bool(obj.get("hand_detected", False)),
            open_palm=bool(obj.get("open_palm", False)),
            inside_zone=bool(obj.get("inside_zone", False)),
            stable_gesture=bool(obj.get("stable_gesture", False)),
            camera_alive=bool(obj.get("camera_alive", False)),
            latency_ok=bool(obj.get("latency_ok", False)),
            confidence=float(obj.get("confidence", 0.0)),
            wrist_xy=(float(wx[0]), float(wx[1])) if wx else None,
            latency_ms=float(obj.get("latency_ms", 0.0)),
            fps=float(obj.get("fps", 0.0)),
            v=int(obj.get("v", PROTOCOL_VERSION)),
        )
