"""Watchdog（WSL 侧）：把“时间/链路”类安全信号汇总成 Events。

判断：
  - 视觉链路超时：距最近一次收到视觉数据 > vision_timeout_s；
  - 摄像头状态：来自最新 VisionState.camera_alive；
  - 手消失：最新状态 hand_detected==False（运动中由状态机决定是否撤回）。
急停来源（estop）由外部注入（键盘 / 后续 GPIO 物理按钮）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from shared.protocol import VisionState
from wsl_core.state_machine import VisionInput, Events


@dataclass
class WatchdogConfig:
    vision_timeout_s: float
    startup_grace_s: float = 5.0


class Watchdog:
    def __init__(self, cfg: WatchdogConfig):
        self.cfg = cfg
        self._ever_received = False
        self._t_start = None

    def build_inputs(
        self,
        now: float,
        last_state: Optional[VisionState],
        seconds_since_last: float,
        estop: bool,
    ) -> tuple[VisionInput, Events]:
        if self._t_start is None:
            self._t_start = now
        if last_state is not None:
            self._ever_received = True

        # 启动宽限：从未收到过视觉、且仍在宽限期内 -> 不判超时（不误 STOP）。
        in_grace = (not self._ever_received) and ((now - self._t_start) <= self.cfg.startup_grace_s)
        vision_timeout = (seconds_since_last > self.cfg.vision_timeout_s) and not in_grace

        if last_state is None:
            # 启动宽限期内尚无数据：视为“未知，不动作”，让状态机停在 WAIT_HAND，
            # 既不触发握手（conditions_met=False），也不误判为摄像头断开而 STOP。
            # 宽限期外仍无数据：camera_alive=False + vision_timeout=True -> STOP。
            vision = VisionInput(
                conditions_met=False,
                hand_present=False,
                camera_alive=in_grace,
                fresh=in_grace,
                inside_zone=False,
            )
        else:
            fresh = not vision_timeout
            vision = VisionInput(
                conditions_met=last_state.all_conditions_met(),
                hand_present=last_state.hand_detected,
                camera_alive=last_state.camera_alive,
                fresh=fresh,
                inside_zone=last_state.inside_zone,
            )

        events = Events(estop=estop, vision_timeout=vision_timeout, reset=False)
        return vision, events
