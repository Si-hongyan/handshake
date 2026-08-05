"""安全状态机（WSL 侧）—— 纯逻辑，时间由外部传入，便于单元测试。

状态：IDLE, WAIT_HAND, APPROACH, HANDSHAKE, RETRACT, STOP, ERROR

正常流：
  IDLE --(系统就绪)--> WAIT_HAND
  WAIT_HAND --(六项条件连续满足 dwell 秒, 且已 armed)--> APPROACH
  APPROACH --(APPROACH 段完成)--> HANDSHAKE
  HANDSHAKE --(4 段握手完成)--> RETRACT
  RETRACT --(RETRACT+HOME 完成, 回到 HOME)--> WAIT_HAND(重新 armed)

安全流（任意状态可进入）：
  - estop 事件 -> STOP
  - 摄像头断开 / 视觉超时：
       在 APPROACH/HANDSHAKE -> RETRACT（撤回）
       其它 -> STOP
  - 手消失（运动中）-> RETRACT
  - 段超时 / 电机异常 / 控制器抛错 -> ERROR
STOP / ERROR 为吸收态，需外部 reset() 才能恢复（阶段一由人工确认）。

防重复触发：一次稳定手势触发后置 armed=False，必须回到 HOME(经 WAIT_HAND) 才重新 armed。
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import List, Optional

from wsl_core.robot_controller import RobotController, Segment, SafetyError


class State(enum.Enum):
    IDLE = "IDLE"
    WAIT_HAND = "WAIT_HAND"
    APPROACH = "APPROACH"
    HANDSHAKE = "HANDSHAKE"
    RETRACT = "RETRACT"
    STOP = "STOP"
    ERROR = "ERROR"


@dataclass
class VisionInput:
    """状态机消费的视觉输入（从 VisionState 提取的瞬时条件）。"""
    conditions_met: bool          # 六项瞬时条件是否全满足
    hand_present: bool            # 是否还检测到手（用于运动中手消失撤回）
    camera_alive: bool
    fresh: bool                   # 本 tick 是否有新视觉数据（否则可能超时）
    inside_zone: bool = True      # 手腕是否在 ROI 内（用于"握完需先离开再重入"）


@dataclass
class Events:
    estop: bool = False           # 物理/软件急停
    vision_timeout: bool = False  # 视觉链路超时（watchdog 给出）
    reset: bool = False           # 人工复位（STOP/ERROR -> IDLE）


class HandshakeStateMachine:
    def __init__(
        self,
        controller: RobotController,
        approach_segments: List[Segment],
        handshake_segments: List[Segment],
        retract_segments: List[Segment],
        dwell_seconds: float,
        logger=None,
    ):
        self.c = controller
        self.approach_segments = approach_segments
        self.handshake_segments = handshake_segments
        self.retract_segments = retract_segments
        self.dwell_seconds = dwell_seconds
        self._log = logger

        self.state = State.IDLE
        self.armed = True             # 是否允许触发（防重复）
        self._dwell_start: Optional[float] = None
        self._queue: List[Segment] = []
        self._started_current = False
        # 握完一次后要求手先离开 ROI(inside_zone=False)再重入，才允许下次触发。
        # 首次触发不需要(初值 False)；完成一次握手回到 WAIT_HAND 后置 True。
        self._needs_release = False

    # ---- 工具 ----
    def _info(self, msg: str):
        if self._log:
            self._log.info(msg)
        else:
            print(msg)

    def _goto(self, s: State, now: float):
        if s != self.state:
            self._info(f"[fsm] {self.state.value} -> {s.value}")
        self.state = s

    def _load_queue(self, segs: List[Segment]):
        self._queue = list(segs)
        self._started_current = False

    def _drive_queue(self, now: float) -> bool:
        """推进段队列。返回 True 表示队列已全部完成。"""
        if not self._queue:
            return True
        seg = self._queue[0]
        if not self._started_current:
            self.c.start_segment(seg, now)
            self._started_current = True
        self.c.update(now)
        if self.c.is_motion_done(now):
            self._queue.pop(0)
            self._started_current = False
        return len(self._queue) == 0

    # ---- 主 tick ----
    def tick(self, now: float, vision: VisionInput, events: Events) -> State:
        # 0) 吸收态：仅响应 reset
        if self.state in (State.STOP, State.ERROR):
            if events.reset:
                self._info("[fsm] reset -> IDLE")
                self.c.cancel()
                self.armed = True
                self._dwell_start = None
                self._needs_release = False
                self._queue.clear()
                self._goto(State.IDLE, now)
            return self.state

        # 1) 急停最高优先
        if events.estop:
            self.c.emergency_stop()
            self._goto(State.STOP, now)
            return self.state

        moving = self.state in (State.APPROACH, State.HANDSHAKE, State.RETRACT)

        # 2) 摄像头断开 / 视觉超时
        lost_vision = events.vision_timeout or (not vision.camera_alive) or (not vision.fresh)
        if lost_vision:
            if moving and self.state != State.RETRACT:
                self._info("[fsm] 视觉丢失/超时 (运动中) -> RETRACT 撤回")
                self._begin_retract(now)
                return self.state
            elif not moving:
                self._info("[fsm] 视觉丢失/超时 (非运动) -> STOP")
                self.c.cancel()
                self._goto(State.STOP, now)
                return self.state
            # RETRACT 中丢视觉：继续撤回

        # 3) 运动中手消失 -> 撤回（APPROACH/HANDSHAKE）
        if self.state in (State.APPROACH, State.HANDSHAKE) and not vision.hand_present:
            self._info("[fsm] 手消失 (运动中) -> RETRACT 撤回")
            self._begin_retract(now)
            return self.state

        # 4) 各状态逻辑
        try:
            if self.state == State.IDLE:
                # 系统就绪即进入等待
                self._goto(State.WAIT_HAND, now)
                self._dwell_start = None

            elif self.state == State.WAIT_HAND:
                self._tick_wait_hand(now, vision)

            elif self.state == State.APPROACH:
                if self._drive_queue(now):
                    self._load_queue(self.handshake_segments)
                    self._goto(State.HANDSHAKE, now)

            elif self.state == State.HANDSHAKE:
                if self._drive_queue(now):
                    self._begin_retract(now)

            elif self.state == State.RETRACT:
                if self._drive_queue(now):
                    # 回到 HOME，重新 armed，回等待
                    self.armed = True
                    self._dwell_start = None
                    # 要求手先离开 ROI 再重入才允许下次触发(避免对着一直放着的手连续握)。
                    self._needs_release = True
                    self._info("[fsm] 撤回完成，回到 HOME，重新 armed(需手离开ROI再重入才再次触发)")
                    self._goto(State.WAIT_HAND, now)

        except SafetyError as e:
            self._info(f"[fsm] SafetyError -> ERROR: {e}")
            self.c.cancel()
            self._goto(State.ERROR, now)

        return self.state

    def _tick_wait_hand(self, now: float, vision: VisionInput):
        if not self.armed:
            # 尚未复位到可触发（理论上回到 WAIT_HAND 时已 re-arm，这里兜底）
            return
        # "握完需先离开再重入"：若仍要求释放，则等手离开 ROI(inside_zone=False)后解除。
        if self._needs_release:
            if not vision.inside_zone:
                self._needs_release = False
                self._info("[fsm] 手已离开 ROI，解除释放要求，可再次触发")
            self._dwell_start = None
            return
        if vision.conditions_met:
            if self._dwell_start is None:
                self._dwell_start = now
            elif (now - self._dwell_start) >= self.dwell_seconds:
                self._info(f"[fsm] 稳定手势达 {self.dwell_seconds}s -> 触发握手")
                self.armed = False           # 单次触发，防重复
                self._dwell_start = None
                self._load_queue(self.approach_segments)
                self._goto(State.APPROACH, now)
        else:
            self._dwell_start = None          # 条件中断，重新计时

    def _begin_retract(self, now: float):
        self.c.cancel()
        self._load_queue(self.retract_segments)
        self._goto(State.RETRACT, now)

    # ---- 供外部查询 ----
    def dwell_elapsed(self, now: float) -> float:
        if self._dwell_start is None:
            return 0.0
        return now - self._dwell_start
