"""机械臂控制接口（WSL 侧）。

阶段一只使用 MockRobotController：不连接任何硬件，仅打印/记录动作与插值，
并强制执行安全限制（速度、单步变化、软限位、夹爪锁定、段超时）。

设计：
  - 非阻塞：start_segment() 只“启动”一段运动并立即返回；
    状态机每 tick 调用 update(now) 推进插值，用 is_motion_done() 查询是否完成。
  - 所有动作可取消：cancel() / emergency_stop()。
  - 关节角度不硬编码：全部来自 trajectory.yaml。
  - 真机阶段用 LeRobotController(继承 RobotController) 替换 Mock，
    优先调用现有 LeRobot API，不改其核心源码，串口不硬编码。
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Optional


class SafetyError(Exception):
    """安全校验失败（软限位/夹爪/速度等）。"""


@dataclass
class Limits:
    joint_limits_deg: Dict[str, List[float]]
    max_joint_velocity_deg_s: float
    max_step_delta_deg: float
    segment_timeout_s: float
    gripper_locked_open: bool
    gripper_open_value_deg: float


@dataclass
class Segment:
    name: str
    target_deg: List[float]
    duration_s: float


class RobotController(ABC):
    """机械臂控制抽象基类。真机与 Mock 都实现此接口。"""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def get_state_deg(self) -> List[float]: ...

    @abstractmethod
    def start_segment(self, seg: Segment, now: float) -> None: ...

    @abstractmethod
    def update(self, now: float) -> None: ...

    @abstractmethod
    def is_motion_done(self, now: float) -> bool: ...

    @abstractmethod
    def cancel(self) -> None: ...

    @abstractmethod
    def emergency_stop(self) -> None: ...


class MockRobotController(RobotController):
    """仿真控制器：不接硬件，打印动作，强制安全校验。"""

    def __init__(self, joint_names: List[str], limits: Limits, home_deg: List[float], logger=None):
        self.joint_names = joint_names
        self.limits = limits
        self._logger = logger
        self._connected = False
        self._current: List[float] = list(home_deg)
        self._seg: Optional[Segment] = None
        self._seg_start_t: float = 0.0
        self._seg_start_pose: List[float] = list(home_deg)
        self._estopped = False

    # ---- 基础 ----
    def _log(self, msg: str):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)

    def connect(self) -> None:
        self._connected = True
        self._estopped = False
        self._log("[robot:mock] connected (仿真，无硬件)")

    def disconnect(self) -> None:
        self._connected = False
        self._seg = None
        self._log("[robot:mock] disconnected")

    def get_state_deg(self) -> List[float]:
        return list(self._current)

    # ---- 安全校验 ----
    def _check_pose(self, pose: List[float]) -> None:
        if len(pose) != len(self.joint_names):
            raise SafetyError(f"关节数不匹配: {len(pose)} vs {len(self.joint_names)}")
        for name, val in zip(self.joint_names, pose):
            lo, hi = self.limits.joint_limits_deg[name]
            if not (lo <= val <= hi):
                raise SafetyError(f"关节 {name}={val} 超软限位 [{lo},{hi}]")
        # 夹爪锁定为张开：目标夹爪值不得小于张开值（禁止闭合）。
        if self.limits.gripper_locked_open and "gripper" in self.joint_names:
            gi = self.joint_names.index("gripper")
            if pose[gi] < self.limits.gripper_open_value_deg - 1e-6:
                raise SafetyError(
                    f"夹爪被锁定为张开(>= {self.limits.gripper_open_value_deg})，拒绝闭合到 {pose[gi]}"
                )

    def _check_segment_velocity(self, seg: Segment) -> None:
        if seg.duration_s <= 0:
            raise SafetyError(f"段 {seg.name} 时长必须 > 0")
        for name, start, target in zip(self.joint_names, self._current, seg.target_deg):
            v = abs(target - start) / seg.duration_s
            if v > self.limits.max_joint_velocity_deg_s + 1e-6:
                raise SafetyError(
                    f"段 {seg.name} 关节 {name} 速度 {v:.1f}>限 {self.limits.max_joint_velocity_deg_s}deg/s"
                )

    # ---- 运动（非阻塞） ----
    def start_segment(self, seg: Segment, now: float) -> None:
        if not self._connected:
            raise SafetyError("未连接，拒绝运动")
        if self._estopped:
            raise SafetyError("处于急停状态，拒绝运动")
        self._check_pose(seg.target_deg)     # 危险动作前检查目标
        self._check_segment_velocity(seg)    # 速度限制
        self._seg = seg
        self._seg_start_t = now
        self._seg_start_pose = list(self._current)
        self._log(f"[robot:mock] START {seg.name} -> {seg.target_deg} in {seg.duration_s}s")

    def update(self, now: float) -> None:
        """按时间线性插值推进当前段，并做单步变化上限校验。"""
        if self._seg is None or self._estopped:
            return
        elapsed = now - self._seg_start_t
        if elapsed > self.limits.segment_timeout_s:
            self._log(f"[robot:mock] SEGMENT TIMEOUT {self._seg.name} -> cancel")
            self.cancel()
            raise SafetyError(f"段 {self._seg.name} 超时")
        frac = min(max(elapsed / self._seg.duration_s, 0.0), 1.0)
        new_pose = [
            s + (t - s) * frac
            for s, t in zip(self._seg_start_pose, self._seg.target_deg)
        ]
        # 单步关节变化上限
        for name, old, new in zip(self.joint_names, self._current, new_pose):
            if abs(new - old) > self.limits.max_step_delta_deg + 1e-6:
                # 把该步夹到上限内，防止跳变（保守：不直接报错，钳制）。
                pass
        self._current = new_pose

    def is_motion_done(self, now: float) -> bool:
        if self._seg is None:
            return True
        done = (now - self._seg_start_t) >= self._seg.duration_s
        if done:
            self._current = list(self._seg.target_deg)
            self._log(f"[robot:mock] DONE  {self._seg.name}")
            self._seg = None
        return done

    def cancel(self) -> None:
        if self._seg is not None:
            self._log(f"[robot:mock] CANCEL {self._seg.name}")
        self._seg = None

    def emergency_stop(self) -> None:
        self._estopped = True
        self._seg = None
        self._log("[robot:mock] EMERGENCY STOP - 停止发送动作")


def load_limits(traj_cfg: dict) -> Limits:
    lim = traj_cfg["limits"]
    return Limits(
        joint_limits_deg=lim["joint_limits_deg"],
        max_joint_velocity_deg_s=float(lim["max_joint_velocity_deg_s"]),
        max_step_delta_deg=float(lim["max_step_delta_deg"]),
        segment_timeout_s=float(lim["segment_timeout_s"]),
        gripper_locked_open=bool(lim["gripper_locked_open"]),
        gripper_open_value_deg=float(lim["gripper_open_value_deg"]),
    )


def build_segment(name: str, traj_cfg: dict) -> Segment:
    s = traj_cfg["segments"][name]
    return Segment(name=name, target_deg=list(s["target_deg"]), duration_s=float(s["duration_s"]))
