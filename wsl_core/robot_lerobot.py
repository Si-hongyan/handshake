"""LeRobot Koch(随从臂)控制器 —— 实现与 MockRobotController 相同的 RobotController 接口，
状态机无需改动即可换用真机。

【安全设计（多重门，默认最保守）】
  1. enable_motion=False（默认）：只读位置 + dry-run 打印，绝不调用 send_action。
     必须显式传 enable_motion=True 才会真正发指令。
  2. 复用 trajectory.yaml 的软限位/速度/单步上限/夹爪锁定/段超时（与 Mock 同一套校验）。
  3. 叠加 LeRobot 自带 max_relative_target（单步位置变化上限）。
  4. 轨迹目标必须来自“本机示教”的文件（trajectory_koch.yaml），
     不使用 Mock 的占位角度（那些未在真机校准，可能自碰撞）。
  5. emergency_stop / cancel 立即停止发送，并可选让电机进入安全保持。

【坐标/单位】
  - 使用 use_degrees=True，关节角以“度”表示，与 trajectory 一致。
  - joint_names 与 LeRobot 的 "<joint>.pos" 键一一映射。

【非阻塞】
  - start_segment 只记录段起止；update(now) 每 tick 计算插值位姿并（在允许时）send_action；
  - is_motion_done(now) 查询是否完成。
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

from wsl_core.robot_controller import (
    RobotController, Limits, Segment, SafetyError,
)


class LeRobotKochController(RobotController):
    def __init__(
        self,
        port: str,
        robot_id: str,
        joint_names: List[str],
        limits: Limits,
        max_relative_target: float = 5.0,
        enable_motion: bool = False,
        io_retries: int = 3,
        io_retry_delay_s: float = 0.01,
        waypoint_mode: bool = False,
        profile_velocity_deg_s: float = 20.0,
        profile_accel_deg_s2: float = 60.0,
        position_tolerance_deg: float = 5.0,
        logger=None,
    ):
        self.port = port
        self.robot_id = robot_id
        self.joint_names = joint_names
        self.limits = limits
        self.max_relative_target = max_relative_target
        self.enable_motion = enable_motion
        # 路点模式：每段只发1条目标，靠 Dynamixel Profile_Velocity 固件限速平滑到位。
        # 写指令从~200条降到~6条，规避 usbipd 写入丢包(抖/崩的根因)。
        self.waypoint_mode = waypoint_mode
        self.profile_velocity_deg_s = profile_velocity_deg_s
        self.profile_accel_deg_s2 = profile_accel_deg_s2
        self.position_tolerance_deg = position_tolerance_deg
        self.io_retries = max(1, io_retries)          # 读/发的最大尝试次数（吸收 usbipd 偶发抖动）
        self.io_retry_delay_s = io_retry_delay_s      # 重试间隔
        self._log = logger

        self._robot = None
        self._connected = False
        self._estopped = False
        self._current: List[float] = [0.0] * len(joint_names)
        self._seg: Optional[Segment] = None
        self._seg_start_t = 0.0
        self._seg_start_pose: List[float] = [0.0] * len(joint_names)

    def _info(self, msg: str):
        (self._log.info if self._log else print)(msg)

    def _retry_io(self, func, what: str):
        """对一次串口 IO 做有限次重试。全部失败则抛出最后异常。

        用于吸收 usbipd(USB-over-IP) 偶发的 'no status packet' 超时。
        注意：只重试“读”与“单帧发送”这类幂等/短操作；不做无限重试，
        连续失败会抛错，交由状态机进入 ERROR/RETRACT，符合安全要求。
        """
        import time as _t
        last = None
        for attempt in range(1, self.io_retries + 1):
            try:
                return func()
            except Exception as e:
                last = e
                if attempt < self.io_retries:
                    self._info(f"[koch] {what} 第{attempt}次失败({type(e).__name__}),重试...")
                    _t.sleep(self.io_retry_delay_s)
        # 全部失败
        raise last

    # ---- 连接 ----
    def connect(self) -> None:
        from lerobot.robots.koch_follower import KochFollower, KochFollowerConfig

        mode = "发送已启用" if self.enable_motion else "DRY-RUN(只读, 不发指令)"
        wp = "路点+固件限速" if self.waypoint_mode else "流式插值"
        self._info(f"[koch] 连接 {self.port} id={self.robot_id} | 模式: {mode} | 执行: {wp}")
        # 路点模式下由固件 Profile_Velocity 限速，需关闭 LeRobot 的相对钳位(否则单条大跳会被钳)
        mrt = None if self.waypoint_mode else self.max_relative_target
        cfg = KochFollowerConfig(
            port=self.port,
            id=self.robot_id,
            max_relative_target=mrt,
            use_degrees=True,                              # 关节角以度表示，与 trajectory 一致
            # 断开时是否松力矩由我们显式控制(见 disconnect)，不用 LeRobot 自动松：
            # 异常中止(半空)时松力矩会让臂砸落，必须保持力矩停在原地。
            disable_torque_on_disconnect=False,
        )
        self._robot = KochFollower(cfg)
        # calibrate=False：要求此前已用 lerobot-calibrate 校准过；避免每次触发校准流程
        self._robot.connect(calibrate=False)
        self._connected = True
        self._estopped = False
        if self.waypoint_mode and self.enable_motion:
            self._apply_profile_limits()
        # 读取初始真实位置作为起点
        self._current = self._read_pose()
        self._info(f"[koch] 初始关节角(度): {self._fmt(self._current)}")

    # XL 系列单位换算：Profile_Velocity 单位 0.229rev/min≈1.374deg/s；
    #                 Profile_Acceleration 单位 214.577rev/min²≈214.577*6=1287.5deg/s²。
    _PVEL_DEG_S_PER_UNIT = 0.229 * 360.0 / 60.0        # ≈1.374
    _PACC_DEG_S2_PER_UNIT = 214.577 * 360.0 / 3600.0    # ≈21.46

    def _apply_profile_limits(self) -> None:
        """写 Profile_Velocity/Acceleration，让固件匀速平滑到位(限速+限加速)。"""
        pv = max(1, round(self.profile_velocity_deg_s / self._PVEL_DEG_S_PER_UNIT))
        pa = max(1, round(self.profile_accel_deg_s2 / self._PACC_DEG_S2_PER_UNIT))
        bus = self._robot.bus
        for m in self.joint_names:
            try:
                bus.write("Profile_Velocity", m, pv)
                bus.write("Profile_Acceleration", m, pa)
            except Exception as e:
                self._info(f"[koch] 写 {m} Profile 失败: {e}")
        # 关键安全校验：读回确认限速真的设上了。
        # 若 Profile_Velocity 仍为 0(=固件全速)，绝不能运动，否则电机会全速冲。
        try:
            back = self._retry_io(
                lambda: bus.sync_read("Profile_Velocity", normalize=False), "读回Profile")
        except Exception as e:
            raise SafetyError(f"无法读回 Profile_Velocity 校验限速({e})，拒绝运动")
        bad = [m for m in self.joint_names if int(back.get(m, 0)) < 1]
        if bad:
            raise SafetyError(
                f"限速未设成功(Profile_Velocity=0=全速)于 {bad}，拒绝运动。请检查串口/重试。")
        self._info(f"[koch] 固件限速已确认: Profile_Velocity={pv}(~{self.profile_velocity_deg_s}deg/s) "
                   f"Profile_Acceleration={pa}(~{self.profile_accel_deg_s2}deg/s²)")

    def disconnect(self, disable_torque: bool = False) -> None:
        """断开连接。

        disable_torque:
          False(默认，安全)—— 保持力矩，臂停在原地不动。用于异常中止/半空停止，避免砸落。
          True —— 主动松力矩。仅当臂已在低位安全姿态(如回到 HOME)时才用。
        """
        self._seg = None
        if self._robot is not None:
            if disable_torque:
                try:
                    self._robot.bus.disable_torque()
                    self._info("[koch] 已松力矩(安全低位)")
                except Exception as e:
                    self._info(f"[koch] 松力矩失败(保持力矩): {e}")
            else:
                self._info("[koch] 断开但保持力矩(臂停在原地，防砸落)")
            try:
                self._robot.disconnect()
            except Exception as e:
                self._info(f"[koch] disconnect 异常: {e}")
        self._connected = False
        self._info("[koch] 已断开")

    # ---- 读位置 ----
    def _read_pose(self) -> List[float]:
        obs = self._retry_io(self._robot.get_observation, "读位置")
        pose = []
        for name in self.joint_names:
            key = f"{name}.pos"
            if key not in obs:
                raise SafetyError(f"观测缺少关节 {key}；joint_names 与真机不匹配")
            pose.append(float(obs[key]))
        return pose

    def get_state_deg(self) -> List[float]:
        if self._connected:
            try:
                self._current = self._read_pose()
            except Exception as e:
                self._info(f"[koch] 读位置失败: {e}")
        return list(self._current)

    # ---- 安全校验（与 Mock 同一套） ----
    def _check_pose(self, pose: List[float], check_gripper_lock: bool = True) -> None:
        """校验位姿。

        check_gripper_lock:
          True  —— 用于“段目标(target)”校验：目标夹爪不许朝闭合走(必须>=张开阈值)。
          False —— 用于“逐帧插值”校验：只查软限位，不再拦夹爪。
                   原因：开机夹爪可能是闭合态(小值)，第一段把它张开(小->大)是安全方向，
                   逐帧会经过<阈值的中间值；只要“段目标”都>=阈值，夹爪就只会朝张开走、永不闭合。
        """
        if len(pose) != len(self.joint_names):
            raise SafetyError(f"关节数不匹配: {len(pose)} vs {len(self.joint_names)}")
        for name, val in zip(self.joint_names, pose):
            lo, hi = self.limits.joint_limits_deg[name]
            if not (lo <= val <= hi):
                raise SafetyError(f"关节 {name}={val:.1f} 超软限位 [{lo},{hi}]")
        if check_gripper_lock and self.limits.gripper_locked_open and "gripper" in self.joint_names:
            gi = self.joint_names.index("gripper")
            if pose[gi] < self.limits.gripper_open_value_deg - 1e-6:
                raise SafetyError(
                    f"夹爪锁定为张开(>= {self.limits.gripper_open_value_deg})，拒绝闭合到 {pose[gi]:.1f}")

    def _check_segment_velocity(self, seg: Segment) -> None:
        if seg.duration_s <= 0:
            raise SafetyError(f"段 {seg.name} 时长必须 > 0")
        for name, start, target in zip(self.joint_names, self._current, seg.target_deg):
            v = abs(target - start) / seg.duration_s
            if v > self.limits.max_joint_velocity_deg_s + 1e-6:
                raise SafetyError(
                    f"段 {seg.name} 关节 {name} 速度 {v:.1f}>限 {self.limits.max_joint_velocity_deg_s}")

    # ---- 运动（非阻塞） ----
    def start_segment(self, seg: Segment, now: float) -> None:
        if not self._connected:
            raise SafetyError("未连接，拒绝运动")
        if self._estopped:
            raise SafetyError("急停状态，拒绝运动")
        # 段起点用“真机实读位置”作为基准，而非我们跟踪的 _current。
        try:
            self._current = self._read_pose()
        except Exception as e:
            self._info(f"[koch] 段起点读位置失败({e})，用跟踪值兜底")
        self._check_pose(seg.target_deg)
        if not self.waypoint_mode:
            # 流式模式：软件按 duration 插值，需校验软件速度上限。
            # 路点模式：速度由固件 Profile_Velocity 限，软件不再按 duration 控速。
            self._check_segment_velocity(seg)
        self._seg = seg
        self._seg_start_t = now
        self._seg_start_pose = list(self._current)
        self._info(f"[koch] START {seg.name} 从 {self._fmt(self._current)} -> "
                   f"{self._fmt(seg.target_deg)}"
                   + ("" if self.waypoint_mode else f" in {seg.duration_s}s"))
        if self.waypoint_mode:
            # 只发一次目标；固件平滑到位。
            self._send(list(seg.target_deg))

    def update(self, now: float) -> None:
        if self._seg is None or self._estopped:
            return
        elapsed = now - self._seg_start_t
        if elapsed > self.limits.segment_timeout_s:
            self._info(f"[koch] SEGMENT TIMEOUT {self._seg.name} -> cancel")
            self.cancel()
            raise SafetyError(f"段 {self._seg.name} 超时")
        if self.waypoint_mode:
            # 路点模式：目标已在 start_segment 发出，固件自行平滑到位；此处不再发指令。
            return
        frac = min(max(elapsed / self._seg.duration_s, 0.0), 1.0)
        target = [s + (t - s) * frac
                  for s, t in zip(self._seg_start_pose, self._seg.target_deg)]
        # 逐帧校验：段目标已在 start_segment 校验为合法(软限位内)。
        # 插值在 [开机侧, 目标] 单调移动，允许“开机姿态本身的越界”存在(如开机肘更弯)，
        # 但绝不允许比开机姿态更远离合法区、或超出另一侧上界。
        self._check_interp_frame(target)
        self._send(target)
        self._current = target

    def _check_interp_frame(self, frame: List[float]) -> None:
        """逐帧插值校验：每关节须在 [min(下界, 开机值), max(上界, 开机值)] 内。

        含义：允许从越界的开机姿态朝合法区运动(安全方向)，
        但任何朝更越界方向、或越过合法区另一侧的值(真 bug) 仍被拦下。
        夹爪锁定不在逐帧查(见 _check_pose)。
        """
        for name, val, start in zip(self.joint_names, frame, self._seg_start_pose):
            lo, hi = self.limits.joint_limits_deg[name]
            eff_lo = min(lo, start)
            eff_hi = max(hi, start)
            if not (eff_lo - 1e-6 <= val <= eff_hi + 1e-6):
                raise SafetyError(
                    f"逐帧越界: {name}={val:.1f} 不在 [{eff_lo:.1f},{eff_hi:.1f}] "
                    f"(软限位[{lo},{hi}], 开机{start:.1f})")

    def _send(self, pose: List[float]) -> None:
        action = {f"{n}.pos": float(v) for n, v in zip(self.joint_names, pose)}
        if not self.enable_motion:
            # DRY-RUN：只打印，不发
            self._info(f"[koch:DRY] would send {self._fmt(pose)}")
            return
        self._retry_io(lambda: self._robot.send_action(action), "发送动作")

    def is_motion_done(self, now: float) -> bool:
        if self._seg is None:
            return True
        if self.waypoint_mode:
            # 路点模式：读实位，全部关节进入容差即完成；否则等待(受 segment_timeout 兜底)。
            if not self.enable_motion:
                # DRY-RUN 无真机运动，用时间兜底(给个名义时长)避免卡死。
                done = (now - self._seg_start_t) >= self._seg.duration_s
            else:
                try:
                    pose = self._read_pose()
                    self._current = pose
                    done = all(
                        abs(p - t) <= self.position_tolerance_deg
                        for p, t in zip(pose, self._seg.target_deg)
                    )
                except Exception as e:
                    self._info(f"[koch] 到位判定读位置失败({e})，继续等待")
                    done = False
            if done:
                self._current = list(self._seg.target_deg)
                self._info(f"[koch] DONE {self._seg.name}")
                self._seg = None
            return done
        # 流式模式：按时间完成
        done = (now - self._seg_start_t) >= self._seg.duration_s
        if done:
            self._send(list(self._seg.target_deg))
            self._current = list(self._seg.target_deg)
            self._info(f"[koch] DONE {self._seg.name}")
            self._seg = None
        return done

    def cancel(self) -> None:
        if self._seg is not None:
            self._info(f"[koch] CANCEL {self._seg.name}")
        self._seg = None

    def emergency_stop(self) -> None:
        self._estopped = True
        self._seg = None
        self._info("[koch] EMERGENCY STOP - 停止发送动作")
        # 注：是否让电机松力矩/保持，取决于安全策略；默认仅停止发送新指令。

    # ---- 工具 ----
    @staticmethod
    def _fmt(pose: List[float]) -> str:
        return "[" + ", ".join(f"{v:.1f}" for v in pose) + "]"
