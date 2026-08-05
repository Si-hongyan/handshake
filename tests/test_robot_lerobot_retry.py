"""LeRobotKochController 重试逻辑单元测试（用假 robot，不接硬件、不导入 lerobot）。"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wsl_core.robot_lerobot import LeRobotKochController
from wsl_core.robot_controller import Limits


JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def make_limits():
    return Limits(
        joint_limits_deg={n: [-180, 180] for n in JOINTS},
        max_joint_velocity_deg_s=1000.0, max_step_delta_deg=1000.0,
        segment_timeout_s=100.0, gripper_locked_open=True, gripper_open_value_deg=30.0,
    )


def make_ctrl():
    return LeRobotKochController(
        port="/dev/null", robot_id="test", joint_names=JOINTS, limits=make_limits(),
        io_retries=3, io_retry_delay_s=0.0,
    )


class TestRetryIO(unittest.TestCase):
    def test_succeeds_first_try(self):
        ctrl = make_ctrl()
        calls = {"n": 0}
        def f():
            calls["n"] += 1
            return "ok"
        self.assertEqual(ctrl._retry_io(f, "test"), "ok")
        self.assertEqual(calls["n"], 1)

    def test_recovers_after_transient(self):
        ctrl = make_ctrl()
        calls = {"n": 0}
        def f():
            calls["n"] += 1
            if calls["n"] < 3:
                raise ConnectionError("no status packet")
            return "recovered"
        self.assertEqual(ctrl._retry_io(f, "test"), "recovered")
        self.assertEqual(calls["n"], 3)

    def test_raises_after_all_fail(self):
        ctrl = make_ctrl()
        calls = {"n": 0}
        def f():
            calls["n"] += 1
            raise ConnectionError("persistent failure")
        with self.assertRaises(ConnectionError):
            ctrl._retry_io(f, "test")
        self.assertEqual(calls["n"], 3)  # 用尽 io_retries 次

    def test_dry_run_default(self):
        """默认 enable_motion=False，_send 不应真正发送。"""
        ctrl = make_ctrl()
        self.assertFalse(ctrl.enable_motion)


class TestGripperLock(unittest.TestCase):
    """夹爪锁定语义：段目标必须张开；逐帧不拦（允许开机闭合->张开）。"""

    def _ctrl(self):
        lim = make_limits()
        lim.joint_limits_deg["gripper"] = [0.0, 45.0]   # 物理全程
        lim.gripper_open_value_deg = 35.0
        from wsl_core.robot_lerobot import LeRobotKochController
        return LeRobotKochController("/dev/null", "t", JOINTS, lim, io_retry_delay_s=0.0)

    def test_target_open_accepted(self):
        from wsl_core.robot_controller import SafetyError
        c = self._ctrl()
        c._check_pose([0, -50, -30, -40, 8, 38], check_gripper_lock=True)  # 不抛错即通过

    def test_target_closing_rejected(self):
        from wsl_core.robot_controller import SafetyError
        c = self._ctrl()
        with self.assertRaises(SafetyError):
            c._check_pose([0, -50, -30, -40, 8, 10], check_gripper_lock=True)

    def test_perframe_closed_allowed(self):
        """逐帧(check_gripper_lock=False)允许开机闭合值(4.6)存在。"""
        c = self._ctrl()
        c._check_pose([2.8, -33.5, -55.9, -59.0, 11.5, 4.6], check_gripper_lock=False)


class TestInterpFrameBounds(unittest.TestCase):
    """逐帧插值校验：允许从越界开机姿态朝合法区走，拦住更越界/越另一侧。"""

    def _ctrl(self, start):
        from wsl_core.robot_lerobot import LeRobotKochController
        lim = make_limits()
        lim.joint_limits_deg["elbow_flex"] = [-64.8, -1.5]
        c = LeRobotKochController("/dev/null", "t", JOINTS, lim, io_retry_delay_s=0.0)
        c._seg_start_pose = start
        return c

    def test_toward_valid_from_out_of_range_allowed(self):
        c = self._ctrl([0, 0, -66.6, 0, 0, 38])   # 开机肘越界
        c._check_interp_frame([0, 0, -60.0, 0, 0, 38])  # 朝合法区，不抛错

    def test_further_out_of_range_rejected(self):
        from wsl_core.robot_controller import SafetyError
        c = self._ctrl([0, 0, -66.6, 0, 0, 38])
        with self.assertRaises(SafetyError):
            c._check_interp_frame([0, 0, -70.0, 0, 0, 38])   # 更越界

    def test_overshoot_other_limit_rejected(self):
        from wsl_core.robot_controller import SafetyError
        c = self._ctrl([0, 0, -66.6, 0, 0, 38])
        with self.assertRaises(SafetyError):
            c._check_interp_frame([0, 0, 10.0, 0, 0, 38])    # 越过上界


if __name__ == "__main__":
    unittest.main()
