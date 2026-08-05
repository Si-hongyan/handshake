"""安全状态机单元测试：正常流 + 手消失/断连/重复触发/超时/急停。

用真实 MockRobotController + 注入时间（now 显式传入），确定性可复现。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wsl_core.robot_controller import (
    MockRobotController, Limits, Segment,
)
from wsl_core.state_machine import HandshakeStateMachine, State, VisionInput, Events


def make_limits():
    return Limits(
        joint_limits_deg={
            "shoulder_pan": [-90, 90], "shoulder_lift": [-90, 45],
            "elbow_flex": [-10, 120], "wrist_flex": [-90, 90],
            "wrist_roll": [-180, 180], "gripper": [0, 40],
        },
        max_joint_velocity_deg_s=1000.0,   # 测试放宽，避免速度限制干扰
        max_step_delta_deg=1000.0,
        segment_timeout_s=100.0,
        gripper_locked_open=True,
        gripper_open_value_deg=30.0,
    )


JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
HOME = [0, -30, 60, 0, 0, 30]


def seg(name, target, dur=1.0):
    return Segment(name=name, target_deg=target, duration_s=dur)


def build_fsm(dwell=0.5):
    ctrl = MockRobotController(JOINTS, make_limits(), HOME, logger=None)
    ctrl.connect()
    approach = [seg("APPROACH", [10, -10, 70, -20, 0, 30])]
    handshake = [
        seg("HS_UP", [10, -5, 72, -10, 0, 30], 0.5),
        seg("HS_DOWN", [10, -15, 72, -30, 0, 30], 0.5),
        seg("HS_UP2", [10, -5, 72, -10, 0, 30], 0.5),
        seg("HS_DOWN2", [10, -15, 72, -30, 0, 30], 0.5),
    ]
    retract = [seg("RETRACT", [0, -30, 60, 0, 0, 30]), seg("HOME", HOME)]
    fsm = HandshakeStateMachine(ctrl, approach, handshake, retract, dwell_seconds=dwell)
    return fsm, ctrl


def good_vision():
    return VisionInput(conditions_met=True, hand_present=True, camera_alive=True, fresh=True)


def no_events():
    return Events()


class TestStateMachine(unittest.TestCase):
    def _run_to(self, fsm, target_state, t0, vision_fn, dt=0.1, max_s=60.0):
        """推进时钟直到到达 target_state，返回到达时间。"""
        t = t0
        while t < t0 + max_s:
            fsm.tick(t, vision_fn(), no_events())
            if fsm.state == target_state:
                return t
            t += dt
        raise AssertionError(f"未到达 {target_state}, 停在 {fsm.state}")

    def test_happy_path_full_cycle(self):
        fsm, ctrl = build_fsm(dwell=0.5)
        # IDLE -> WAIT_HAND
        fsm.tick(0.0, good_vision(), no_events())
        self.assertEqual(fsm.state, State.WAIT_HAND)
        # dwell 满足 -> APPROACH
        self._run_to(fsm, State.APPROACH, 0.0, good_vision)
        # 一路完成 -> 回到 WAIT_HAND（经 HANDSHAKE/RETRACT）
        t = self._run_to(fsm, State.WAIT_HAND, 1.0, good_vision, max_s=30)
        self.assertTrue(fsm.armed)  # 回 HOME 后重新 armed

    def test_requires_release_before_retrigger(self):
        """握完一次后，手一直在 ROI 不离开，则不再触发；离开再重入才触发。"""
        fsm, ctrl = build_fsm(dwell=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        self._run_to(fsm, State.APPROACH, 0.0, good_vision)
        t = self._run_to(fsm, State.WAIT_HAND, 1.0, good_vision, max_s=30)
        self.assertTrue(fsm._needs_release)  # 完成后要求释放

        # 手仍在 ROI(inside_zone=True) 且条件满足，持续 2s：不应再次触发
        in_zone = lambda: VisionInput(True, True, True, True, inside_zone=True)
        tt = t
        for _ in range(20):
            tt += 0.1
            fsm.tick(tt, in_zone(), no_events())
        self.assertEqual(fsm.state, State.WAIT_HAND)  # 仍在等待，未触发

        # 手离开 ROI 一次 -> 解除释放要求
        left = lambda: VisionInput(False, True, True, True, inside_zone=False)
        tt += 0.1
        fsm.tick(tt, left(), no_events())
        self.assertFalse(fsm._needs_release)

        # 手重入并稳定 -> 再次触发
        self._run_to(fsm, State.APPROACH, tt + 0.1, in_zone, max_s=5)

    def test_single_trigger_no_repeat(self):
        """一次稳定手势只触发一次；触发后 armed=False 直到回 HOME。"""
        fsm, ctrl = build_fsm(dwell=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        self._run_to(fsm, State.APPROACH, 0.0, good_vision)
        self.assertFalse(fsm.armed)  # 已触发，锁定
        # 在运动过程中即便条件持续满足，也不会再次触发新周期
        # （仍在 APPROACH/HANDSHAKE/RETRACT，不回 WAIT_HAND 就不 re-arm）
        self.assertIn(fsm.state, (State.APPROACH, State.HANDSHAKE, State.RETRACT))

    def test_estop_from_any_state(self):
        fsm, ctrl = build_fsm()
        fsm.tick(0.0, good_vision(), no_events())
        fsm.tick(0.1, good_vision(), Events(estop=True))
        self.assertEqual(fsm.state, State.STOP)
        self.assertTrue(ctrl._estopped)

    def test_estop_recover_with_reset(self):
        fsm, ctrl = build_fsm()
        fsm.tick(0.0, good_vision(), Events(estop=True))
        self.assertEqual(fsm.state, State.STOP)
        fsm.tick(0.1, good_vision(), Events(reset=True))
        self.assertEqual(fsm.state, State.IDLE)
        self.assertTrue(fsm.armed)

    def test_vision_timeout_when_idle_goes_stop(self):
        fsm, ctrl = build_fsm()
        fsm.tick(0.0, good_vision(), no_events())  # -> WAIT_HAND
        stale = VisionInput(conditions_met=False, hand_present=False, camera_alive=True, fresh=False)
        fsm.tick(0.2, stale, Events(vision_timeout=True))
        self.assertEqual(fsm.state, State.STOP)

    def test_camera_dead_when_waiting_goes_stop(self):
        fsm, ctrl = build_fsm()
        fsm.tick(0.0, good_vision(), no_events())
        dead = VisionInput(conditions_met=False, hand_present=False, camera_alive=False, fresh=True)
        fsm.tick(0.2, dead, no_events())
        self.assertEqual(fsm.state, State.STOP)

    def test_hand_disappear_during_motion_retracts(self):
        fsm, ctrl = build_fsm(dwell=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        self._run_to(fsm, State.APPROACH, 0.0, good_vision)
        # 手消失（仍在运动）-> RETRACT
        gone = VisionInput(conditions_met=False, hand_present=False, camera_alive=True, fresh=True)
        fsm.tick(5.0, gone, no_events())
        self.assertEqual(fsm.state, State.RETRACT)

    def test_vision_timeout_during_motion_retracts(self):
        fsm, ctrl = build_fsm(dwell=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        self._run_to(fsm, State.APPROACH, 0.0, good_vision)
        stale = VisionInput(conditions_met=False, hand_present=True, camera_alive=True, fresh=False)
        fsm.tick(5.0, stale, Events(vision_timeout=True))
        self.assertEqual(fsm.state, State.RETRACT)

    def test_gripper_close_rejected_goes_error(self):
        """尝试闭合夹爪的段应被安全校验拒绝 -> ERROR。"""
        ctrl = MockRobotController(JOINTS, make_limits(), HOME, logger=None)
        ctrl.connect()
        approach = [seg("BAD_APPROACH", [10, -10, 70, -20, 0, 5])]  # gripper=5 < 30 闭合
        fsm = HandshakeStateMachine(ctrl, approach, [], [], dwell_seconds=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        t = 0.0
        for _ in range(20):
            t += 0.1
            fsm.tick(t, good_vision(), no_events())
            if fsm.state == State.ERROR:
                break
        self.assertEqual(fsm.state, State.ERROR)

    def test_joint_limit_violation_goes_error(self):
        ctrl = MockRobotController(JOINTS, make_limits(), HOME, logger=None)
        ctrl.connect()
        approach = [seg("OOB", [200, -10, 70, -20, 0, 30])]  # pan=200 超限
        fsm = HandshakeStateMachine(ctrl, approach, [], [], dwell_seconds=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        t = 0.0
        for _ in range(20):
            t += 0.1
            fsm.tick(t, good_vision(), no_events())
            if fsm.state == State.ERROR:
                break
        self.assertEqual(fsm.state, State.ERROR)

    def test_dwell_interrupt_prevents_trigger(self):
        """dwell 期间条件中断，不应触发。"""
        fsm, ctrl = build_fsm(dwell=0.5)
        fsm.tick(0.0, good_vision(), no_events())
        fsm.tick(0.1, good_vision(), no_events())
        fsm.tick(0.2, good_vision(), no_events())
        bad = VisionInput(conditions_met=False, hand_present=True, camera_alive=True, fresh=True)
        fsm.tick(0.3, bad, no_events())  # 中断
        fsm.tick(0.4, good_vision(), no_events())
        self.assertEqual(fsm.state, State.WAIT_HAND)  # 尚未达 dwell


if __name__ == "__main__":
    unittest.main()
