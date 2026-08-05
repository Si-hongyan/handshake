"""视觉联调主程序：真实视觉(Windows) → WSL 状态机 → 真机 Koch(路点+固件限速)。

这是完整闭环：MediaPipe 手势触发 → 机械臂执行预示教握手。
默认 DRY-RUN(只读不发)，必须显式 --enable-motion 才真动。

安全(沿用全套)：
  - watchdog：摄像头断/视觉超时/手消失(运动中) → 撤回或 STOP；
  - 状态机：dwell 触发、单次触发防重复、软限位/夹爪锁定/段超时；
  - 路点模式：固件 Profile_Velocity 限速，写指令少、抗 usbipd 丢包；
  - 异常/退出：保持力矩不砸落(仅正常回 HOME 才松力矩)；
  - 键盘 e=急停 r=复位 q=退出。

安全测试顺序(务必遵守)：
  1) --dry-run(默认)：真视觉触发，机械臂只打印不动，验证联调链路；
  2) --enable-motion + 泡沫块/假手 + 无人 + 手在急停旁：真动，验证视觉触发真机；
  3) 软质末端、先接近不接触；4) 真人测试(仅前面全过后)。

用法：
  # 阶段联调 DRY-RUN（真视觉，臂不动）
  python3 run/run_full_wsl.py --config-dir config --dry-run
  # 真动（泡沫块、无人、手在急停旁）
  python3 run/run_full_wsl.py --config-dir config --enable-motion --profile-velocity 18
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml  # noqa: E402

from wsl_core.logging_conf import setup_logging  # noqa: E402
from wsl_core.receiver import UdpReceiver  # noqa: E402
from wsl_core.watchdog import Watchdog, WatchdogConfig  # noqa: E402
from wsl_core.robot_controller import load_limits, build_segment  # noqa: E402
from wsl_core.robot_lerobot import LeRobotKochController  # noqa: E402
from wsl_core.state_machine import HandshakeStateMachine, State  # noqa: E402


def _load(cfg_dir, name):
    with open(os.path.join(cfg_dir, name), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _poll_key():
    if not sys.stdin.isatty():
        return None
    r, _, _ = select.select([sys.stdin], [], [], 0)
    return sys.stdin.read(1) if r else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--traj", default="trajectory_koch.yaml", help="真机示教轨迹")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="koch_main")
    ap.add_argument("--rate", type=float, default=15.0, help="状态机/轮询频率 Hz")
    ap.add_argument("--profile-velocity", type=float, default=18.0)
    ap.add_argument("--profile-accel", type=float, default=60.0)
    ap.add_argument("--max-relative-target", type=float, default=15.0)
    ap.add_argument("--streaming", action="store_true", help="回退流式(默认路点)")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", default=True,
                      help="默认：真视觉触发但臂只读不发")
    mode.add_argument("--enable-motion", action="store_true", help="真动(需显式)")
    ap.add_argument("--log-file", default="logs/full.log")
    args = ap.parse_args()

    log = setup_logging("INFO", args.log_file)
    import logging as _logging
    _logging.getLogger().setLevel(_logging.ERROR)  # 压 LeRobot 钳位刷屏
    enable_motion = bool(args.enable_motion)

    net = _load(args.config_dir, "network.yaml")
    traj = _load(args.config_dir, args.traj)
    vision_cfg = _load(args.config_dir, "vision.yaml")

    limits = load_limits(traj)
    jn = traj["joint_names"]
    seq = traj["sequence"]
    approach = [build_segment(n, traj) for n in seq["approach"]]
    handshake = [build_segment(n, traj) for n in seq["handshake"]]
    retract = [build_segment(n, traj) for n in seq["retract"]]

    banner = "真动 ⚠️" if enable_motion else "DRY-RUN(臂只读不发)"
    log.info("=" * 60)
    log.info(f"[full] 视觉联调 | 模式: {banner} | 轨迹: {args.traj}")
    if enable_motion:
        log.info("[full] ⚠️ 视觉将触发真机运动！确认: 泡沫块/假手、无人、手在急停旁!")
    log.info("=" * 60)

    ctrl = LeRobotKochController(
        port=args.port, robot_id=args.id, joint_names=jn, limits=limits,
        max_relative_target=args.max_relative_target, enable_motion=enable_motion,
        waypoint_mode=(not args.streaming),
        profile_velocity_deg_s=args.profile_velocity,
        profile_accel_deg_s2=args.profile_accel, logger=log,
    )
    try:
        ctrl.connect()
    except Exception as e:
        log.error(f"[full] 连接机械臂失败: {e}")
        sys.exit(1)

    fsm = HandshakeStateMachine(
        ctrl, approach, handshake, retract,
        dwell_seconds=vision_cfg["stable_gesture"]["dwell_seconds"], logger=log,
    )
    recv = UdpReceiver(net["udp"]["bind_host"], net["udp"]["port"], net["udp"]["max_datagram_bytes"])
    wd = Watchdog(WatchdogConfig(
        vision_timeout_s=float(net["vision_timeout_s"]),
        startup_grace_s=float(net.get("startup_grace_s", 5.0)),
    ))

    period = 1.0 / max(args.rate, 1.0)
    log.info(f"[full] 监听 {net['udp']['port']} | dwell={fsm.dwell_seconds}s "
             f"| 键位 e=急停 r=复位 q=退出")

    estop_latch = False
    reached_home_once = False
    try:
        while True:
            now = time.time()
            recv.poll()
            last = recv.last_state
            since = recv.seconds_since_last(now)

            key = _poll_key()
            reset_req = False
            if key == "e":
                estop_latch = True; log.info("[full] 键盘急停")
            elif key == "r":
                reset_req = True; estop_latch = False; log.info("[full] 键盘复位")
            elif key == "q":
                log.info("[full] 退出"); break

            vision, events = wd.build_inputs(now, last, since, estop=estop_latch)
            if reset_req:
                events.reset = True

            prev = fsm.state
            fsm.tick(now, vision, events)
            # 记录是否曾正常回到 HOME(用于退出时决定松力矩)
            if prev in (State.RETRACT,) and fsm.state == State.WAIT_HAND:
                reached_home_once = True

            time.sleep(period)
    except KeyboardInterrupt:
        log.info("[full] KeyboardInterrupt: 急停")
        try:
            ctrl.emergency_stop()
        except Exception:
            pass
    finally:
        ctrl.emergency_stop()
        # 仅当当前处于 HOME/等待态(低位安全)才松力矩；否则保持力矩防砸落。
        safe_low = fsm.state in (State.WAIT_HAND, State.IDLE, State.STOP) and reached_home_once
        if safe_low:
            ctrl.disconnect(disable_torque=True)
        else:
            log.warning("[full] 非低位安全结束：保持力矩，臂停在原地，请手动扶稳后断电。")
            ctrl.disconnect(disable_torque=False)
        recv.close()
        log.info("[full] 已安全退出")


if __name__ == "__main__":
    main()
