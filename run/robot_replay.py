"""C 阶段 · 真机回放（无视觉，纯固定序列）。

两种模式:
  C1  --dry-run（默认）: 真的 connect() 上电+读位置，但只打印 would-send，绝不发运动指令。
  C2  --enable-motion   : 真正发指令让臂运动。需显式加此参数，且强烈建议配 --speed-scale 放慢。

安全:
  - 用 config/trajectory_koch.yaml（真机示教+安全修正版）；
  - 复用状态机的段驱动 + 全套安全校验（软限位/速度/单步/夹爪锁定/段超时）；
  - Ctrl-C 立即 cancel + emergency_stop + disconnect（断开即松力矩）；
  - 任意 SafetyError -> 立即停止发送并退出；
  - --speed-scale 放慢所有段时长（C2 首跑建议 2.0~3.0，即慢 2~3 倍）。

用法:
  # C1 dry-run 连真机(只读不发)
  ./.venv/bin/python run/robot_replay.py --config-dir config --port /dev/ttyACM0 --id koch_main --dry-run
  # C2 真动(极低速, 人在急停旁)
  ./.venv/bin/python run/robot_replay.py --config-dir config --port /dev/ttyACM0 --id koch_main \
      --enable-motion --speed-scale 3.0
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml  # noqa: E402

from wsl_core.logging_conf import setup_logging  # noqa: E402
from wsl_core.robot_controller import load_limits, build_segment, SafetyError  # noqa: E402
from wsl_core.robot_lerobot import LeRobotKochController  # noqa: E402
from wsl_core.state_machine import HandshakeStateMachine, State, VisionInput, Events  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--traj", default="trajectory_koch.yaml",
                    help="轨迹文件名(默认真机示教版)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="koch_main")
    ap.add_argument("--rate", type=float, default=20.0, help="控制 tick 频率 Hz")
    ap.add_argument("--speed-scale", type=float, default=1.0,
                    help="段时长放大倍数(>1 更慢)。C2 首跑建议 2~3")
    ap.add_argument("--max-relative-target", type=float, default=15.0,
                    help="LeRobot 单步位置变化上限(度)。默认15，避免钳死平滑插值；"
                         "我们自己已有软限位/速度/逐帧步长保护，此为二级兜底")
    ap.add_argument("--streaming", action="store_true",
                    help="用旧的流式插值(5Hz)执行，回退方案。默认用路点+固件限速(更丝滑抗丢包)")
    ap.add_argument("--profile-velocity", type=float, default=18.0,
                    help="路点模式固件限速(deg/s)，默认18")
    ap.add_argument("--profile-accel", type=float, default=60.0,
                    help="路点模式固件限加速(deg/s²)，默认60")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", default=True,
                      help="C1: 连真机但只读不发(默认)")
    mode.add_argument("--enable-motion", action="store_true",
                      help="C2: 真正发指令运动(需显式)")
    ap.add_argument("--log-file", default="logs/replay.log")
    args = ap.parse_args()

    log = setup_logging("INFO", args.log_file)
    # LeRobot 的 "Relative goal position ... clamped" 是 root logger 的 WARNING，
    # 会刷屏淹没我们的日志。max_relative_target 调大后本就不该频繁触发；这里压到 ERROR。
    import logging as _logging
    _logging.getLogger().setLevel(_logging.ERROR)
    enable_motion = bool(args.enable_motion)  # 只有显式 --enable-motion 才发指令

    traj = yaml.safe_load(open(os.path.join(args.config_dir, args.traj), encoding="utf-8"))
    limits = load_limits(traj)
    jn = traj["joint_names"]
    seq = traj["sequence"]

    # 应用 speed-scale：放慢所有段
    scale = max(args.speed_scale, 1.0)
    for s in traj["segments"].values():
        s["duration_s"] = float(s["duration_s"]) * scale

    approach = [build_segment(n, traj) for n in seq["approach"]]
    handshake = [build_segment(n, traj) for n in seq["handshake"]]
    retract = [build_segment(n, traj) for n in seq["retract"]]

    banner = "C2 真动模式 ⚠️" if enable_motion else "C1 DRY-RUN(只读, 不发指令)"
    log.info("=" * 60)
    log.info(f"[replay] 模式: {banner}")
    log.info(f"[replay] 轨迹: {args.traj} | speed_scale={scale} | rate={args.rate}Hz")
    if enable_motion:
        log.info("[replay] ⚠️ 将真正发送运动指令。确认: 泡沫块/假手、人在急停旁、量程内无障碍!")
    log.info("=" * 60)

    ctrl = LeRobotKochController(
        port=args.port, robot_id=args.id, joint_names=jn, limits=limits,
        max_relative_target=args.max_relative_target,
        enable_motion=enable_motion,
        waypoint_mode=(not args.streaming),
        profile_velocity_deg_s=args.profile_velocity,
        profile_accel_deg_s2=args.profile_accel,
        logger=log,
    )

    try:
        ctrl.connect()   # 真连接: 上电+读位置(C1/C2 都会上力矩)
    except Exception as e:
        log.error(f"[replay] 连接失败: {e}")
        sys.exit(1)

    fsm = HandshakeStateMachine(ctrl, approach, handshake, retract,
                                dwell_seconds=0.1, logger=log)

    # 无视觉: 直接注入"条件满足"触发一次序列; 中途不再需要视觉
    def vision_ok():
        return VisionInput(conditions_met=True, hand_present=True,
                           camera_alive=True, fresh=True)

    period = 1.0 / max(args.rate, 1.0)
    t0 = time.time()
    triggered_once = False
    completed_at_home = False   # 仅正常回到 HOME 才为 True
    try:
        # 启动: IDLE->WAIT_HAND
        fsm.tick(time.time(), vision_ok(), Events())
        while True:
            now = time.time()
            fsm.tick(now, vision_ok(), Events())

            if fsm.state == State.ERROR:
                log.error("[replay] 进入 ERROR，停止。")
                break
            # 完成一圈: 触发过且回到 WAIT_HAND
            if fsm.state in (State.APPROACH, State.HANDSHAKE, State.RETRACT):
                triggered_once = True
            if triggered_once and fsm.state == State.WAIT_HAND and fsm.armed:
                log.info("[replay] 一次完整握手序列回放完成，回到 HOME。")
                completed_at_home = True
                break
            if now - t0 > 120:
                log.warning("[replay] 超时 120s，退出。")
                break
            time.sleep(period)
    except KeyboardInterrupt:
        log.info("[replay] Ctrl-C: 取消 + 急停")
        fsm.tick(time.time(), vision_ok(), Events(estop=True))
    except SafetyError as e:
        log.error(f"[replay] SafetyError: {e}")
        ctrl.emergency_stop()
    finally:
        ctrl.emergency_stop()   # 退出前确保不再发指令
        # 安全策略：仅当“正常回到 HOME(低位安全)”才松力矩；
        # 任何异常中止(半空)保持力矩、臂停在原地，避免松力矩砸落。
        if completed_at_home:
            ctrl.disconnect(disable_torque=True)
        else:
            log.warning("[replay] 非正常结束：保持力矩，臂停在原地。请手动扶稳后断电。")
            ctrl.disconnect(disable_torque=False)
        log.info("[replay] 已安全退出。")


if __name__ == "__main__":
    main()
