"""阶段 B 仿真主程序（WSL 侧，非阻塞主循环）。

- UDP 接收最新视觉状态（Windows 侧发来）；
- Watchdog 汇总安全信号；
- 状态机以 ~20 Hz tick；
- MockRobotController 打印 APPROACH/HANDSHAKE/RETRACT，不接硬件；
- 键盘 'e' 急停，'r' 复位，'q' 退出（非阻塞读取 stdin）。

用法：
  python3 run/run_sim_wsl.py --config-dir config --rate 20
  # 无视觉源时也能启动，会因视觉超时进入 STOP（用于验证 watchdog）。
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
from wsl_core.robot_controller import (  # noqa: E402
    MockRobotController, load_limits, build_segment,
)
from wsl_core.state_machine import HandshakeStateMachine, State  # noqa: E402


def _load(cfg_dir, name):
    with open(os.path.join(cfg_dir, name), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _poll_key() -> str | None:
    """非阻塞读取一个字符（stdin 为 tty 时）。"""
    if not sys.stdin.isatty():
        return None
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.read(1)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--rate", type=float, default=20.0, help="状态机频率 Hz (10~20)")
    ap.add_argument("--log-file", default="logs/sim.log")
    ap.add_argument("--duration", type=float, default=0.0, help=">0 时运行该秒数后自动退出")
    args = ap.parse_args()

    log = setup_logging("INFO", args.log_file)

    net = _load(args.config_dir, "network.yaml")
    traj = _load(args.config_dir, "trajectory.yaml")
    vision_cfg = _load(args.config_dir, "vision.yaml")

    limits = load_limits(traj)
    joint_names = traj["joint_names"]
    home_deg = list(traj["segments"]["HOME"]["target_deg"])
    seq = traj["sequence"]

    approach = [build_segment(n, traj) for n in seq["approach"]]
    handshake = [build_segment(n, traj) for n in seq["handshake"]]
    retract = [build_segment(n, traj) for n in seq["retract"]]

    controller = MockRobotController(joint_names, limits, home_deg, logger=log)
    controller.connect()

    fsm = HandshakeStateMachine(
        controller, approach, handshake, retract,
        dwell_seconds=vision_cfg["stable_gesture"]["dwell_seconds"], logger=log,
    )

    recv = UdpReceiver(net["udp"]["bind_host"], net["udp"]["port"], net["udp"]["max_datagram_bytes"])
    wd = Watchdog(WatchdogConfig(
        vision_timeout_s=float(net["vision_timeout_s"]),
        startup_grace_s=float(net.get("startup_grace_s", 5.0)),
    ))

    period = 1.0 / max(args.rate, 1.0)
    log.info(f"[sim] 启动: rate={args.rate}Hz port={net['udp']['port']} "
             f"vision_timeout={net['vision_timeout_s']}s")
    log.info("[sim] 键位: e=急停 r=复位 q=退出")

    estop_latch = False
    t_start = time.time()
    try:
        while True:
            now = time.time()
            recv.poll()  # 只留最新
            last = recv.last_state
            since = recv.seconds_since_last(now)

            key = _poll_key()
            reset_req = False
            if key == "e":
                estop_latch = True
                log.info("[sim] 键盘急停")
            elif key == "r":
                reset_req = True
                estop_latch = False
                log.info("[sim] 键盘复位")
            elif key == "q":
                log.info("[sim] 退出")
                break

            vision, events = wd.build_inputs(now, last, since, estop=estop_latch)
            if reset_req:
                events.reset = True

            fsm.tick(now, vision, events)

            if args.duration > 0 and (now - t_start) >= args.duration:
                log.info("[sim] 到达 --duration，退出")
                break

            time.sleep(period)
    except KeyboardInterrupt:
        log.info("[sim] KeyboardInterrupt")
    finally:
        # 程序退出前确保不再发送动作（安全第 11 条）。
        controller.emergency_stop()
        controller.disconnect()
        recv.close()
        log.info("[sim] 已安全退出")


if __name__ == "__main__":
    main()
