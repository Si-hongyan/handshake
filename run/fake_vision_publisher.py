"""测试工具：伪造视觉状态发布器（WSL 本地，用于在没有 Windows 摄像头时验证阶段 B）。

它模拟 Windows 视觉端，通过 UDP 向状态机发送“张开手掌进入 ROI 且稳定”的视觉状态，
让你在纯 WSL 内跑通 触发->握手->撤回->HOME 全流程。

用法（另开一个终端，先启动 run_sim_wsl.py，再运行本脚本）：
  python3 run/fake_vision_publisher.py --seconds 6 --good-after 0.0
选项 --drop-after N ：N 秒后停止发送（模拟摄像头断开 / 手消失 -> 触发撤回/STOP）。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml  # noqa: E402
from shared.protocol import VisionState  # noqa: E402
from windows_vision.sender import UdpSender  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--drop-after", type=float, default=-1.0,
                    help=">0 时在该秒后停止发送，模拟视觉丢失")
    args = ap.parse_args()

    with open(os.path.join(args.config_dir, "network.yaml"), encoding="utf-8") as f:
        net = yaml.safe_load(f)

    sender = UdpSender(net["udp"]["send_host"], net["udp"]["port"])
    period = 1.0 / args.rate
    t0 = time.time()
    seq = 0
    print(f"[fake] 向 {net['udp']['send_host']}:{net['udp']['port']} 发送模拟视觉状态")
    try:
        while time.time() - t0 < args.seconds:
            now = time.time()
            if args.drop_after > 0 and (now - t0) >= args.drop_after:
                print("[fake] drop-after 到达，停止发送（模拟视觉丢失）")
                break
            seq += 1
            st = VisionState(
                seq=seq, t_send=now,
                hand_detected=True, open_palm=True, inside_zone=True,
                stable_gesture=True, camera_alive=True, latency_ok=True,
                confidence=0.95, wrist_xy=(0.65, 0.5), latency_ms=20.0, fps=args.rate,
            )
            sender.send(st)
            time.sleep(period)
    finally:
        sender.close()
        print("[fake] 结束")


if __name__ == "__main__":
    main()
