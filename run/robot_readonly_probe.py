"""C 阶段 · 步骤①：只读连接测试（不发送任何电机指令）。

作用：
  - 用 LeRobot KochFollower 连接 /dev/ttyACM0；
  - 读取当前关节位置 get_observation()；
  - 打印校准状态；
  - 全程不调用 send_action，电机不动。

用法（WSL venv，已 usbipd 直通 + 在 dialout 组）：
  ./.venv/bin/python run/robot_readonly_probe.py --port /dev/ttyACM0 --id koch_main

若首次运行报"未校准"，说明该臂还没在本机校准过，需要先跑 LeRobot 官方校准：
  lerobot-calibrate --robot.type=koch_follower --robot.port=/dev/ttyACM0 --robot.id=koch_main
校准也会让你手动移动关节，但那是 LeRobot 官方流程，属读取标定、非自主运动。
"""

from __future__ import annotations

import argparse
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="koch_main")
    args = ap.parse_args()

    # 延迟导入，未装 lerobot 时给清晰提示
    try:
        from lerobot.robots.koch_follower import KochFollower, KochFollowerConfig
    except Exception as e:
        print(f"[FATAL] 无法导入 lerobot koch_follower: {e}")
        print("请确认已在 WSL venv 安装 lerobot v0.6.0。")
        sys.exit(2)

    print(f"[probe] 连接 {args.port} id={args.id}（只读，不发指令）")
    cfg = KochFollowerConfig(
        port=args.port, id=args.id,
        use_degrees=True, disable_torque_on_disconnect=True,
    )

    robot = KochFollower(cfg)
    try:
        # calibrate=False：不触发校准流程，只尝试连接
        robot.connect(calibrate=False)
    except Exception as e:
        print(f"[probe] connect(calibrate=False) 失败: {e}")
        print("  常见原因：①未校准(先跑 lerobot-calibrate) ②端口不对 ③波特率/电机ID未设")
        try:
            robot.disconnect()
        except Exception:
            pass
        sys.exit(1)

    try:
        is_cal = getattr(robot, "is_calibrated", None)
        print(f"[probe] is_calibrated = {is_cal}")

        obs = robot.get_observation()
        print("[probe] 当前观测（关节位置等）:")
        for k, v in obs.items():
            if k.endswith(".pos"):
                print(f"    {k:24s} = {v}")
        # 也打印非 .pos 键的键名（可能有电压/温度等）
        other = [k for k in obs if not k.endswith(".pos")]
        if other:
            print("[probe] 其它观测键:", other)
    finally:
        robot.disconnect()
        print("[probe] 已断开。全程未发送任何电机指令。")


if __name__ == "__main__":
    main()
