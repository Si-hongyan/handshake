"""C 阶段 · 步骤②：手动示教，采集本机真实握手轨迹（只读关节角，不驱动电机）。

为什么需要：trajectory.yaml 里的角度是 Mock 占位假值，未在真机校准，
直接下发可能自碰撞/超限。必须在你这台臂上“手扶示教”出真实安全位姿。

流程（你全程手扶臂，电机不主动运动）：
  1. 脚本连接臂（calibrate=False，要求已用 lerobot-calibrate 校准过）；
  2. 提示你把臂摆到某个位姿（如 HOME），你摆好后按回车；
  3. 脚本 get_observation() 读回该位姿真实关节角；
  4. 依次采集 HOME / APPROACH / HANDSHAKE_UP / HANDSHAKE_DOWN / RETRACT；
  5. 写入 config/trajectory_koch.yaml（保留 Mock 版限位结构，仅替换 target_deg）。

安全：
  - 本脚本从不调用 send_action；
  - 建议示教时电机处于可徒手移动状态（force/torque off）。
    若 LeRobot 连接后电机上力矩导致摆不动，先中止，改用官方 teleop/记录流程，
    或在电机断力矩下示教。不要硬掰上了力矩的电机。

用法：
  ./.venv/bin/python run/robot_teach.py --port /dev/ttyACM0 --id koch_main \
      --config-dir config --out config/trajectory_koch.yaml
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml


POSES = ["HOME", "APPROACH", "HANDSHAKE_UP", "HANDSHAKE_DOWN", "RETRACT"]

PROMPTS = {
    "HOME": "把臂摆到 HOME（安全初始位，远离桌面/人）",
    "APPROACH": "把臂摆到 APPROACH（伸向握手区、但尚未到握手点）",
    "HANDSHAKE_UP": "把臂摆到 握手动作的 上 位",
    "HANDSHAKE_DOWN": "把臂摆到 握手动作的 下 位",
    "RETRACT": "把臂摆到 RETRACT（收回、准备回 HOME 前的过渡位）",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="koch_main")
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--out", default="config/trajectory_koch.yaml")
    args = ap.parse_args()

    try:
        from lerobot.robots.koch_follower import KochFollower, KochFollowerConfig
    except Exception as e:
        print(f"[FATAL] 导入 lerobot 失败: {e}")
        sys.exit(2)

    # 读 Mock 版 trajectory 作为结构模板（关节名、限位、时长、sequence）
    with open(os.path.join(args.config_dir, "trajectory.yaml"), encoding="utf-8") as f:
        template = yaml.safe_load(f)
    joint_names = template["joint_names"]

    cfg = KochFollowerConfig(
        port=args.port, id=args.id,
        use_degrees=True, disable_torque_on_disconnect=True,
    )
    robot = KochFollower(cfg)
    print(f"[teach] 连接 {args.port}（不发指令）...")
    robot.connect(calibrate=False)

    # 关键：connect()->configure() 会让电机上力矩(变硬)，示教需徒手掰动，
    # 因此连接后立即松掉全部力矩。松力矩后臂会因重力下垂——手要扶住！
    try:
        robot.bus.disable_torque()
        print("[teach] 已松开全部电机力矩（可徒手移动）。⚠️ 臂会因重力下垂，请扶稳！")
    except Exception as e:
        print(f"[teach] 松力矩失败: {e}")
        print("[teach] 为安全起见中止（避免徒手硬掰上力矩的电机）。")
        robot.disconnect()
        sys.exit(1)

    def read_pose():
        obs = robot.get_observation()
        return [round(float(obs[f"{n}.pos"]), 2) for n in joint_names]

    captured = {}
    try:
        print("\n=== 手动示教开始 ===")
        print("每个位姿：手扶臂摆好 → 按回车采集。Ctrl-C 可随时中止。\n")
        for name in POSES:
            input(f">>> [{name}] {PROMPTS[name]}\n    摆好后按回车采集...")
            pose = read_pose()
            captured[name] = pose
            print(f"    采集到 {name}: {pose}\n")
    except KeyboardInterrupt:
        print("\n[teach] 已中止，未写文件。")
        robot.disconnect()
        sys.exit(1)
    finally:
        robot.disconnect()

    # 组装输出：沿用模板的 limits/duration/sequence，仅替换 target_deg
    out = {
        "joint_names": joint_names,
        "limits": template["limits"],
        "segments": {},
        "sequence": template["sequence"],
        "_note": "本文件由 robot_teach.py 在真机上手动示教生成，target_deg 为真实采集值。",
    }
    for name in POSES:
        dur = template["segments"][name]["duration_s"]
        out["segments"][name] = {"target_deg": captured[name], "duration_s": dur}

    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
    print(f"[teach] 已写入 {args.out}")
    print("[teach] 请人工检查各角度合理、且在 limits.joint_limits_deg 范围内，再用于低速回放。")


if __name__ == "__main__":
    main()
