# Handshake Demo 

一个握手 Demo。

动作序列（固定、预定义，不由模型生成）：
`HOME → APPROACH → HANDSHAKE_UP → HANDSHAKE_DOWN → HANDSHAKE_UP → HANDSHAKE_DOWN → RETRACT → HOME`

---

## 架构
```
┌──────── Windows 原生 Python ────────┐   UDP    ┌──────── WSL2 Ubuntu ────────┐
│ camera(只留最新帧)                   │ 最新状态  │ receiver(只留最新)            │
│ → MediaPipe HandLandmarker(异步)    │ ───────► │ → watchdog → 状态机(20Hz)     │
│ → 张手判定 / ROI 判定                 │localhost│ → MockRobotController(打印)  │
│ → VisionState → UdpSender           │  51555   │  + 键盘急停/复位                            │
│ + ROI 可视化 / 关键点 / 计时器        │           │                            │
└──────────────────────────────────────┘         └──────────────────────────────┘
```

WSL2 默认没有 `/dev/video*`，USB 摄像头直通不稳定。让摄像头 + MediaPipe 留在 Windows 原生 Python，WSL 只收轻量 JSON 视觉状态。UDP 天然“丢旧留新”，避免帧堆积。

---

## 目录结构

```
handshake/
├─ config/
│  ├─ network.yaml       # UDP host/port、视觉超时、启动宽限
│  ├─ roi.yaml           # 归一化 ROI（可鼠标拖拽保存）
│  ├─ vision.yaml        # 摄像头/MediaPipe/张手/稳定/延迟阈值
│  └─ trajectory.yaml    # 关节名、软限位、各段目标角与时长（占位）
├─ shared/
│  └─ protocol.py        # VisionState JSON 协议（两侧共用）
├─ windows_vision/       # 【Windows 运行】
│  ├─ camera.py          # 只留最新帧的抓帧线程
│  ├─ hand_landmarker.py # MediaPipe Tasks LIVE_STREAM 封装
│  ├─ open_palm.py       # 张手判定（纯几何，无依赖，可测）
│  ├─ roi.py             # ROI 逻辑+可视化+鼠标拖拽
│  ├─ vision_state.py    # 稳定手势跟踪器（纯逻辑，可测）
│  ├─ sender.py          # UDP 发送
│  └─ main.py            # 视觉主程序
├─ wsl_core/             # 【WSL 运行】
│  ├─ receiver.py        # UDP 接收（只留最新）
│  ├─ state_machine.py   # 安全状态机（纯逻辑，可测）
│  ├─ robot_controller.py# 抽象接口 + MockRobotController（含安全校验）
│  ├─ watchdog.py        # 汇总安全信号
│  └─ logging_conf.py
├─ run/
│  ├─ run_sim_wsl.py         # 仿真主程序
│  └─ fake_vision_publisher.py # 测试工具：无 Windows 也能触发全流程
├─ tests/                # unittest（零第三方依赖）
├─ requirements-windows.txt
└─ requirements-wsl.txt
```
---

## 开始
  1. 插好机械臂 USB + 上电

  2. Windows 管理员 PowerShell —— 重新 attach:
  usbipd attach --wsl --busid 6-1

  ▎ 若期间 wsl --shutdown 过,重启 WSL 后也是这一条。

  3. WSL 里验证串口回来:
  ls -l /dev/ttyACM0
  看到设备即可。

  4. 徒手把臂摆到大致 HOME 舒展姿态(减小首段行程)

  5. 查当前 WSL IP(NAT 模式重启可能变)给 Windows 视觉用:
  bash run/get_wsl_ip.sh

  6. 跑联调:
  WSL
  .venv/bin/python run/run_full_wsl.py --config-dir config --enable-motion --profile-velocity 15
   Windows(用第5步的IP)
  python -m windows_vision.main --config-dir config --send-host <WSL_IP>

  关键提醒

  - 每次 usbipd attach 是必须的(WSL重启/拔插后都要)
  - NAT 模式 WSL IP 会变,Windows 视觉的 --send-host 每次用 get_wsl_ip.sh 查一下
