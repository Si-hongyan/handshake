# Handshake Demo 

一个**可验证、低风险**的握手 Demo。

动作序列（固定、预定义，不由模型生成）：
`HOME → APPROACH → HANDSHAKE_UP → HANDSHAKE_DOWN → HANDSHAKE_UP → HANDSHAKE_DOWN → RETRACT → HOME`

---

## 架构
```
┌──────── Windows 原生 Python ────────┐   UDP    ┌──────── WSL2 Ubuntu ────────┐
│ camera(只留最新帧)                   │ 最新状态  │ receiver(只留最新)            │
│ → MediaPipe HandLandmarker(异步)    │ ───────► │ → watchdog → 状态机(20Hz)     │
│ → 张手判定 / ROI 判定                 │localhost│ → MockRobotController(打印)  │
│ → VisionState → UdpSender           │  51555   │                            │
│ + ROI 可视化 / 关键点 / 计时器        │           │ + 键盘急停/复位               │
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

## 安装

### WSL 侧
阶段一的仿真与单元测试**只用标准库 + 系统 PyYAML**，无需额外安装即可运行。
若你的 WSL Python 无 pip 且需要独立环境：
```bash
sudo apt-get update && sudo apt-get install -y python3-venv python3-pip   # 需要时
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements-wsl.txt
```

### Windows 侧
在 **Windows 原生 Python**（非 WSL）里：
```powershell
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements-windows.txt
```
下载 MediaPipe 模型到 `models/hand_landmarker.task`：
```
https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```
放到 `config/vision.yaml` 中 `mediapipe.model_path` 指定的路径（默认 `models/hand_landmarker.task`）。

---

## 运行命令

### ⚠️ WSL2 NAT 模式联调

默认 NAT 模式下，**Windows 不能用 `127.0.0.1` 把 UDP 发进 WSL**（localhost 转发主要走 TCP，UDP 不可靠）。
必须发往 **WSL 的 eth0 IP**，且该 IP 每次重启 WSL 可能变化。

- 取当前 IP（WSL 里跑）： `bash run/get_wsl_ip.sh`  → 例如 `172.17.36.149`
- Windows 视觉端指定目标：`--send-host <该IP>`，或直接用 `run_vision.bat`（自动查 IP）。
- 验证网络是否通（WSL 起监听后，Windows 发一个包）：
  ```
  python -c "import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(b'HELLO',('<WSL_IP>',51555)); print('sent')"
  ```
- 已实测：Windows 发到 WSL IP 可达，来源显示为 WSL NAT 网关（如 `172.17.32.1`），属正常。
- 若仍不通：检查 Windows 防火墙出站；或改用 WSL **mirrored 网络模式**（Win11 22H2+：`.wslconfig` 加 `[wsl2]\nnetworkingMode=mirrored`，此时 localhost 可直接用）。

### 视觉识别测试（Windows）
```powershell
python -m windows_vision.main --config-dir config
```
窗口内：手部关键点、ROI、六项视觉布尔、稳定计时器、FPS/延迟。
键位：`q` 退出，`s` 保存 ROI，鼠标左键拖拽重画 ROI。

### Windows 视觉 → WSL 状态机
```bash
# 1) WSL：启动状态机接收端
python3 run/run_sim_wsl.py --config-dir config --rate 20
```
```bat
:: 2) Windows（conda lerobot 环境，项目目录）：自动查 WSL IP 并启动视觉
run_vision.bat
:: 或手动： python -m windows_vision.main --config-dir config --send-host <WSL_IP>
```
触发方式：张开手掌 → 移进 ROI（框变绿）→ **保持不动约 1 秒**（stable=1）触发；
想看完整握手周期，触发后**手保持在位约 8 秒**。

### 状态机仿真
```bash
python3 run/run_sim_wsl.py --config-dir config --rate 20
```
键位：`e` 急停，`r` 复位，`q` 退出。无视觉源时会因超时进入 STOP（验证 watchdog）。


### 无 Windows 也能跑通全流程（WSL 本地自测）
开两个终端：
```bash
# 终端1
python3 run/run_sim_wsl.py --config-dir config --rate 20 --duration 13
# 终端2（0.5s 后）
python3 run/fake_vision_publisher.py --seconds 12          # 触发完整握手周期
python3 run/fake_vision_publisher.py --seconds 12 --drop-after 4   # 模拟运动中视觉丢失 -> 撤回
```

### 单元测试
```bash
python3 -m unittest discover -s tests -v
```

### 机械臂低速测试命令（**尚未启用**）
真机阶段将新增 `LeRobotController` 与 `run/run_robot_lowspeed.py`，
仅在阶段 A/B 全部通过、且完成校验后启用；当前仓库**不含**任何下发真机的代码路径。

---

## 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| WSL 收不到视觉，一直 STOP | Windows 防火墙拦 UDP / 端口不一致 | 允许 Python 入站；确认两侧 `network.yaml` port 一致 |
| sim 刚启动就 STOP | 视觉端未启动且超过 `startup_grace_s` | 先启动 sim，宽限期内启动视觉；或调大 `startup_grace_s` |
| 找不到 `hand_landmarker.task` | 模型未下载 | 按上文下载到 `model_path`；程序会拒绝启动 |
| 张手判定不准 | 阈值不合适 | 调 `vision.yaml: open_palm.min_extended_fingers` |
| 触发太灵敏/太迟钝 | dwell 太短/太长 | 调 `stable_gesture.dwell_seconds`（0.5~1.0）|
| 延迟高、卡顿 | 帧堆积 | 已只留最新帧；降分辨率/`target_fps`，确认 LIVE_STREAM |
| ROI 位置不对 | 默认矩形不合适 | 运行时拖拽后按 `s` 保存，或直接改 `roi.yaml` |
| WSL localhost 收不到 Windows 包 | 少数网络模式下 localhost 转发异常 | 用 `--rate` 先跑 fake_vision_publisher 自测；或改 `send_host` 为 WSL 的 eth0 IP |


---

## 开机复原
  1. 插好机械臂 USB + 上电

  2. Windows 管理员 PowerShell —— 重新 attach(bind 不用重做):
  usbipd attach --wsl --busid 6-1

  ▎ 若期间 wsl --shutdown 过,重启 WSL 后也是这一条。

  3. WSL 里验证串口回来:
  ls -l /dev/ttyACM0
  看到设备即可(dialout 权限还在)。

  4. 徒手把臂摆到大致 HOME 舒展姿态(减小首段行程)

  5. 查当前 WSL IP(NAT 模式重启可能变)给 Windows 视觉用:
  bash run/get_wsl_ip.sh

  6. 跑联调:
  WSL
  .venv/bin/python run/run_full_wsl.py --config-dir config --enable-motion --profile-velocity 15
   Windows(用第5步的IP)
  python -m windows_vision.main --config-dir config --send-host <WSL_IP>

  关键提醒

  - 每次 usbipd attach 是必须的(WSL重启/拔插后都要)——这是最容易忘的一步
  - NAT 模式 WSL IP 会变,Windows 视觉的 --send-host 每次用 get_wsl_ip.sh 查一下
  - 校准不用重做,除非拆过电机或换了臂
