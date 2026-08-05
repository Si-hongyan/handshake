@echo off
REM ========================================================================
REM 阶段 A/联调 一键启动（Windows 侧，在 conda lerobot 环境里运行）
REM   自动向 WSL 查询当前 eth0 IP，作为 UDP 目标，避免每次手输。
REM 用法（Anaconda Prompt，已 conda activate lerobot，cd 到项目目录）：
REM   run_vision.bat
REM ========================================================================
setlocal

REM 从 WSL 取当前 IP（NAT 模式每次重启可能变）
for /f "usebackq tokens=1" %%i in (`wsl hostname -I`) do set WSL_IP=%%i

if "%WSL_IP%"=="" (
  echo [ERROR] 无法从 WSL 获取 IP。请确认 WSL 正在运行： wsl -l -v
  exit /b 1
)

echo [INFO] WSL IP = %WSL_IP%
echo [INFO] 启动视觉，UDP 目标 %WSL_IP%:51555
python -m windows_vision.main --config-dir config --send-host %WSL_IP% %*

endlocal
