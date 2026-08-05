# Windows 侧一键环境准备（阶段 A 视觉）
# 用法（在 Windows PowerShell 中，进入项目目录后运行）：
#   1) 先允许脚本执行： Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
#   2) 运行：           .\setup_windows.ps1
#
# 前置：Windows 上需已安装“真正的”Python（不是 Microsoft Store 存根）。
#   若 `python --version` 弹出商店或报错，请先从 https://www.python.org/downloads/
#   安装 Python 3.10~3.12，安装时勾选 “Add python.exe to PATH”。

$ErrorActionPreference = "Stop"

Write-Host "== 检查 Python ==" -ForegroundColor Cyan
$pyOk = $false
foreach ($cmd in @("py -3", "python")) {
    try {
        $v = & cmd /c "$cmd --version" 2>&1
        if ($v -match "Python 3\.(1[0-2]|[89])") { Write-Host "找到: $v ($cmd)"; $script:PY = $cmd; $pyOk = $true; break }
    } catch {}
}
if (-not $pyOk) {
    Write-Host "未找到可用的 Python 3.8~3.12。请先从 python.org 安装并勾选 Add to PATH。" -ForegroundColor Red
    exit 1
}

Write-Host "== 创建虚拟环境 .venv ==" -ForegroundColor Cyan
& cmd /c "$PY -m venv .venv"

Write-Host "== 升级 pip 并安装依赖 ==" -ForegroundColor Cyan
& .\.venv\Scripts\python.exe -m pip install --upgrade pip
& .\.venv\Scripts\python.exe -m pip install -r requirements-windows.txt

Write-Host "== 检查 MediaPipe 模型 ==" -ForegroundColor Cyan
if (Test-Path "models\hand_landmarker.task") {
    Write-Host "模型已存在: models\hand_landmarker.task"
} else {
    Write-Host "下载模型..."
    New-Item -ItemType Directory -Force -Path "models" | Out-Null
    Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task" -OutFile "models\hand_landmarker.task"
}

Write-Host "`n== 完成 ==" -ForegroundColor Green
Write-Host "启动阶段A视觉： .\.venv\Scripts\python.exe -m windows_vision.main --config-dir config"
