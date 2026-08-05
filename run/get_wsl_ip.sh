#!/usr/bin/env bash
# 打印 WSL 当前 eth0 IP —— Windows 视觉端 --send-host 要用这个值。
# NAT 模式下每次重启 WSL 该 IP 可能变化。
# 用法： bash run/get_wsl_ip.sh
ip -4 addr show eth0 2>/dev/null | grep -oP 'inet \K[\d.]+'
