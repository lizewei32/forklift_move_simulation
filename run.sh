#!/bin/bash
# 运行脚本 - 自动激活虚拟环境并运行仿真

cd "$(dirname "$0")"

# 激活虚拟环境
source venv/bin/activate

# 运行仿真
echo "=========================================="
echo "无人叉车路径仿真软件"
echo "=========================================="
echo ""

python control_sim_monitor.py

echo ""
echo "仿真完成!"
