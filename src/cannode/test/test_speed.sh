#!/bin/bash
# 速度控制测试脚本
# 用法: ./test_speed.sh [速度值]
# 示例: ./test_speed.sh 0.5

SPEED=${1:-0.3}  # 默认速度 0.3 m/s

echo "========================================"
echo "速度控制测试"
echo "目标速度: ${SPEED} m/s"
echo "========================================"

ros2 launch cannode send_control_cmd.launch.py \
    speed:=${SPEED} \
    arm_angle:=0.0 \
    shovel_angle:=0.0 \
    steering_target:=0.0 \
    publish_rate:=10.0

