#!/bin/bash
# 急停测试脚本
# 用法: ./test_estop.sh

echo "========================================"
echo "急停测试"
echo "发送急停信号..."
echo "========================================"

ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.0 \
    arm_angle:=0.0 \
    shovel_angle:=0.0 \
    steering_target:=0.0 \
    estop:=true \
    publish_rate:=10.0

