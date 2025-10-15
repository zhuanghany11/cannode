#!/bin/bash
# 大臂角度控制测试脚本
# 用法: ./test_arm.sh [角度值]
# 示例: ./test_arm.sh 30.0

ARM_ANGLE=${1:-30.0}  # 默认角度 30.0 度

echo "========================================"
echo "大臂角度控制测试"
echo "目标角度: ${ARM_ANGLE} 度"
echo "========================================"

ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.0 \
    arm_angle:=${ARM_ANGLE} \
    shovel_angle:=0.0 \
    steering_target:=0.0 \
    arm_enable:=true \
    publish_rate:=10.0

