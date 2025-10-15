#!/bin/bash
# 铲斗角度控制测试脚本
# 用法: ./test_shovel.sh [角度值]
# 示例: ./test_shovel.sh 25.0

SHOVEL_ANGLE=${1:-25.0}  # 默认角度 25.0 度

echo "========================================"
echo "铲斗角度控制测试"
echo "目标角度: ${SHOVEL_ANGLE} 度"
echo "========================================"

ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.0 \
    arm_angle:=0.0 \
    shovel_angle:=${SHOVEL_ANGLE} \
    steering_target:=0.0 \
    shovel_enable:=true \
    publish_rate:=10.0

