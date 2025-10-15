#!/bin/bash
# 综合控制测试脚本 - 模拟装载作业
# 用法: ./test_comprehensive.sh [速度] [大臂角度] [铲斗角度] [转向角度]
# 示例: ./test_comprehensive.sh 0.3 30.0 20.0 5.0

SPEED=${1:-0.3}            # 默认速度 0.3 m/s
ARM_ANGLE=${2:-30.0}       # 默认大臂角度 30.0 度
SHOVEL_ANGLE=${3:-20.0}    # 默认铲斗角度 20.0 度
STEERING=${4:-0.0}         # 默认转向角度 0.0 度

echo "========================================"
echo "综合控制测试 - 装载作业模拟"
echo "目标速度:     ${SPEED} m/s"
echo "大臂角度:     ${ARM_ANGLE} 度"
echo "铲斗角度:     ${SHOVEL_ANGLE} 度"
echo "转向角度:     ${STEERING} 度"
echo "========================================"

ros2 launch cannode send_control_cmd.launch.py \
    speed:=${SPEED} \
    arm_angle:=${ARM_ANGLE} \
    shovel_angle:=${SHOVEL_ANGLE} \
    steering_target:=${STEERING} \
    arm_enable:=true \
    shovel_enable:=true \
    publish_rate:=20.0

