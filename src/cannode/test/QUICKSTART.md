# 测试节点快速开始指南

## 一分钟快速上手

### 1. 启动测试节点（默认参数）
```bash
ros2 launch cannode send_control_cmd.launch.py
```

### 2. 常用测试场景

**速度测试**（0.5 m/s）：
```bash
./test_speed.sh 0.5
```

**大臂控制**（30度）：
```bash
./test_arm.sh 30.0
```

**铲斗控制**（25度）：
```bash
./test_shovel.sh 25.0
```

**综合测试**（速度0.3m/s + 大臂30° + 铲斗20°）：
```bash
./test_comprehensive.sh 0.3 30.0 20.0 0.0
```

**急停**：
```bash
./test_estop.sh
```

## 完整闭环测试

### 终端 1 - 启动 cannode
```bash
ros2 launch cannode cannode.launch.py
```

### 终端 2 - 发送控制指令
```bash
ros2 launch cannode send_control_cmd.launch.py speed:=0.3 arm_angle:=30.0
```

### 终端 3 - 监控反馈（可选）
```bash
ros2 topic echo /SaVehicleReportV2
```

## 自定义参数

所有可用参数：
- `speed` - 目标速度 (m/s)
- `arm_angle` - 大臂角度 (度)
- `shovel_angle` - 铲斗角度 (度)
- `steering_target` - 转向角度
- `acceleration` - 加速度 (m/s²)
- `throttle` - 油门 (0-100)
- `brake` - 制动 (0-100)
- `arm_enable` - 大臂使能 (true/false)
- `shovel_enable` - 铲斗使能 (true/false)
- `estop` - 急停 (true/false)
- `publish_rate` - 发布频率 (Hz)

示例：
```bash
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.4 \
    arm_angle:=35.0 \
    shovel_angle:=25.0 \
    publish_rate:=20.0
```

## 更多信息

详细文档请参考 [README.md](README.md)

