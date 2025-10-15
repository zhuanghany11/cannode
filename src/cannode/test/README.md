# 控制指令测试节点使用说明

## 概述

`send_control_cmd` 是一个用于向 cannode 发送测试控制指令的 ROS2 节点。该节点可以发送固定的控制参数，用于测试和调试车辆控制系统。

## 功能特性

- 发送 Protobuf 格式的控制指令到 `/vehicle_command` 话题
- 支持通过 launch 参数动态配置所有控制参数
- 可调整发布频率
- 支持实时参数调整
- 提供详细的日志输出

## 控制参数

| 参数名 | 类型 | 默认值 | 单位 | 说明 |
|--------|------|--------|------|------|
| `speed` | double | 0.0 | m/s | 目标速度 |
| `arm_angle` | double | 0.0 | 度 | 大臂目标角度 |
| `shovel_angle` | double | 0.0 | 度 | 铲斗目标角度 |
| `steering_target` | double | 0.0 | 度/% | 转向目标角度 |
| `acceleration` | double | 0.0 | m/s² | 目标加速度 |
| `throttle` | double | 0.0 | % | 油门开度 (0-100) |
| `brake` | double | 0.0 | % | 制动开度 (0-100) |
| `arm_enable` | bool | true | - | 大臂控制使能 |
| `shovel_enable` | bool | true | - | 铲斗控制使能 |
| `estop` | bool | false | - | 急停信号 |
| `publish_rate` | double | 10.0 | Hz | 控制指令发布频率 |

## 使用方法

### 1. 基本使用

使用默认参数启动：
```bash
ros2 launch cannode send_control_cmd.launch.py
```

### 2. 设置单个参数

设置目标速度为 0.5 m/s：
```bash
ros2 launch cannode send_control_cmd.launch.py speed:=0.5
```

设置大臂角度为 30 度：
```bash
ros2 launch cannode send_control_cmd.launch.py arm_angle:=30.0
```

设置铲斗角度为 15 度：
```bash
ros2 launch cannode send_control_cmd.launch.py shovel_angle:=15.0
```

### 3. 组合多个参数

同时设置速度、大臂角度和铲斗角度：
```bash
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.3 \
    arm_angle:=25.0 \
    shovel_angle:=15.0
```

完整的工作场景测试：
```bash
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.5 \
    arm_angle:=30.0 \
    shovel_angle:=20.0 \
    steering_target:=10.0 \
    arm_enable:=true \
    shovel_enable:=true \
    publish_rate:=20.0
```

### 4. 调整发布频率

设置为 20 Hz 发布频率：
```bash
ros2 launch cannode send_control_cmd.launch.py publish_rate:=20.0
```

### 5. 急停测试

触发急停：
```bash
ros2 launch cannode send_control_cmd.launch.py estop:=true
```

## 典型测试场景

### 场景 1: 速度控制测试
测试车辆速度控制，以 0.3 m/s 的速度前进：
```bash
ros2 launch cannode send_control_cmd.launch.py speed:=0.3
```

### 场景 2: 大臂角度控制测试
测试大臂角度控制，目标角度为 35 度：
```bash
ros2 launch cannode send_control_cmd.launch.py arm_angle:=35.0
```

### 场景 3: 铲斗角度控制测试
测试铲斗角度控制，目标角度为 25 度：
```bash
ros2 launch cannode send_control_cmd.launch.py shovel_angle:=25.0
```

### 场景 4: 转向控制测试
测试转向控制，目标转向角度为 15 度：
```bash
ros2 launch cannode send_control_cmd.launch.py steering_target:=15.0
```

### 场景 5: 综合场景测试
模拟装载作业：前进 + 抬起大臂 + 收铲斗
```bash
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.2 \
    arm_angle:=40.0 \
    shovel_angle:=30.0 \
    steering_target:=0.0
```

### 场景 6: 高频率控制测试
测试高频率控制指令发送（50 Hz）：
```bash
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.4 \
    publish_rate:=50.0
```

## 与 cannode 配合使用

### 完整闭环测试流程

1. 启动 cannode（在第一个终端）：
```bash
ros2 launch cannode cannode.launch.py
```

2. 启动测试节点（在第二个终端）：
```bash
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.3 \
    arm_angle:=30.0
```

3. 监控底盘状态反馈（在第三个终端）：
```bash
ros2 topic echo /SaVehicleReportV2
```

4. 查看控制指令（在第四个终端）：
```bash
ros2 topic echo /vehicle_command
```

## 实时参数调整

在节点运行过程中，可以使用 `ros2 param set` 命令实时调整参数：

调整速度：
```bash
ros2 param set /send_control_cmd speed 0.6
```

调整大臂角度：
```bash
ros2 param set /send_control_cmd arm_angle 40.0
```

查看当前所有参数：
```bash
ros2 param list /send_control_cmd
```

获取特定参数的值：
```bash
ros2 param get /send_control_cmd speed
```

## 话题信息

### 发布的话题
- `/vehicle_command` (sa_msgs/msg/ProtoAdapter)
  - 包含序列化的 `control::ControlCommand` Protobuf 消息
  - 用于向 cannode 发送控制指令

### 消息格式
控制指令使用 Protobuf 格式，主要字段包括：
- `speed`: 目标速度
- `arm_angle`: 大臂角度
- `shovel_angle`: 铲斗角度
- `steering_target`: 转向目标
- `acceleration`: 加速度
- `throttle`: 油门
- `brake`: 制动
- `arm_enable`: 大臂使能
- `shovel_enable`: 铲斗使能
- `estop`: 急停

## 注意事项

1. **安全第一**：在实车上测试前，请确保已经进行了充分的仿真测试
2. **参数范围**：请确保参数值在合理范围内，避免超出车辆物理限制
3. **急停功能**：始终保持急停按钮可用，必要时可以立即停止
4. **监控反馈**：建议同时监控 `/SaVehicleReportV2` 话题以查看车辆状态反馈
5. **频率设置**：发布频率不宜过高，建议 10-50 Hz 之间
6. **角度单位**：所有角度参数的单位为度（degree）

## 调试技巧

### 查看发送的消息
```bash
ros2 topic echo /vehicle_command
```

### 查看话题发布频率
```bash
ros2 topic hz /vehicle_command
```

### 查看消息结构
```bash
ros2 topic info /vehicle_command -v
```

### 查看节点信息
```bash
ros2 node info /send_control_cmd
```

## 故障排查

### 问题：节点启动失败
- 检查是否已正确安装 sa_msgs 包
- 确认 Protobuf 库已正确链接

### 问题：cannode 没有收到消息
- 确认话题名称是否正确（`/vehicle_command`）
- 检查网络连接和 ROS2 域 ID 设置
- 使用 `ros2 topic list` 确认话题已创建

### 问题：参数不生效
- 确认参数名称拼写正确
- 检查参数类型是否匹配（double/bool）
- 查看节点日志输出的参数配置信息

## 测试建议

1. **渐进式测试**：从小参数开始，逐步增大
2. **单项测试**：先单独测试每个控制项，再进行组合测试
3. **记录数据**：使用 `ros2 bag record` 记录测试数据以便分析
4. **多次验证**：重复测试以确保控制稳定性

## 示例测试序列

完整的测试序列示例（可保存为脚本）：

```bash
#!/bin/bash

# 测试 1: 静止状态
echo "测试 1: 静止状态"
ros2 launch cannode send_control_cmd.launch.py &
sleep 5
pkill -f send_control_cmd

# 测试 2: 低速前进
echo "测试 2: 低速前进"
ros2 launch cannode send_control_cmd.launch.py speed:=0.2 &
sleep 10
pkill -f send_control_cmd

# 测试 3: 抬起大臂
echo "测试 3: 抬起大臂"
ros2 launch cannode send_control_cmd.launch.py arm_angle:=30.0 &
sleep 10
pkill -f send_control_cmd

# 测试 4: 综合动作
echo "测试 4: 综合动作"
ros2 launch cannode send_control_cmd.launch.py \
    speed:=0.3 arm_angle:=35.0 shovel_angle:=25.0 &
sleep 15
pkill -f send_control_cmd
```

## 相关文件

- 源代码: `test/send_control_cmd.cc`
- Launch 文件: `launch/send_control_cmd.launch.py`
- Protobuf 定义: `protobuf/proto/control_msgs/control_cmd.proto`
- CMakeLists 配置: `CMakeLists.txt`

## 技术支持

如有问题，请参考：
1. `IMPLEMENTATION_NOTES.md` - 实现细节说明
2. `doc/changan_protocol.dbc` - CAN 协议定义
3. ROS2 日志输出

