## cannode 源码概览（仅 src/cannode/src/）

本说明文档聚焦 `src/cannode/src/` 目录的代码结构、主要模块、关键函数与调用关系，以及整车控制闭环的数据流与 CAN 报文映射。

### 目录结构

- `vehicle_protocol/`
  - `changan.h/.cc`：长安协议的核心实现（本项目主要修改点）。
  - `shantui.h/.cc`：参考范本（不修改）。
  - `xugong.h/.cc`：可忽略。
- `low_level_controller/`
  - `acc_to_speed_controller.h/.cc`：加速度→速度 PID 控制器。
  - `valve_to_angle_controller.h/.cc`：阀开度→角度 PID 控制器（用于大臂、铲斗、转向）。
- `chassis_driver.h/.cc`：底盘 CAN 抽象基类，封装发送/接收线程与分发机制。
- `socket_can_interface.h/.cc`：SocketCAN 底层封装。
- `msg_manager.h/.cc`：ROS2 桥接节点，订阅控制指令、发布底盘状态。

---

## 运行时数据流（高层）

1) ROS2 输入
- 订阅话题：`/vehicle_command`（`sa_msgs::msg::ProtoAdapter` 封装 `control::ControlCommand`）。
- 处理函数：`MsgManager::procControlImplement()` 反序列化后，调用 `ChassisCan::setCtlCmd()` 注入控制指令。

2) 发送线程（控制输出）
- 创建处：`ChassisCan::Initialize()` 内部启动线程，周期调用 `ChassisCan::SendCmdProc()`。
- 调度：`SendCmdProc()` 递增心跳计数，调用派生类 `ChanganCANParser::SendCmdFunc()`。
- 周期控制：`ChanganCANParser::SendCmdFunc()` 使用 `steady_clock` 动态计算 `dt`，并在 20ms 周期下发 4 类报文：
  - `sendChassisControl()` → 0x18000001 底盘控制（上高压/心跳/模式）。
  - `sendHydraulicControl(dt)` → 0x18000002 液压/转向控制（含转向阀 PID）。
  - `sendWalkingControl(dt)` → 0x18000003 行走控制（速度闭环→扭矩/速度请求与方向）。
  - `sendActuatorControl(dt)` → 0x18000004 机构阀控制（大臂/铲斗角度闭环择向）。

3) 接收线程（反馈输入）
- 创建处：`ChassisCan::Initialize()` 内部启动线程，循环调用 `ChassisCan::StartRecv()`。
- 分发：`ChassisCan::HandleRecvData()` 掩掉扩展帧标志后，`callHandleWireCanCmdFunc()` 基于 `initFuncMap()` 注册表分发到对应 handler。
- 长安协议反馈：
  - 0x181F0001 `handle0x181F0001()`：更新驾驶模式、电量/故障位。
  - 0x181F0002 `handle0x181F0002()`：更新档位/液压电机状态。
  - 0x181F0003 `handle0x181F0003()`：轮速→车速反馈（km/h→m/s）。
  - 0x181F0007 `handle0x181F0007()`：丹佛斯转向角（编码器缺失兜底可选）。
  - 倾角/编码器：
    - 0x00000581 车体：`handle0x00000581()`
    - 0x00000582 大臂：`handle0x00000582()`
    - 0x00000583 铲斗：`handle0x00000583()`
    - 0x18FF0015 转向编码器：`handle0x18FF0015()`（byte1..2 拼接/100 → 度）
  - `updateRelativeAnglesAndChassis()`：将绝对倾角差分+零点标定，得到大臂相对车体、铲斗相对大臂角度；并写入 `chassisProtoMsg`。

4) ROS2 输出
- 发布话题：`/SaVehicleReportV2`（`sa_msgs::msg::ProtoAdapter` 封装 `control::canbus::Chassis`）。
- 发布处：`MsgManager::PublishChassisStatus()` 从协议解析器获取 `chassisProtoMsg` 序列化后发布。

---

## 关键模块详解

### vehicle_protocol/changan.*（核心）

- 类：`ChanganCANParser : public ChassisCan`
  - 初始化：
    - `initFuncMap()`：注册 CAN ID→处理函数映射。
    - `initPIDControllers()`：初始化 4 个 PID 控制器（可外部覆盖）。
  - 周期发送：
    - `SendCmdFunc()`：计算 `dt`，按 20ms 发送 4 类报文。
    - `sendChassisControl()`：0x18000001，上高压/心跳/模式等。
    - `sendHydraulicControl(double dt)`：0x18000002；将 `steering_target`（百分比）映射为角度（±50°），与 `current_steer_angle_` 做 PID，输出转向阀电流（幅值限幅，方向由误差符号给出）。
    - `sendWalkingControl(double dt)`：0x18000003；`AccToSpeedController` 根据目标/当前速度输出加速度，映射为扭矩幅值并限幅，同时设置方向与请求转速（带 15000 偏移）。
    - `sendActuatorControl(double dt)`：0x18000004；大臂/铲斗角度闭环，`ValveToAngleController` 输出阀电流幅值，按误差方向选择抬/降、内收/外翻通道。
  - 接收处理：
    - `handle0x181F0001/2/3/7()`：VCU 状态/行走/丹佛斯转向。
    - `handle0x00000581/82/83()`：倾角仪；`parseYPitchDegFromSenseFrame()` 兼容多种数据布局。
    - `handle0x18FF0015()`：转向角编码器；`byte[1..2]/100.0` 度。
    - `updateRelativeAnglesAndChassis()`：计算相对角并写入 `chassisProtoMsg`（`arm_angle`、`shovel_angle`、`steering_percentage`）。
  - 外部配置接口：
    - `setSpeedPid(...)`、`setArmPid(...)`、`setBucketPid(...)`、`setSteerPid(...)`：动态更新 PID 参数。
    - `setZeroOffsets(boom, bucket, steer)`：设置大臂/铲斗/转向零点偏置。

### low_level_controller/*（闭环控制器）

- `AccToSpeedController`
  - 输入：目标速度/当前速度、`dt`。
  - 输出：加速度（m/s²）。
  - 特性：PID（带积分限幅、输出限幅、微分防零除）。

- `ValveToAngleController`
  - 输入：目标角度/当前角度、`dt`。
  - 输出：阀开度/电流（非负，带限幅）。
  - 特性：误差死区、积分限幅、微分防零除；适用于大臂、铲斗、转向。

### chassis_driver.*（底盘 CAN 基类）

- 线程模型：
  - `Initialize()`：启动发送线程（周期调用 `SendCmdProc()`）与接收线程（循环 `StartRecv()`）。
  - `SendCmdProc()`：维护心跳计数、调用派生类 `SendCmdFunc()`。
  - `StartRecv()`：读取 CAN 帧，时间戳记录后调用 `HandleRecvData()`。
- 分发机制：
  - `HandleRecvData()`：掩掉 EFF 标志，调用 `callHandleWireCanCmdFunc()`。
  - `callHandleWireCanCmdFunc()`：基于 `handleChassisCanFuncMap` 分发至具体 handler。
- 数据接口：
  - `setCtlCmd(const ControlCommand&)`：注入控制指令。
  - `get_chassis_status_msg()`：加锁返回底盘状态 protobuf。

### socket_can_interface.*（SocketCAN 封装）

- 初始化/绑定：`CanInit()` 打开 RAW 套接字、设置 CAN_FD/时间戳、bind 到接口。
- 发送：`SendCanFrame()` 自动根据 ID 设置扩展帧标志（>0x7FF），支持单帧/多帧。
- 接收：`ReadCan()` 返回 `can_frame` 与接收时间戳。

### msg_manager.*（ROS2 桥接）

- 订阅：`/vehicle_command`（`ProtoAdapter` 封装 `ControlCommand`）→ `procControlImplement()` 反序列化后注入解析器。
- 发布：`/SaVehicleReportV2`（`ProtoAdapter` 封装 `Chassis`）→ `PublishChassisStatus()` 获取并发布底盘状态。
- 实例化：`cannodeCreateInstance()` 依据 `vcu_type` 选择协议解析器，默认长安；随后 `Initialize()` 启动线程。
- CAN 设备配置：`cannodeCfg()` 调用系统命令设置 bitrate 与开启接口。

---

## CAN 报文与量纲要点（长安）

- 上位机→VCU（发送）
  - 0x18000001：底盘控制。
  - 0x18000002：液压/转向；转向阀电流幅值（限幅）、方向位依据误差符号；液压电机模式/经济模式占位。
  - 0x18000003：行走；速度闭环输出加速度→扭矩幅值，方向位分离；请求转速加 `+15000` 偏移。
  - 0x18000004：机构电磁阀；大臂/铲斗角度闭环输出电流，误差方向决定抬/降、内收/外翻。

- VCU→上位机（接收）
  - 0x181F0001/2/3/7：模式/电机/行走/丹佛斯状态。
  - 0x00000581/582/583：车体/大臂/铲斗倾角（Y_pitch）。
  - 0x18FF0015：转向角编码器（byte1..2 拼接后 /100 → 度）。

- 量纲/约定
  - 转向目标：`steering_target` 百分比 → 角度（±50°）。
  - 底盘状态：`steering_percentage = steer_angle_deg / 50 * 100`（写入 `chassis.pb`）。
  - 大臂/铲斗角度：由绝对倾角差分 + 零点偏置得到相对角。
  - dt：使用 `steady_clock` 实时计算（异常保护回退 0.02s）。

---

## 参数与可调项

- PID 参数默认值在 `initPIDControllers()` 中配置，可通过以下接口动态覆盖：
  - `setSpeedPid(...)`、`setArmPid(...)`、`setBucketPid(...)`、`setSteerPid(...)`。
- 零点标定：`setZeroOffsets(boom_zero_deg, bucket_zero_deg, steer_zero_deg)`。
- 如需在运行时通过 ROS2 参数注入，可在 `MsgManager` 中获取参数后调用上述接口（已预留接口与示例注释）。

---

## 扩展与排错建议

- 新增 CAN 解析：在 `initFuncMap()` 注册 ID→handler，并在 `.cc` 中实现对应 `handle0xXXXXXXXX()`。
- 校验 DBC 对齐：`checkDataStructure()` 静态断言各结构体均为 8 字节。
- 方向/幅值：阀电流与扭矩统一采用“幅值 + 方向位/通道”模式，避免无符号字段负数截断。
- 观察闭环：关注 `chassisProtoMsg.speed_mps / arm_angle / shovel_angle / steering_percentage` 与目标指令的一致性。


