#include "changan.h"
#include <iostream>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <linux/can.h>
#include <cmath>

namespace cannode {

ChanganCANParser::ChanganCANParser(std::string canPort)
    : ChassisCan(canPort),
      current_speed_(0.0),
      current_arm_angle_(0.0),
      current_bucket_angle_(0.0),
      current_steer_angle_(0.0),
      heartbeat_counter_(0) {
    
    checkDataStructure();
    initPIDControllers();
  initFuncMap();
}

void ChanganCANParser::initFuncMap() {
    // 注册CAN消息处理函数
    handleChassisCanFuncMap[CA_STATUS_FB_01] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0001);
    handleChassisCanFuncMap[CA_MOTOR_STATUS_02] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0002);
    handleChassisCanFuncMap[CA_WALKING_STATUS] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0003);
    handleChassisCanFuncMap[CA_DANFOSS_STEERING] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0007);

    // 倾角仪与转向角编码器（新增）
    handleChassisCanFuncMap[CA_IMU_BODY_PITCH] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x00000581);
    handleChassisCanFuncMap[CA_IMU_BOOM_PITCH] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x00000582);
    handleChassisCanFuncMap[CA_IMU_BUCKET_PITCH] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x00000583);
    handleChassisCanFuncMap[CA_STEER_ENCODER] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x18FF0015);
}

void ChanganCANParser::initPIDControllers() {
    // 初始化速度控制器 (加速度到速度)
    // 参数可以从launch文件配置，这里使用默认值
    speed_controller_.init(
        1.5,    // kp: 比例增益
        0.2,    // ki: 积分增益
        0.1,    // kd: 微分增益
        3.0,    // max_output: 最大加速度 (m/s^2)
        -3.0,   // min_output: 最小加速度 (m/s^2)
        5.0     // max_integral: 积分限幅
    );
    
    // 初始化大臂角度控制器 (阀开度到角度)
    arm_controller_.init(
        25.0,   // kp: 比例增益
        0.8,    // ki: 积分增益
        3.0,    // kd: 微分增益
        1500.0, // max_output: 最大阀电流 (mA)
        0.0,    // min_output: 最小阀电流 (mA)
        500.0,  // max_integral: 积分限幅
        0.5     // deadzone: 死区 (度)
    );
    
    // 初始化铲斗角度控制器 (阀开度到角度)
    bucket_controller_.init(
        25.0,   // kp: 比例增益
        0.8,    // ki: 积分增益
        3.0,    // kd: 微分增益
        1500.0, // max_output: 最大阀电流 (mA)
        0.0,    // min_output: 最小阀电流 (mA)
        500.0,  // max_integral: 积分限幅
        0.5     // deadzone: 死区 (度)
    );
    
    // 初始化转向角度控制器 (阀开度到角度)
    steer_controller_.init(
        30.0,   // kp: 比例增益
        1.0,    // ki: 积分增益
        2.5,    // kd: 微分增益
        1500.0, // max_output: 最大阀电流 (mA)
        0.0,    // min_output: 最小阀电流 (mA)
        500.0,  // max_integral: 积分限幅
        1.0     // deadzone: 死区 (度)
    );
    
    std::cout << "[ChanganCANParser] PID控制器初始化完成" << std::endl;
}

// ================= PID/零点外部配置接口实现 =================
void ChanganCANParser::setSpeedPid(double kp, double ki, double kd,
                                   double max_output, double min_output, double max_integral) {
    speed_controller_.init(kp, ki, kd, max_output, min_output, max_integral);
}

void ChanganCANParser::setArmPid(double kp, double ki, double kd,
                                 double max_output, double min_output, double max_integral, double deadzone) {
    arm_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
}

void ChanganCANParser::setBucketPid(double kp, double ki, double kd,
                                    double max_output, double min_output, double max_integral, double deadzone) {
    bucket_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
}

void ChanganCANParser::setSteerPid(double kp, double ki, double kd,
                                   double max_output, double min_output, double max_integral, double deadzone) {
    steer_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
}

void ChanganCANParser::setZeroOffsets(double boom_zero_deg, double bucket_zero_deg, double steer_zero_deg) {
    angle_sensors_.boom_zero_offset_deg = boom_zero_deg;
    angle_sensors_.bucket_zero_offset_deg = bucket_zero_deg;
    angle_sensors_.steer_zero_offset_deg = steer_zero_deg;
}

void ChanganCANParser::SendCmdFunc() {
    static int cnt_20ms = 0;
    static int cnt_50ms = 0;
    
    // 计算真实 dt（秒）
    double dt = 0.02; // fallback
    auto now = std::chrono::steady_clock::now();
    if (last_ctrl_time_inited_) {
        std::chrono::duration<double> d = now - last_ctrl_time_;
        dt = d.count();
        if (dt <= 0.0 || dt > 0.2) { dt = 0.02; } // 异常保护
    }
    last_ctrl_time_ = now;
    last_ctrl_time_inited_ = true;

    // 20ms发送一次控制命令
    if (++cnt_20ms >= 2) {
        cnt_20ms = 0;
        sendChassisControl();
        sendHydraulicControl(dt);
        sendWalkingControl(dt);
        sendActuatorControl(dt);
    }
}

void ChanganCANParser::sendChassisControl() {
    CA_ChassisCtrl chassisCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_CHASSIS_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 默认上高压
        chassisCtrl.high_voltage_enable = 1;
        
        // 软急停（可从control_cmd获取）
        chassisCtrl.soft_estop = ctl_cmd.estop() ? 1 : 0;
        
        // 心跳信号（至少200ms翻转一次，即每20次调用翻转一次，20ms*20=400ms实际，这里简化为计数，每次都翻转。）
        if (++heartbeat_counter_ >= 1) {  // 20ms * 10 = 200ms
            heartbeat_counter_ = 0;
            chassisCtrl.heartbeat = !chassisCtrl.heartbeat;
        } else {
            chassisCtrl.heartbeat = (heartbeat_counter_ < 5) ? 0 : 1;
        }
        
        // 整车模式 (0=有人, 1=无人)
        chassisCtrl.vehicle_mode = 1;  // 默认无人模式
        
        // 工作灯等其他控制可以根据需要添加
    }
    
    memcpy(canFrame.data, &chassisCtrl, sizeof(CA_ChassisCtrl));
    SendCanFrame(canFrame);
}

void ChanganCANParser::sendHydraulicControl(double dt) {
    CA_HydraulicCtrl hydraulicCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_HYDRAULIC_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 液压电机使能
        hydraulicCtrl.hyd_motor_enable = 1;
        
        // 液压电机工作模式: 1=转速模式, 2=扭矩模式
        hydraulicCtrl.hyd_work_mode = 1;  // 转速模式
        
        // 液压电机经济模式: 0=标准, 1=经济, 2=动力
        hydraulicCtrl.hyd_econ_mode = 0;  // 标准模式
        
        // 液压电机转速 (范围-15000~15000, 恒定物理值2000rpm，CAN发送17000)
        hydraulicCtrl.hyd_req_speed = 17000;  // 物理值2000rpm
        
        // 液压电机扭矩 (范围-3000~3000, 有3000的offset, 默认物理值为0Nm, CAN发送3000)
        hydraulicCtrl.hyd_req_torque = 3000;
        
        // 转向控制 - 使用转向角度控制器
        // steering_target 是百分比 [-100, 100]，需要转换为角度 (假设±50度)
        double target_steer_percentage = ctl_cmd.steering_target();
        double target_steer_angle = target_steer_percentage * 0.5;  // 转换为角度 (-50~50度)
        // dt 由 SendCmdFunc 实时计算传入
        
        // 计算转向阀电流 (通过PID控制器)
        double steer_valve_current = steer_controller_.compute(target_steer_angle, current_steer_angle_, dt);
        // 取绝对值并限幅到 [0, 500] (上限是1500 mA）
        double steer_valve_current_abs = std::fabs(steer_valve_current);
        if (steer_valve_current_abs > 500.0) steer_valve_current_abs = 500.0;
        
        // 确定转向方向
        double angle_error = target_steer_angle - current_steer_angle_;
        if (angle_error > 1.0) {
            hydraulicCtrl.steer_valve_dir = 2;  // 右转
        } else if (angle_error < -1.0) {
            hydraulicCtrl.steer_valve_dir = 1;  // 左转
        } else {
            hydraulicCtrl.steer_valve_dir = 0;  // 不转向
        }
        
        // 设置转向阀电流（幅值）
        hydraulicCtrl.steer_valve_current = static_cast<uint16_t>(steer_valve_current_abs);
    }
    
    memcpy(canFrame.data, &hydraulicCtrl, sizeof(CA_HydraulicCtrl));
    SendCanFrame(canFrame);
}

void ChanganCANParser::sendWalkingControl(double dt) {
    CA_WalkingCtrl walkingCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_WALKING_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 行走电机使能
        walkingCtrl.drive_motor_enable = 1;
        
        // 行走电机工作模式:  1=转速模式, 2=扭矩模式（按头文件注释统一）
        walkingCtrl.drive_work_mode = 2;  // 扭矩模式，后续按需求切换
        
        // 行走电机经济模式
        walkingCtrl.drive_econ_mode = 0;  // 标准模式
        
        // 从control_cmd获取目标速度
        double target_speed = ctl_cmd.speed();  // m/s
        // dt 由 SendCmdFunc 实时计算传入
        
        // 使用速度控制器计算所需加速度
        double acceleration = speed_controller_.compute(target_speed, current_speed_, dt);
        
        // 根据加速度计算扭矩（简化模型：扭矩正比于加速度）
        // 加速度范围 [0, 2] m/s^2 -> 扭矩绝对值 [0, 200] Nm （最大值3000,但需要限幅）
        int32_t torque_cmd = static_cast<int32_t>(std::round(std::abs(acceleration) * 100.0));
        if (torque_cmd > 200) torque_cmd = 200;
        // 数据下发增加一个3000的offset
        walkingCtrl.drive_req_torque = static_cast<uint16_t>(torque_cmd+3000);
        
        // 设置行走电机转速（范围-15000~15000），无作用，加速度由扭矩控制。
        walkingCtrl.drive_req_speed = static_cast<uint16_t>(15000);
        
        // 确定行走方向
        if (target_speed > 0.1) {
            walkingCtrl.drive_direction = 1;  // 前进
        } else if (target_speed < -0.1) {
            walkingCtrl.drive_direction = 2;  // 后退
        } else {
            walkingCtrl.drive_direction = 0;  // 空档
        }
        
        // 刹车控制（如果有驻车制动请求）
        // TODO： 目前没有通过制动进行车辆控制的功能。
        if (ctl_cmd.parking_brake()) {
            walkingCtrl.brake_valve_current = 1200;  // 刹车电流 (400-1600mA)
        } else {
            walkingCtrl.brake_valve_current = 400;   // 最小刹车电流
        }
    }
    
    memcpy(canFrame.data, &walkingCtrl, sizeof(CA_WalkingCtrl));
    SendCanFrame(canFrame);
}

void ChanganCANParser::sendActuatorControl(double dt) {
    CA_ActuatorCtrl actuatorCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_ACTUATOR_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // dt 由 SendCmdFunc 实时计算传入
        
        // 大臂控制 - 使用角度控制器
        double target_arm_angle = ctl_cmd.arm_angle();  // 从control_cmd获取目标大臂角度
        double arm_valve_current = arm_controller_.compute(target_arm_angle, current_arm_angle_, dt);
        double arm_valve_current_abs = std::fabs(arm_valve_current);
        if (arm_valve_current_abs > 500.0) arm_valve_current_abs = 500.0;
        
        // 根据误差方向确定大臂动作
        double arm_error = target_arm_angle - current_arm_angle_;
        if (arm_error > 0.5) {  // 需要抬起
            actuatorCtrl.arm_up_current = static_cast<uint16_t>(arm_valve_current_abs);
            actuatorCtrl.arm_down_current = 0;
        } else if (arm_error < -0.5) {  // 需要下降
            actuatorCtrl.arm_up_current = 0;
            actuatorCtrl.arm_down_current = static_cast<uint16_t>(arm_valve_current_abs);
        } else {  // 在死区内，不动作
            actuatorCtrl.arm_up_current = 0;
            actuatorCtrl.arm_down_current = 0;
        }
        
        // 铲斗控制 - 使用角度控制器
        double target_bucket_angle = ctl_cmd.shovel_angle();  // 从control_cmd获取目标铲斗角度
        double bucket_valve_current = bucket_controller_.compute(target_bucket_angle, current_bucket_angle_, dt);
        double bucket_valve_current_abs = std::fabs(bucket_valve_current);
        if (bucket_valve_current_abs > 500.0) bucket_valve_current_abs = 500.0;
        
        // 根据误差方向确定铲斗动作
        double bucket_error = target_bucket_angle - current_bucket_angle_;
        if (bucket_error > 0.5) {  // 需要外翻
            actuatorCtrl.bucket_out_current = static_cast<uint16_t>(bucket_valve_current_abs);
            actuatorCtrl.bucket_in_current = 0;
        } else if (bucket_error < -0.5) {  // 需要内收
            actuatorCtrl.bucket_out_current = 0;
            actuatorCtrl.bucket_in_current = static_cast<uint16_t>(bucket_valve_current_abs);
        } else {  // 在死区内，不动作
            actuatorCtrl.bucket_out_current = 0;
            actuatorCtrl.bucket_in_current = 0;
        }
    }
    
    memcpy(canFrame.data, &actuatorCtrl, sizeof(CA_ActuatorCtrl));
    SendCanFrame(canFrame);
}

// ================================
// 接收消息处理函数
// ================================

void ChanganCANParser::handle0x181F0001(struct Canframe *recvCanFrame) {
    // 状态反馈消息
    if (recvCanFrame->frame.can_dlc != 8) {
        std::cerr << "Unexpected CAN frame DLC for 0x181F0001: " 
                  << static_cast<int>(recvCanFrame->frame.can_dlc) << std::endl;
        return;
    }
    
    VCU_heartbeat_cnt = 0;  // 重置VCU心跳计数
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新驾驶模式
        if (canFrameUnion.status_fb_01.vehicle_mode_sts) {
            chassisProtoMsg.set_driving_mode(control::canbus::Chassis::COMPLETE_AUTO_DRIVE);
        } else {
            chassisProtoMsg.set_driving_mode(control::canbus::Chassis::COMPLETE_MANUAL);
        }
        
        // 更新电池电量 (分辨率0.4)
        double battery_soc = canFrameUnion.status_fb_01.battery_soc * 0.4;
        // chassisProtoMsg.set_battery_soc(battery_soc);  // 如果protobuf定义了此字段
        
        // 更新故障信息
        // 可以根据需要处理各种故障位
    }
}

void ChanganCANParser::handle0x181F0002(struct Canframe *recvCanFrame) {
    // 电机状态消息
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 液压电机状态
        // 可以记录液压电机的实际转速、扭矩等信息
        
        // 更新档位信息
        uint8_t gear = canFrameUnion.motor_status_02.drive_direction;
        control::canbus::Chassis::GearPosition gear_pos;
        switch(gear) {
            case 0:
                gear_pos = control::canbus::Chassis::GEAR_NEUTRAL;
                break;
            case 1:
                gear_pos = control::canbus::Chassis::GEAR_DRIVE;
                break;
            case 2:
                gear_pos = control::canbus::Chassis::GEAR_REVERSE;
                break;
            default:
                gear_pos = control::canbus::Chassis::GEAR_NEUTRAL;
                break;
        }
        chassisProtoMsg.set_gear_location(gear_pos);
    }
}

void ChanganCANParser::handle0x181F0003(struct Canframe *recvCanFrame) {
    // 行走电机状态消息
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新车速 (从轮速计算，分辨率0.16, 单位可能是km/h，需要转换为m/s)
        double wheel_speed_kmh = canFrameUnion.walking_status.wheel_speed * 0.16;
        current_speed_ = wheel_speed_kmh / 3.6;  // 转换为m/s
        chassisProtoMsg.set_speed_mps(current_speed_);
        
        // 更新行走电机扭矩等信息（如果需要）
    }
}

void ChanganCANParser::handle0x181F0007(struct Canframe *recvCanFrame) {
    // 丹佛斯转向消息
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 若未收到编码器，则可用丹佛斯角作为备用（映射 -50~+50 度）功能没有启用。
        // if (!steer_encoder_available_) {
        //     double raw_angle = canFrameUnion.danfoss_steering.danfoss_angle;
        //     current_steer_angle_ = (raw_angle / 4095.0) * 100.0 - 50.0;
        //     // 存入百分比：角度 / 50deg * 100
        //     double steering_percentage = (current_steer_angle_ / 50.0) * 100.0;
        //     chassisProtoMsg.set_steering_percentage(static_cast<float>(steering_percentage));
        // }
    }
}

// ================================
// 新增：倾角仪与角度编码器解析
// ================================

// 尝试从8字节数据解析Y轴俯仰角（度）
// 1) 优先尝试 float32 小端在 byte[4..7]
// 2) 其次尝试 int16 小端缩放 0.01 在 byte[4..5]
// 3) 再次尝试 int16 大端缩放 0.01 在 byte[4..5]
// 4) 兜底返回0.0
static double parseYPitchDegFromSenseFrame(const uint8_t data[8]) {
    // 尝试 float32 小端 (bytes 4..7)
    union { float f; uint32_t u; } conv = {0};
    conv.u = static_cast<uint32_t>(data[4]) |
             (static_cast<uint32_t>(data[5]) << 8) |
             (static_cast<uint32_t>(data[6]) << 16) |
             (static_cast<uint32_t>(data[7]) << 24);
    if (std::isfinite(conv.f) && std::fabs(conv.f) <= 180.0) {
        return static_cast<double>(conv.f);
    }

    // 尝试 int16 小端缩放0.01 (bytes 4..5)
    int16_t v_le = static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                (static_cast<uint16_t>(data[5]) << 8));
    double deg_le = static_cast<double>(v_le) * 0.01;
    if (std::fabs(deg_le) <= 180.0) {
        return deg_le;
    }

    // 尝试 int16 大端缩放0.01 (bytes 4..5)
    int16_t v_be = static_cast<int16_t>(static_cast<uint16_t>(data[5]) |
                (static_cast<uint16_t>(data[4]) << 8));
    double deg_be = static_cast<double>(v_be) * 0.01;
    if (std::fabs(deg_be) <= 180.0) {
        return deg_be;
    }

    return 0.0;
}

void ChanganCANParser::updateRelativeAnglesAndChassis() {
    // 计算相对角，加入零点标定
    angle_sensors_.boom_rel_body_deg =
        (angle_sensors_.boom_pitch_y_deg - angle_sensors_.body_pitch_y_deg) +
        angle_sensors_.boom_zero_offset_deg;

    angle_sensors_.bucket_rel_boom_deg =
        (angle_sensors_.bucket_pitch_y_deg - angle_sensors_.boom_pitch_y_deg) +
        angle_sensors_.bucket_zero_offset_deg;

    // 更新内部当前角度
    current_arm_angle_ = angle_sensors_.boom_rel_body_deg;
    current_bucket_angle_ = angle_sensors_.bucket_rel_boom_deg;

    // 将角度写入 chassisProtoMsg
    chassisProtoMsg.set_arm_angle(static_cast<float>(current_arm_angle_));
    chassisProtoMsg.set_shovel_angle(static_cast<float>(current_bucket_angle_));

    // 转向角（若编码器有效，带零点）
    if (steer_encoder_available_) {
        current_steer_angle_ = angle_sensors_.steering_angle_deg + angle_sensors_.steer_zero_offset_deg;
        double steering_percentage = (current_steer_angle_ / 50.0) * 100.0;
        chassisProtoMsg.set_steering_percentage(static_cast<float>(steering_percentage));
    }
}

void ChanganCANParser::handle0x00000581(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        angle_sensors_.body_pitch_y_deg = parseYPitchDegFromSenseFrame(recvCanFrame->frame.data);
        updateRelativeAnglesAndChassis();
    }
}

void ChanganCANParser::handle0x00000582(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        angle_sensors_.boom_pitch_y_deg = parseYPitchDegFromSenseFrame(recvCanFrame->frame.data);
        updateRelativeAnglesAndChassis();
    }
}

void ChanganCANParser::handle0x00000583(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        angle_sensors_.bucket_pitch_y_deg = parseYPitchDegFromSenseFrame(recvCanFrame->frame.data);
        updateRelativeAnglesAndChassis();
    }
}

void ChanganCANParser::handle0x18FF0015(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        // byte[1..2] 拼接为角度*100（假定大端字节序），单位：度
        uint16_t raw = static_cast<uint16_t>((recvCanFrame->frame.data[1] << 8) |
                                             recvCanFrame->frame.data[2]);
        double deg = static_cast<double>(raw) / 100.0;
        angle_sensors_.steering_angle_deg = deg;
        steer_encoder_available_ = true;
        updateRelativeAnglesAndChassis();
    }
}

void ChanganCANParser::checkDataStructure() {
    // 确保结构体大小正确
    static_assert(sizeof(CA_ChassisCtrl) == 8, "Invalid CA_ChassisCtrl size");
    static_assert(sizeof(CA_HydraulicCtrl) == 8, "Invalid CA_HydraulicCtrl size");
    static_assert(sizeof(CA_WalkingCtrl) == 8, "Invalid CA_WalkingCtrl size");
    static_assert(sizeof(CA_ActuatorCtrl) == 8, "Invalid CA_ActuatorCtrl size");
    static_assert(sizeof(CA_StatusFB01) == 8, "Invalid CA_StatusFB01 size");
    static_assert(sizeof(CA_MotorStatus02) == 8, "Invalid CA_MotorStatus02 size");
    static_assert(sizeof(CA_WalkingStatus) == 8, "Invalid CA_WalkingStatus size");
    static_assert(sizeof(CA_DanfossSteering) == 8, "Invalid CA_DanfossSteering size");
    
    std::cout << "[ChanganCANParser] 数据结构检查通过，所有结构体大小为8字节" << std::endl;
}

} // namespace cannode
