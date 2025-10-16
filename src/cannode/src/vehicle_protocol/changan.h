#pragma once

#include <cstdint>
#include <linux/can.h>
#include <chrono>
#include "../chassis_driver.h"
#include "../low_level_controller/acc_to_speed_controller.h"
#include "../low_level_controller/valve_to_angle_controller.h"

namespace cannode {

// CAN消息ID定义（DBC中的ID需要减去0x80000000）
enum ChanganCanProtoId {
    // 上位机→VCU 控制命令
    CA_CHASSIS_CTRL     = 0x18000001,  // 控制命令消息
    CA_HYDRAULIC_CTRL   = 0x18000002,  // 液压电机控制消息
    CA_WALKING_CTRL     = 0x18000003,  // 行走电机控制消息
    CA_ACTUATOR_CTRL    = 0x18000004,  // 机构电磁阀控制消息
    
    // VCU→上位机 状态反馈
    CA_STATUS_FB_01     = 0x181F0001,  // 状态反馈消息1
    CA_MOTOR_STATUS_02  = 0x181F0002,  // 电机状态消息2
    CA_WALKING_STATUS   = 0x181F0003,  // 行走电机状态
    CA_POWER_INFO       = 0x181F0004,  // 功率信息
    CA_VALVE_CURRENT_FB = 0x181F0005,  // 阀电流反馈
    CA_JOINT_PRESSURE   = 0x181F0006,  // 关节油压
    CA_DANFOSS_STEERING = 0x181F0007   // 丹佛斯转向
};

// 角度传感器与倾角仪 CAN 消息ID（参考 Read_angle_sensors/doc/Sense_DBC.dbc）
// 注：以下ID为实际在总线使用的ID（若DBC给出ID需-0x80000000，则此处已为减法后的结果）
enum ChanganAngleSensorId {
    CA_IMU_BODY_PITCH   = 0x00000581,  // 车体倾角仪（使用Y_pitch）
    CA_IMU_BOOM_PITCH   = 0x00000582,  // 大臂倾角仪（使用Y_pitch）
    CA_IMU_BUCKET_PITCH = 0x00000583,  // 铲斗倾角仪（使用Y_pitch）
    CA_STEER_ENCODER    = 0x18FF0015   // 转向角度编码器（byte[1..2] / 100.0 -> 度）
};

#pragma pack(push, 1)

// ================================
// 上位机→VCU 控制命令定义
// ================================

// 控制命令消息 (0x18000001)
typedef struct ChanganChassisControl {
    // byte 0
    uint8_t high_voltage_enable : 1;      // bit 0: 整车上高压
    uint8_t reserved1 : 1;                // bit 1: 预留
    uint8_t soft_estop : 1;               // bit 2: 软急停
    uint8_t left_turn_light : 1;          // bit 3: 左转灯
    uint8_t right_turn_light : 1;         // bit 4: 右转灯
    uint8_t reserved2 : 2;                // bit 5-6: 预留
    uint8_t vehicle_mode : 1;             // bit 7: 整车模式
    
    // byte 1
    uint8_t reserved3 : 1;                // bit 8: 预留
    uint8_t heartbeat : 1;                // bit 9: 心跳信号
    uint8_t speed_gear : 1;               // bit 10: 速度挡位
    uint8_t work_light : 1;               // bit 11: 工作大灯
    uint8_t clear_steer_error : 1;        // bit 12: 清除转向阀错误
    uint8_t high_beam : 1;                // bit 13: 远光灯
    uint8_t low_beam : 1;                 // bit 14: 近光灯
    uint8_t horn : 1;                     // bit 15: 喇叭
    
    // byte 2
    uint8_t alarm_lights : 5;             // bit 16-20: 声光报警器
    uint8_t clear_error : 1;              // bit 21: 清错
    uint8_t reserved4 : 2;                // bit 22-23: 预留
    
    // byte 3-7 (40位预留)
    uint8_t reserved5[5];                 // 预留
} CA_ChassisCtrl;

// 液压电机控制消息 (0x18000002)
typedef struct ChanganHydraulicControl {
    // byte 0
    uint8_t hyd_motor_enable : 1;         // bit 0: 液压电机使能
    uint8_t hyd_work_mode : 2;            // bit 1-2: 工作模式
    uint8_t hyd_econ_mode : 2;            // bit 3-4: 经济模式
    uint8_t steer_valve_dir : 2;          // bit 5-6: 转向电磁阀方向
    uint8_t reserved1 : 1;                // bit 7: 预留
    
    // byte 1-2 (16位液压电机请求扭矩)
    uint16_t hyd_req_torque;              // 液压电机请求扭矩
    
    // byte 3-4 (16位液压电机请求转速)
    uint16_t hyd_req_speed;               // 液压电机请求转速
    
    // byte 5-6 (16位转向电磁阀电流)
    uint16_t steer_valve_current;         // 转向电磁阀电流 (0-1500mA)
    
    // byte 7
    uint8_t reserved2;                    // 预留
} CA_HydraulicCtrl;

// 行走电机控制消息 (0x18000003)
typedef struct ChanganWalkingControl {
    // byte 0
    uint8_t drive_motor_enable : 1;       // bit 0: 行走电机使能
    uint8_t drive_work_mode : 2;          // bit 1-2: 工作模式
    uint8_t drive_direction : 2;          // bit 3-4: 行走方向 (0=空档,1=前进,2=后退)
    uint8_t drive_econ_mode : 2;          // bit 5-6: 经济模式
    uint8_t reserved1 : 1;                // bit 7: 预留
    
    // byte 1-2 (16位行走电机请求转速)
    uint16_t drive_req_speed;             // 行走电机请求转速
    
    // byte 3-4 (16位行走电机请求扭矩)
    uint16_t drive_req_torque;            // 行走电机请求扭矩
    
    // byte 5-6 (16位刹车电磁阀控制)
    uint16_t brake_valve_current;         // 刹车电磁阀控制 (400-1600mA)
    
    // byte 7
    uint8_t reserved2;                    // 预留
} CA_WalkingCtrl;

// 机构电磁阀控制消息 (0x18000004)
typedef struct ChanganActuatorControl {
    // byte 0-1 (16位大臂抬电磁阀)
    uint16_t arm_up_current;              // 大臂抬电磁阀电流 (0-1500mA)
    
    // byte 2-3 (16位大臂降电磁阀)
    uint16_t arm_down_current;            // 大臂降电磁阀电流 (0-1500mA)
    
    // byte 4-5 (16位铲斗内收电磁阀)
    uint16_t bucket_in_current;           // 铲斗内收电磁阀电流 (0-1500mA)
    
    // byte 6-7 (16位铲斗外翻电磁阀)
    uint16_t bucket_out_current;          // 铲斗外翻电磁阀电流 (0-1500mA)
} CA_ActuatorCtrl;

// ================================
// VCU→上位机 信息定义
// ================================

// 状态反馈消息1 (0x181F0001)
typedef struct ChanganStatusFeedback01 {
    // byte 0
    uint8_t high_voltage_sts : 1;         // bit 0: 上高压状态
    uint8_t parking_brake : 1;            // bit 1: 停车制动
    uint8_t soft_estop_sts : 1;           // bit 2: 软急停状态
    uint8_t left_turn_sts : 1;            // bit 3: 左转向灯
    uint8_t right_turn_sts : 1;           // bit 4: 右转向灯
    uint8_t vehicle_mode_sts : 1;         // bit 5: 整车模式
    uint8_t external_estop : 1;           // bit 6: 外部急停状态
    uint8_t reserved1 : 1;                // bit 7: 预留
    
    // byte 1
    uint8_t heartbeat_sts : 1;            // bit 8: 心跳信号
    uint8_t heartbeat_error : 1;          // bit 9: 上位机心跳状态
    uint8_t alarm_lights_sts : 5;         // bit 10-14: 声光报警器
    uint8_t charging_sts : 1;             // bit 15: 充电状态
    
    // byte 2-3 (16位刹车控制电流)
    uint16_t brake_current;               // 刹车控制电流
    
    // byte 4 (8位设备电量)
    uint8_t battery_soc;                  // 设备电量 (分辨率0.4)
    
    // byte 5 (故障位)
    uint8_t arm_up_error : 1;             // bit 40: 大臂抬电磁阀故障
    uint8_t arm_down_error : 1;           // bit 41: 大臂降电磁阀故障
    uint8_t bucket_in_error : 1;          // bit 42: 铲斗内收电磁阀故障
    uint8_t bucket_out_error : 1;         // bit 43: 铲斗外翻电磁阀故障
    uint8_t brake_valve_error : 1;        // bit 44: 脚制动电磁阀故障
    uint8_t steer_valve_error : 1;        // bit 45: 转向电磁阀故障
    uint8_t drive_com_error : 1;          // bit 46: 行走电机通讯故障
    uint8_t hyd_com_error : 1;            // bit 47: 液压电机通讯故障
    
    // byte 6
    uint8_t fault_level : 2;              // bit 48-49: 本体故障等级
    uint8_t low_beam_sts : 1;             // bit 50: 近光灯状态
    uint8_t high_beam_sts : 1;            // bit 51: 远光灯状态
    uint8_t work_light_sts : 1;           // bit 52: 工作灯状态
    uint8_t horn_feedback : 1;            // bit 53: 喇叭反馈
    uint8_t speed_gear_fb : 1;            // bit 54: 速度档反馈
    uint8_t steer_valve_estop : 1;        // bit 55: 转向阀故障急停
    
    // byte 7
    uint8_t arm_up_fb : 1;                // bit 56: 大臂抬动作反馈
    uint8_t arm_down_fb : 1;              // bit 57: 大臂降动作反馈
    uint8_t bucket_in_fb : 1;             // bit 58: 铲斗内收动作反馈
    uint8_t bucket_out_fb : 1;            // bit 59: 铲斗外翻动作反馈
    uint8_t left_valve_fb : 1;            // bit 60: 左转电磁阀反馈
    uint8_t right_valve_fb : 1;           // bit 61: 右转电磁阀反馈
    uint8_t reserved2 : 2;                // bit 62-63: 预留
} CA_StatusFB01;

// 电机状态消息2 (0x181F0002)
typedef struct ChanganMotorStatus02 {
    // byte 0-1 (16位液压电机实际转速)
    uint16_t hyd_actual_speed;            // 液压电机实际转速
    
    // byte 2-3 (16位液压电机实际扭矩)
    uint16_t hyd_actual_torque;           // 液压电机实际扭矩
    
    // byte 4-5 (16位液压电机电流)
    uint16_t hyd_current;                 // 液压电机电流
    
    // byte 6
    uint8_t hyd_motor_en : 1;             // bit 48: 液压电机使能信号
    uint8_t reserved1 : 2;                // bit 49-50: 预留
    uint8_t hyd_work_mode : 2;            // bit 51-52: 液压电机工作模式
    uint8_t reserved2 : 3;                // bit 53-55: 预留
    
    // byte 7
    uint8_t drive_motor_en : 1;           // bit 56: 行走电机使能信号
    uint8_t drive_direction : 2;          // bit 57-58: 行进电机挡位信息
    uint8_t drive_work_mode : 2;          // bit 59-60: 行走电机工作模式
    uint8_t drive_econ_mode : 2;          // bit 61-62: 行走电机经济模式
    uint8_t reserved3 : 1;                // bit 63: 预留
} CA_MotorStatus02;

// 行走电机状态 (0x181F0003)
typedef struct ChanganWalkingStatus {
    // byte 0-1 (16位行走电机电流)
    uint16_t drive_current;               // 行走电机电流
    
    // byte 2-3 (16位行走电机实际扭矩)
    uint16_t drive_actual_torque;         // 行走电机实际扭矩
    
    // byte 4-5 (16位行走电机转速/车速)
    uint16_t drive_speed;                 // 行走电机转速/车速
    
    // byte 6-7 (16位轮速)
    uint16_t wheel_speed;                 // 轮速 (分辨率0.16, 范围0-60)
} CA_WalkingStatus;

// 丹佛斯转向 (0x181F0007)
typedef struct ChanganDanfossSteering {
    // byte 0-1 (16位丹佛斯方向机角度)
    uint16_t danfoss_angle;               // 丹佛斯方向机角度 (0-4095)
    
    // byte 2 (8位丹佛斯状态)
    uint8_t danfoss_status;               // 丹佛斯状态 (0x00=正常,0x11=初始化,0xFF=故障)
    
    // byte 3-7 (40位预留)
    uint8_t reserved[5];                  // 预留
} CA_DanfossSteering;

#pragma pack(pop)

// CAN消息联合体
union ChanganCanFrameMsg {
    // 控制命令
    CA_ChassisCtrl    chassis_ctrl;
    CA_HydraulicCtrl  hydraulic_ctrl;
    CA_WalkingCtrl    walking_ctrl;
    CA_ActuatorCtrl   actuator_ctrl;
    
    // 状态反馈
    CA_StatusFB01        status_fb_01;
    CA_MotorStatus02     motor_status_02;
    CA_WalkingStatus     walking_status;
    CA_DanfossSteering   danfoss_steering;
    
    uint8_t canFrameData[8];
};

/**
 * @brief 长安车辆CAN协议解析器
 */
class ChanganCANParser : public ChassisCan {
public:
    ChanganCANParser(std::string canPort);
    ~ChanganCANParser() = default;
    
    void initFuncMap() override;
    void SendCmdFunc() override;
    
    // 发送控制命令
    void sendChassisControl();
    void sendHydraulicControl(double dt);
    void sendWalkingControl(double dt);
    void sendActuatorControl(double dt);
    
    // 处理接收消息
    void handle0x181F0001(struct Canframe *recvCanFrame);  // 状态反馈
    void handle0x181F0002(struct Canframe *recvCanFrame);  // 电机状态
    void handle0x181F0003(struct Canframe *recvCanFrame);  // 行走电机状态
    void handle0x181F0007(struct Canframe *recvCanFrame);  // 丹佛斯转向

    // 角度与倾角传感器（新增）
    void handle0x00000581(struct Canframe *recvCanFrame);  // 车体倾角仪（Y_pitch）
    void handle0x00000582(struct Canframe *recvCanFrame);  // 大臂倾角仪（Y_pitch）
    void handle0x00000583(struct Canframe *recvCanFrame);  // 铲斗倾角仪（Y_pitch）
    void handle0x18FF0015(struct Canframe *recvCanFrame);  // 转向角度编码器
    
    void checkDataStructure();
    
    // ======= 外部配置接口（用于从launch/参数动态配置）=======
    void setSpeedPid(double kp, double ki, double kd,
                     double max_output, double min_output, double max_integral);
    void setArmPid(double kp, double ki, double kd,
                   double max_output, double min_output, double max_integral, double deadzone);
    void setBucketPid(double kp, double ki, double kd,
                      double max_output, double min_output, double max_integral, double deadzone);
    void setSteerPid(double kp, double ki, double kd,
                     double max_output, double min_output, double max_integral, double deadzone);
    void setZeroOffsets(double boom_zero_deg, double bucket_zero_deg, double steer_zero_deg);

private:
    // PID控制器实例
    AccToSpeedController speed_controller_;      // 车速控制器
    ValveToAngleController arm_controller_;      // 大臂角度控制器
    ValveToAngleController bucket_controller_;   // 铲斗角度控制器
    ValveToAngleController steer_controller_;    // 转向角度控制器
    
    // 当前反馈状态（从CAN读取）
    double current_speed_;         // 当前车速 (m/s)
    double current_arm_angle_;     // 当前大臂角度 (度)
    double current_bucket_angle_;  // 当前铲斗角度 (度)
    double current_steer_angle_;   // 当前转向角度 (度)
    
    // 心跳计数器
    uint8_t heartbeat_counter_;
    
    // 初始化PID参数（可从launch文件配置）
    void initPIDControllers();

    // ================= 统一的角度/倾角传感器数据维护结构 ================
    struct AngleSensorsData {
        // 原始Y轴俯仰（来自各自倾角仪），单位：度
        double body_pitch_y_deg = 0.0;    // 车体
        double boom_pitch_y_deg = 0.0;    // 大臂（绝对）
        double bucket_pitch_y_deg = 0.0;  // 铲斗（绝对）

        // 相对角（经差分及零点标定），单位：度
        double boom_rel_body_deg = 0.0;    // 大臂相对车体
        double bucket_rel_boom_deg = 0.0;  // 铲斗相对大臂

        // 转向角（来自角度编码器），单位：度
        double steering_angle_deg = 0.0;

        // 零点标定偏置（单位：度），正值表示加在计算后的相对角上
        double boom_zero_offset_deg = 0.0;    // 大臂相对车体零点
        double bucket_zero_offset_deg = 0.0;  // 铲斗相对大臂零点
        double steer_zero_offset_deg = 0.0;   // 转向角零点
    } angle_sensors_;

    // 根据当前 angle_sensors_ 计算相对角并写入 chassisProtoMsg
    void updateRelativeAnglesAndChassis();

    // 是否收到过转向角编码器数据
    bool steer_encoder_available_ = false;

    // 控制周期时间戳（用于计算真实 dt）
    std::chrono::steady_clock::time_point last_ctrl_time_;
    bool last_ctrl_time_inited_ = false;
};

} // namespace cannode
