/*
#pragma once

#include <cstdint>
#include <array>
#include <optional>
#include <stdint.h>

#define CA_CAN_DLEN 8

enum CanProtoId {
    CCU_CMD_1 = 0x18000001, // 上位机→VCU 控制命令1
    CCU_CMD_2 = 0x18000002, // 上位机→VCU 控制命令2
    CCU_CMD_3 = 0x18000003, // 上位机→VCU 控制命令3
    CCU_CMD_4 = 0x18000004, // 上位机→VCU 控制命令4
    VCU_INFO_1 = 0x181F0001, // VCU→上位机 信息1
    VCU_INFO_2 = 0x181F0002, // VCU→上位机 信息2
    VCU_INFO_3 = 0x181F0003, // VCU→上位机 信息3
    VCU_INFO_4 = 0x181F0004, // VCU→上位机 信息4
    VCU_INFO_5 = 0x181F0005, // VCU→上位机 信息5
    VCU_INFO_6 = 0x181F0006, // VCU→上位机 信息6
    VCU_INFO_7 = 0x181F0007  // VCU→上位机 信息7
};

#pragma pack(push, 1)

// ================================
// 上位机→VCU 控制命令定义
// ================================

// 上位机→VCU 控制命令1 (0x18000001)
typedef struct CanCtrlCmd1 {
    // byte0
    uint8_t high_voltage : 1;      // 整车上高压
    uint8_t reserved1 : 1;         // 预留
    uint8_t soft_estop : 1;        // 软急停
    uint8_t left_turn : 1;         // 左转灯
    uint8_t right_turn : 1;        // 右转灯
    uint8_t reserved2 : 2;         // 预留
    uint8_t vehicle_mode : 1;      // 整车模式
    
    // byte1
    uint8_t reserved3 : 1;          // 预留
    uint8_t heartbeat : 1;         // 心跳信号
    uint8_t speed_gear : 1;        // 速度挡位切换
    uint8_t work_light : 1;        // 工作大灯
    uint8_t clear_steer_error : 1; // 清除转向阀错误
    uint8_t high_beam : 1;         // 远光灯
    uint8_t low_beam : 1;          // 近光灯
    uint8_t horn : 1;              // 喇叭
    
    // byte2
    uint8_t alarm_lights : 5;      // 声光报警器
    uint8_t clear_error : 1;       // 清错
    uint8_t reserved4 : 2;         // 预留
    
    // byte3 - byte7 (40位预留)
    uint8_t reserved5[5];          // 40位预留空间
}CA0x18000001Struct;

// 上位机→VCU 控制命令2 (0x18000002)
typedef struct CanCtrlCmd2 {
    // byte0
    uint8_t hyd_motor_en : 1;      // 液压电机使能
    uint8_t hyd_work_mode : 2;     // 液压电机工作模式
    uint8_t hyd_econ_mode : 2;     // 液压电机经济模式
    uint8_t steer_valve_dir : 2;   // 转向电磁阀方向
    uint8_t reserved1 : 1;         // 预留
    
    // byte1 - byte2 (16位液压电机请求扭矩)
    uint16_t hyd_req_torque;       // 液压电机请求扭矩
    
    // byte3 - byte4 (16位液压电机请求转速)
    uint16_t hyd_req_speed;        // 液压电机请求转速
    
    // byte5 - byte6 (16位转向电磁阀电流)
    uint16_t steer_valve_current; // 转向电磁阀电流
    
    // byte7 (8位预留)
    uint8_t reserved2;             // 预留
}CA0x18000002Struct;

// 上位机→VCU 控制命令3 (0x18000003)
typedef struct CanCtrlCmd3 {
    // byte0
    uint8_t drive_motor_en : 1;    // 行走电机使能
    uint8_t drive_work_mode : 2;   // 行走电机工作模式
    uint8_t drive_direction : 2;   // 行走电机方向
    uint8_t drive_econ_mode : 2;   // 行走电机经济模式
    uint8_t reserved1 : 1;         // 预留
    
    // byte1 - byte2 (16位行走电机请求转速)
    uint16_t drive_req_speed;      // 行走电机请求转速
    
    // byte3 - byte4 (16位行走电机请求扭矩)
    uint16_t drive_req_torque;     // 行走电机请求扭矩
    
    // byte5 - byte6 (16位刹车电磁阀控制)
    uint16_t brake_valve_current;  // 刹车电磁阀控制
    
    // byte7 (8位预留)
    uint8_t reserved2;             // 预留
}CA0x18000003Struct;

// 上位机→VCU 控制命令4 (0x18000004)
typedef struct CanCtrlCmd4 {
    // byte0 - byte1 (16位大臂抬电磁阀)
    uint16_t arm_up_current;       // 大臂抬电磁阀电流
    
    // byte2 - byte3 (16位大臂降电磁阀)
    uint16_t arm_down_current;     // 大臂降电磁阀电流
    
    // byte4 - byte5 (16位铲斗内收电磁阀)
    uint16_t bucket_in_current;    // 铲斗内收电磁阀电流
    
    // byte6 - byte7 (16位铲斗外翻电磁阀)
    uint16_t bucket_out_current;   // 铲斗外翻电磁阀电流
}CA0x18000004Struct;

// ================================
// VCU→上位机 信息定义
// ================================

// VCU→上位机 信息1 (0x181F0001)
typedef struct CanVCUInfo1 {
    // byte0
    uint8_t high_voltage_sts : 1;  // 上高压状态
    uint8_t parking_brake : 1;     // 停车制动
    uint8_t soft_estop_sts : 1;    // 软急停状态
    uint8_t left_turn_sts : 1;     // 左转向灯
    uint8_t right_turn_sts : 1;    // 右转向灯
    uint8_t vehicle_mode_sts : 1;  // 整车模式
    uint8_t external_estop : 1;    // 外部急停状态
    uint8_t reserved1 : 1;         // 预留
    
    // byte1
    uint8_t heartbeat_sts : 1;     // 心跳信号
    uint8_t heartbeat_error : 1;   // 上位机心跳状态
    uint8_t alarm_lights_sts : 5;  // 声光报警器
    uint8_t charging_sts : 1;      // 充电状态
    
    // byte2 - byte3 (16位刹车控制电流)
    uint16_t brake_current;        // 刹车控制电流
    
    // byte4 (8位设备电量)
    uint8_t battery_soc;           // 设备电量
    
    // byte5
    uint8_t arm_up_error : 1;      // 大臂抬电磁阀故障
    uint8_t arm_down_error : 1;    // 大臂降电磁阀故障
    uint8_t bucket_in_error : 1;   // 铲斗内收电磁阀故障
    uint8_t bucket_out_error : 1;  // 铲斗外翻电磁阀故障
    uint8_t brake_valve_error : 1; // 脚制动电磁阀故障
    uint8_t steer_valve_error : 1; // 转向电磁阀故障
    uint8_t drive_com_error : 1;   // 行走电机通讯故障
    uint8_t hyd_com_error : 1;     // 液压电机通讯故障
    
    // byte6
    uint8_t fault_level : 2;       // 本体故障等级
    uint8_t low_beam_sts : 1;      // 近光灯状态
    uint8_t high_beam_sts : 1;     // 远光灯状态
    uint8_t work_light_sts : 1;    // 工作灯状态
    uint8_t horn_feedback : 1;     // 喇叭反馈
    uint8_t speed_gear_fb : 1;     // 速度档反馈
    uint8_t steer_valve_estop : 1; // 转向阀故障急停
    
    // byte7
    uint8_t arm_up_fb : 1;         // 大臂抬动作反馈
    uint8_t arm_down_fb : 1;       // 大臂降动作反馈
    uint8_t bucket_in_fb : 1;      // 铲斗内收动作反馈
    uint8_t bucket_out_fb : 1;     // 铲斗外翻动作反馈
    uint8_t left_valve_fb : 1;     // 左转电磁阀反馈
    uint8_t right_valve_fb : 1;    // 右转电磁阀反馈
    uint8_t reserved2 : 2;         // 预留
} CA0x181F0001Struct;

// VCU→上位机 信息2 (0x181F0002)
typedef struct CanVCUInfo2 {
    // byte0 - byte1 (16位液压电机实际转速)
    uint16_t hyd_actual_speed;     // 液压电机实际转速
    
    // byte2 - byte3 (16位液压电机实际扭矩)
    uint16_t hyd_actual_torque;    // 液压电机实际扭矩
    
    // byte4 - byte5 (16位液压电机电流)
    uint16_t hyd_current;          // 液压电机电流
    
    // byte6
    uint8_t hyd_motor_en : 1;      // 液压电机使能信号
    uint8_t reserved1 : 2;          // 预留
    uint8_t hyd_work_mode : 2;     // 液压电机工作模式
    
    // byte7
    uint8_t drive_motor_en : 1;    // 行走电机使能信号
    uint8_t drive_direction : 2;   // 行进电机挡位信息
    uint8_t drive_work_mode : 2;   // 行走电机工作模式
    uint8_t drive_econ_mode : 2;   // 行走电机经济模式
    uint8_t reserved2 : 1;         // 预留
} CA0x181F0002Struct;

// VCU→上位机 信息3 (0x181F0003)
typedef struct CanVCUInfo3 {
    // byte0 - byte1 (16位行走电机电流)
    uint16_t drive_current;         // 行走电机电流
    
    // byte2 - byte3 (16位行走电机实际扭矩)
    uint16_t drive_actual_torque;   // 行走电机实际扭矩
    
    // byte4 - byte5 (16位行走电机转速/车速)
    uint16_t drive_speed;           // 行走电机转速/车速
    
    // byte6 - byte7 (16位轮速)
    uint16_t wheel_speed;           // 轮速
} CA0x181F0003Struct;

// VCU→上位机 信息4 (0x181F0004)
typedef struct CanVCUInfo4 {
    // byte0 - byte1 (16位行进电机一功率)
    uint16_t drive_power;          // 行进电机一功率
    
    // byte2 - byte3 (16位预留)
    uint16_t reserved1;             // 预留
    
    // byte4 - byte5 (16位液压电机功率)
    uint16_t hyd_power;             // 液压电机功率
    
    // byte6 - byte7 (16位转向阀实际电流)
    uint16_t steer_valve_current;  // 转向阀实际电流
} CA0x181F0004Struct;

// VCU→上位机 信息5 (0x181F0005)
typedef struct CanVCUInfo5 {
    // 64位动臂、铲斗实际反馈电流
    uint16_t arm_up_feedback;       // 动臂举升反馈电流
    uint16_t arm_down_feedback;     // 动臂下落反馈电流
    uint16_t bucket_in_feedback;    // 铲斗收斗反馈电流
    uint16_t bucket_out_feedback;   // 铲斗翻斗反馈电流
} CA0x181F0005Struct;

// VCU→上位机 信息6 (0x181F0006)
typedef struct CanVCUInfo6 {
    // 64位各个关节油压
    uint16_t arm_large_pressure;    // 动臂大腔油压
    uint16_t arm_small_pressure;    // 动臂小腔油压
    uint16_t bucket_large_pressure; // 铲斗大腔油压
    uint16_t bucket_small_pressure; // 铲斗小腔油压
} CA0x181F0006Struct;

// VCU→上位机 信息7 (0x181F0007)
typedef struct CanVCUInfo7 {
    // byte0 - byte1 (16位丹佛斯方向机角度)
    uint16_t denison_angle;         // 丹佛斯方向机角度
    
    // byte2 (8位丹佛斯状态)
    uint8_t denison_status;         // 丹佛斯状态
    
    // byte3 - byte7 (40位预留)
    uint8_t reserved[5];            // 40位预留空间
} CA0x181F0007Struct;

#pragma pack(pop)


union CanFrameMsg {
  //cmd1 模式 上下电 大灯 清故障等
  CA0x18000001Struct CA0x181F0001St;
  //cmd2 液压电机 转向阀等
  CA0x18000002Struct CA0x181F0002St;
  //cmd3 行走电机
  CA0x18000003Struct CA0x181F0003St;
  //cmd4 动臂铲斗电磁阀
  CA0x18000004Struct CA0x181F0004St;

  //info1 灯 故障 大臂铲斗动作等
  CA0x181F0001Struct CA0x181F0001St;
  //info2 液压电机 行走电机模式等
  CA0x181F0002Struct CA0x181F0002St;
  //info3 行走电机扭矩转速等
  CA0x181F0003Struct CA0x181F0003St;
  //info4 行走电机功率 液压电机功率 转向阀电流等
  CA0x181F0004Struct CA0x181F0004St;
  //info5 动臂铲斗电流等
  CA0x181F0005Struct CA0x181F0005St;
  //info6 动臂铲斗油压等
  CA0x181F0006Struct CA0x181F0006St;
  //info7 丹佛斯信息等
  CA0x181F0007Struct CA0x181F0007St;

  uint8_t canFrameData[8];
};


class ChanganCANParser: public ChassisCan
{
public:
    ChanganCANParser(std::string canPort);
    ~ChanganCANParser() = default;
    // 解析CAN帧
    std::optional<ChanganData> ParseCANFrame(const CANFrame& frame);
    void initFuncMap() override;
    void SendCmdFunc() override;

private:
    // 解析具体的数据字段
    //std::optional<ChanganData> ParseDataField(const std::array<uint8_t, 8>& data);
};
*/