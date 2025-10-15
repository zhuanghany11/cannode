#if 0
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// CAN ID 定义
#define VCU_TO_HOST_ID_1 0x18FF0001
#define VCU_TO_HOST_ID_2 0x18FF0002
#define VCU_TO_HOST_ID_3 0x18FF0003
#define VCU_TO_HOST_ID_4 0x18FF0004
#define VCU_TO_HOST_ID_5 0x18FF0005
#define VCU_TO_HOST_ID_6 0x18FF0006
#define VCU_TO_HOST_ID_9 0x18FF0009
#define VCU_TO_HOST_ID_A 0x18FF000A
#define HOST_TO_VCU_ID_1 0x18FFF001
#define HOST_TO_VCU_ID_2 0x18FFF002
#define HOST_TO_VCU_ID_B0EE 0x18FFB0EE
#define HOST_TO_VCU_ID_B1EF 0x18FFB1EF

#pragma pack(push, 1)

// CAN ID 0x18FF0001 数据结构
typedef struct {
    uint8_t high_voltage_status : 1;
    uint8_t parking_brake : 1;
    uint8_t horn_status : 1;
    uint8_t left_turn_signal : 1;
    uint8_t right_turn_signal : 1;
    uint8_t walk_motor_mode : 2;
    uint8_t wet_brake_alarm : 1;
    uint8_t emergency_stop : 1;
    uint8_t gear_signal : 3;
    uint8_t rotation_alarm : 3;
    uint8_t heartbeat_status : 1;
    uint16_t brake_control;
    uint16_t front_rear_angle;
    uint8_t battery_level;
    uint8_t charging_status : 1;
    uint8_t hydraulic_lock : 1;
    uint8_t fault_level : 2;
    uint8_t turtle_rabbit_gear : 1;
    uint8_t work_light : 1;
    uint8_t vehicle_mode : 2;
} VCUToHost1;

// CAN ID 0x18FF0002 数据结构
typedef struct {
    uint16_t boom_lift_current;
    uint16_t boom_lower_current;
    uint16_t bucket_close_current;
    uint16_t bucket_open_current;
} VCUToHost2;

// CAN ID 0x18FF0003 数据结构
typedef struct {
    uint16_t boom_big_pressure;
    uint16_t boom_small_pressure;
    uint16_t bucket_big_pressure;
    uint16_t bucket_small_pressure;
} VCUToHost3;

// CAN ID 0x18FF0004 数据结构
typedef struct {
    uint16_t hydraulic_motor_speed;
    uint16_t hydraulic_motor_torque;
    uint16_t hydraulic_motor_current;
    uint8_t hydraulic_motor_enable : 1;
} VCUToHost4;

// CAN ID 0x18FF0005 数据结构
typedef struct {
    uint16_t walk_motor_current;
    uint16_t walk_motor_torque;
    uint16_t walk_motor_speed;
    uint8_t walk_motor_enable : 1;
} VCUToHost5;

// CAN ID 0x18FF0006 数据结构
typedef struct {
    uint8_t throttle_opening;
    uint8_t brake_opening;
    uint8_t heartbeat_signal : 1;
    uint8_t can_loss_1 : 1;
    uint8_t can_loss_2 : 1;
    uint8_t can_loss_3 : 1;
    uint8_t can_loss_4 : 1;
    uint8_t walk_motor_fault : 1;
    uint8_t hydraulic_motor_fault : 1;
    uint8_t boom_lift_valve_fault : 1;
    uint8_t boom_lower_valve_fault : 1;
    uint8_t bucket_close_valve_fault : 1;
    uint8_t bucket_open_valve_fault : 1;
    uint8_t foot_brake_valve_fault : 1;
    uint8_t turn_valve_fault : 1;
    uint8_t low_beam : 1;
    uint8_t high_beam : 1;
    uint8_t hydraulic_motor_voltage;
    uint16_t turn_valve_current;
    uint8_t hydraulic_motor_mode : 2;
    uint8_t vehicle_mode_2 : 2;
} VCUToHost6;

// CAN ID 0x18FF0009 数据结构
typedef struct {
    uint16_t rear_motor_current;
    uint8_t rear_motor_voltage;
    uint16_t front_motor_current;
    uint8_t front_motor_voltage;
    uint16_t hydraulic_motor_current_2;
} VCUToHost9;

// CAN ID 0x18FF000A 数据结构
typedef struct {
    uint16_t boom_angle;
    uint16_t bucket_angle;
} VCUToHostA;

// CAN ID 0x18FFF001 数据结构
typedef struct {
    uint8_t high_voltage : 1;
    uint8_t parking_brake : 1;
    uint8_t emergency_mode : 1;
    uint8_t left_turn_signal : 1;
    uint8_t right_turn_signal : 1;
    uint8_t walk_motor_mode : 2;
    uint8_t mode_switch : 1;

    uint8_t hydraulic_lock : 1;
    uint8_t heartbeat_signal : 1;
    uint8_t turtle_rabbit_gear : 1;
    uint8_t work_light : 1;
    uint8_t low_voltage_signal : 1;
    uint8_t high_beam : 1;
    uint8_t low_beam : 1;
    uint8_t horn : 1;

    uint8_t rotation_alarm : 3;
    uint8_t left_turn_valve : 1;
    uint8_t right_turn_valve : 1;
    uint8_t clear_error : 1;
    uint8_t hydraulic_motor_mode : 2;
    
    uint16_t turn_pwm_or_angle;

    uint8_t res6;
    uint8_t res7;
    uint8_t res8;
} HostToVCU1;

typedef struct {
    uint16_t boom_lift_pwm;
    uint16_t boom_lower_pwm;
    uint16_t bucket_close_pwm;
    uint16_t bucket_open_pwm;
} HostToVCU2;

typedef struct {
    uint8_t hydraulic_motor_enable : 1;
    uint8_t hydraulic_motor_mode : 2;
    uint8_t gear : 2;
    uint8_t res1 : 3;
    uint16_t target_speed;
    uint16_t request_torque;
    uint16_t brake_valve_pwm;
    uint8_t res8;
} HostToVCUB0EE;

typedef struct {
    uint8_t hydraulic_motor_enable;
    uint16_t request_torque;
    uint16_t target_speed;
    uint8_t work_mode;
    uint8_t res7;
    uint8_t res8;
} HostToVCUB1EF;

#pragma pack(pop)

// CAN帧结构
typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
} CanFrame;

// 基础封装/解析函数
inline void parse_vcu_to_host1(const uint8_t *buffer, VCUToHost1 *data) {
    data->high_voltage_status = (buffer[0] >> 0) & 0x01;
    data->parking_brake = (buffer[0] >> 1) & 0x01;
    data->horn_status = (buffer[0] >> 2) & 0x01;
    data->left_turn_signal = (buffer[0] >> 3) & 0x01;
    data->right_turn_signal = (buffer[0] >> 4) & 0x01;
    data->walk_motor_mode = (buffer[0] >> 5) & 0x03;
    data->wet_brake_alarm = (buffer[0] >> 7) & 0x01;
    data->emergency_stop = (buffer[1] >> 0) & 0x01;
    data->gear_signal = (buffer[1] >> 1) & 0x07;
    data->rotation_alarm = (buffer[1] >> 4) & 0x07;
    data->heartbeat_status = (buffer[1] >> 7) & 0x01;
    data->brake_control = (buffer[2] << 8) | buffer[3];
    data->front_rear_angle = (buffer[4] << 8) | buffer[5];
    data->battery_level = buffer[6];
    data->charging_status = (buffer[7] >> 0) & 0x01;
    data->hydraulic_lock = (buffer[7] >> 1) & 0x01;
    data->fault_level = (buffer[7] >> 2) & 0x03;
    data->turtle_rabbit_gear = (buffer[7] >> 4) & 0x01;
    data->work_light = (buffer[7] >> 5) & 0x01;
    data->vehicle_mode = (buffer[7] >> 6) & 0x03;
}

inline void parse_vcu_to_host2(const uint8_t *buffer, VCUToHost2 *data) {
    data->boom_lift_current = (buffer[0] << 8) | buffer[1];
    data->boom_lower_current = (buffer[2] << 8) | buffer[3];
    data->bucket_close_current = (buffer[4] << 8) | buffer[5];
    data->bucket_open_current = (buffer[6] << 8) | buffer[7];
}

inline void parse_vcu_to_host3(const uint8_t *buffer, VCUToHost3 *data) {
    data->boom_big_pressure = (buffer[0] << 8) | buffer[1];
    data->boom_small_pressure = (buffer[2] << 8) | buffer[3];
    data->bucket_big_pressure = (buffer[4] << 8) | buffer[5];
    data->bucket_small_pressure = (buffer[6] << 8) | buffer[7];
}

inline void parse_vcu_to_host4(const uint8_t *buffer, VCUToHost4 *data) {
    data->hydraulic_motor_speed = (buffer[0] << 8) | buffer[1];
    data->hydraulic_motor_torque = (buffer[2] << 8) | buffer[3];
    data->hydraulic_motor_current = (buffer[4] << 8) | buffer[5];
    data->hydraulic_motor_enable = (buffer[6] & 0x01);
}

inline void parse_vcu_to_host5(const uint8_t *buffer, VCUToHost5 *data) {
    data->walk_motor_current = (buffer[0] << 8) | buffer[1];
    data->walk_motor_torque = (buffer[2] << 8) | buffer[3];
    data->walk_motor_speed = (buffer[4] << 8) | buffer[5];
    data->walk_motor_enable = (buffer[6] & 0x01);
}

inline void parse_vcu_to_host6(const uint8_t *buffer, VCUToHost6 *data) {
    data->throttle_opening = buffer[0];
    data->brake_opening = buffer[1];
    data->heartbeat_signal = (buffer[2] >> 0) & 0x01;
    data->can_loss_1 = (buffer[2] >> 1) & 0x01;
    data->can_loss_2 = (buffer[2] >> 2) & 0x01;
    data->can_loss_3 = (buffer[2] >> 3) & 0x01;
    data->can_loss_4 = (buffer[2] >> 4) & 0x01;
    data->walk_motor_fault = (buffer[2] >> 5) & 0x01;
    data->hydraulic_motor_fault = (buffer[2] >> 6) & 0x01;
    data->boom_lift_valve_fault = (buffer[3] >> 0) & 0x01;
    data->boom_lower_valve_fault = (buffer[3] >> 1) & 0x01;
    data->bucket_close_valve_fault = (buffer[3] >> 2) & 0x01;
    data->bucket_open_valve_fault = (buffer[3] >> 3) & 0x01;
    data->foot_brake_valve_fault = (buffer[3] >> 4) & 0x01;
    data->turn_valve_fault = (buffer[3] >> 5) & 0x01;
    data->low_beam = (buffer[4] >> 0) & 0x01;
    data->high_beam = (buffer[4] >> 1) & 0x01;
    data->hydraulic_motor_voltage = buffer[5];
    data->turn_valve_current = (buffer[6] << 8) | buffer[7];
    data->hydraulic_motor_mode = (buffer[8] >> 0) & 0x03;
    data->vehicle_mode_2 = (buffer[8] >> 2) & 0x03;
}

inline void parse_vcu_to_host9(const uint8_t *buffer, VCUToHost9 *data) {
    data->rear_motor_current = (buffer[0] << 8) | buffer[1];
    data->rear_motor_voltage = buffer[2];
    data->front_motor_current = (buffer[3] << 8) | buffer[4];
    data->front_motor_voltage = buffer[5];
    data->hydraulic_motor_current_2 = (buffer[6] << 8) | buffer[7];
}

inline void parse_vcu_to_hostA(const uint8_t *buffer, VCUToHostA *data) {
    data->boom_angle = (buffer[0] << 8) | buffer[1];
    data->bucket_angle = (buffer[2] << 8) | buffer[3];
}

// 封装Host到VCU的消息
inline void pack_host_to_vcu1(const HostToVCU1 *data, uint8_t *buffer) {
    buffer[0] = (data->high_voltage << 0) | (data->parking_brake << 1) | (data->emergency_mode << 2) | (data->left_turn_signal << 3) | (data->right_turn_signal << 4) | (data->walk_motor_mode << 5) | (data->mode_switch << 7);
    buffer[1] = (data->hydraulic_lock << 0) | (data->heartbeat_signal << 1) | (data->turtle_rabbit_gear << 2) | (data->work_light << 3) | (data->low_voltage_signal << 4) | (data->high_beam << 5) | (data->low_beam << 6) | (data->horn << 7);
    buffer[2] = (data->rotation_alarm << 0) | (data->left_turn_valve << 1) | (data->right_turn_valve << 2) | (data->clear_error << 3) | (data->hydraulic_motor_mode << 4);
    buffer[3] = (data->turn_pwm_or_angle >> 8) & 0xFF;
    buffer[4] = data->turn_pwm_or_angle & 0xFF;
}

inline void pack_host_to_vcu2(const HostToVCU2 *data, uint8_t *buffer) {
    buffer[0] = (data->boom_lift_pwm >> 8) & 0xFF;
    buffer[1] = data->boom_lift_pwm & 0xFF;
    buffer[2] = (data->boom_lower_pwm >> 8) & 0xFF;
    buffer[3] = data->boom_lower_pwm & 0xFF;
    buffer[4] = (data->bucket_close_pwm >> 8) & 0xFF;
    buffer[5] = data->bucket_close_pwm & 0xFF;
    buffer[6] = (data->bucket_open_pwm >> 8) & 0xFF;
    buffer[7] = data->bucket_open_pwm & 0xFF;
}

inline void pack_host_to_vcu_b0ee(const HostToVCUB0EE *data, uint8_t *buffer) {
    buffer[0] = (data->hydraulic_motor_enable << 0) | (data->hydraulic_motor_mode << 1) | (data->gear << 3);
    buffer[1] = (data->target_speed >> 8) & 0xFF;
    buffer[2] = data->target_speed & 0xFF;
    buffer[3] = (data->request_torque >> 8) & 0xFF;
    buffer[4] = data->request_torque & 0xFF;
    buffer[5] = (data->brake_valve_pwm >> 8) & 0xFF;
    buffer[6] = data->brake_valve_pwm & 0xFF;
}

inline void pack_host_to_vcu_b1ef(const HostToVCUB1EF *data, uint8_t *buffer) {
    buffer[0] = (data->hydraulic_motor_enable << 0);
    buffer[1] = (data->request_torque >> 8) & 0xFF;
    buffer[2] = data->request_torque & 0xFF;
    buffer[3] = (data->target_speed >> 8) & 0xFF;
    buffer[4] = data->target_speed & 0xFF;
    buffer[5] = (data->work_mode << 0);
}


// struct chassis_can_status {
//     uint8_t vehicle_mode;          // 车辆模式
//     uint8_t gear_signal;           // 档位信号
//     uint8_t brake_control;         // 制动控制
//     uint8_t rotation_alarm;        // 旋转报警
//     uint8_t walk_motor_enable;     // 行走电机使能
//     uint8_t hydraulic_motor_enable;// 液压电机使能
//     private:
//   VCUToHost1 vehicle_info_;  // 车辆信息
//   VCUToHost2 current_Info_;  // 动臂铲斗电流信息
//   VCUToHost3 pressure_Info_;  // 动臂铲斗关节油压信息
//   VCUToHost4 hydraulic_motor_Info_;  // 液压电机信息
//   VCUToHost5 walk_motor_Info_;  // 行走电机信息
//   VCUToHost6 status_Info_;  // 车辆状态信息
//   VCUToHost9 rear_front_motor_Info_;  // 控制信息
//   VCUToHostA angle_Info_;  // 动臂铲斗角度信息
//   HostToVCU1 set_vehicle_info_;  // 车辆控制信息
//   HostToVCU2 set_boom_lift_info_; // 动臂铲斗控制信息
//   HostToVCUB1EF set_hydraulic_motor_info_; // 液压电机控制信息
//   HostToVCUB0EE set_walk_motor_info_; // 行走电机控制信息
// };

#endif