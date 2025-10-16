#pragma once

#include <string>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <mutex> 
#include <linux/can.h> 
#include <sys/time.h> 
#include "socket_can_interface.h"
#include "sa_msgs/msg/proto_adapter.hpp"
#include "../protobuf/out/common_msgs/chassis_msgs/chassis.pb.h"
#include "../protobuf/out/control_msgs/control_cmd.pb.h"
namespace cannode {

#define PERIODTIME_SEND_THREAD 10  // ms
#define CAN_DLEN 8
struct Canframe {
  struct can_frame frame;
  struct timeval timestamp;
};

/**
 * @brief 底盘 CAN 抽象基类
 *
 * 职责：
 *  - 提供 CAN 发送/接收能力
 *  - 管理控制指令（`ctl_cmd`）与底盘状态（`chassisProtoMsg`）
 *  - 周期调度发送（`SendCmdProc`）与回调分发（`callHandleWireCanCmdFunc`）
 */
class ChassisCan : public SocketCanInterface {
public:
  ChassisCan(std::string canPort);
  virtual ~ChassisCan();
  /**
   * @brief 获取底盘状态 protobuf 引用
   * 调用处：`MsgManager::PublishChassisStatus()`。
   */
  control::canbus::Chassis& get_chassis_status_msg();
  /**
   * @brief 设置控制指令（由上层 ROS2 订阅回调注入）
   * 调用处：`MsgManager::procControlImplement()`。
   */
  void setCtlCmd(const control::ControlCommand& cmd);
  /**
   * @brief 周期发送线程入口，内部调用派生类 `SendCmdFunc()`
   * 调用处：`Initialize()` 创建的发送线程。
   */
  void SendCmdProc();
  /**
   * @brief 初始化：启动发送与接收线程
   * 调用处：协议解析器实例创建后立即调用。
   */
  void Initialize();
  /**
   * @brief CAN 接收循环，读取总线数据并分发
   * 调用处：`Initialize()` 创建的接收线程。
   */
  void StartRecv();
  /**
   * @brief 接收帧处理，掩掉 EFF 标志后分发到具体 handler
   * 调用处：`StartRecv()` 内部。
   */
  void HandleRecvData(struct Canframe *recvCanFrame);
  /**
   * @brief 根据 `handleChassisCanFuncMap` 调用对应处理函数
   * 调用处：`HandleRecvData()` 内部。
   */
  void callHandleWireCanCmdFunc(struct Canframe *recvCanFrame);

public:
  //映射命令处理函数
  typedef void (ChassisCan::*handleChassisCanFuncPtr)(struct Canframe *);
  std::unordered_map<uint32_t, handleChassisCanFuncPtr>
      handleChassisCanFuncMap;
  virtual void initFuncMap() = 0;
  virtual void SendCmdFunc() = 0;
  control::canbus::Chassis chassisProtoMsg; // 底盘状态 protobuf消息(各个车型管理赋值)
  control::ControlCommand ctl_cmd; // 控制命令 protobuf消息（回调赋值）
  std::mutex vehicleStatusMutex;
  std::mutex vehicleCtlCmdMutex;
  uint16_t VCU_heartbeat_cnt = 0;
  bool VCU_Life1 = true;
};  // class ChassisCan
}  // namespace cannode
