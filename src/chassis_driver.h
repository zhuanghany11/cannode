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

class ChassisCan : public SocketCanInterface {
public:
  ChassisCan(std::string canPort);
  virtual ~ChassisCan();
  control::canbus::Chassis& get_chassis_status_msg();
  void setCtlCmd(const control::ControlCommand& cmd);
  void SendCmdProc();
  void Initialize();
  void StartRecv();
  void HandleRecvData(struct Canframe *recvCanFrame);
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
