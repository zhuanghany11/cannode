
#include <iostream>
#include <cstdint>
#include <cstring>
#include <mutex> 
#include <linux/can.h>
#include "../chassis_driver.h"
#include "shantui.h"
namespace cannode {

ShantuiCANParser::ShantuiCANParser(std::string canPort):ChassisCan(canPort) {
  checkdatastructure();
  initFuncMap();
}

void ShantuiCANParser::initFuncMap() {
  handleChassisCanFuncMap[0x18FF2021] = static_cast<void (ChassisCan::*)(Canframe*)>(&ShantuiCANParser::handle0x18FF2021);
  handleChassisCanFuncMap[0x18FF2221] = static_cast<void (ChassisCan::*)(Canframe*)>(&ShantuiCANParser::handle0x18FF2221);
  handleChassisCanFuncMap[0x04854001] = static_cast<void (ChassisCan::*)(Canframe*)>(&ShantuiCANParser::handle0x04854001);
  handleChassisCanFuncMap[0x18501F50] = static_cast<void (ChassisCan::*)(Canframe*)>(&ShantuiCANParser::handle0x18501F50);
}

void ShantuiCANParser::SendCmdFunc() {
  static int cnt_20ms = 0;
  static int cnt_100ms = 0;
  if (++cnt_20ms >= 2) {
    cnt_20ms = 0;
    sendCtrlInfo1();
  }
  if (++cnt_100ms >= 10) {
    cnt_100ms = 0;
    sendCtrlInfo2();
  }
}

void ShantuiCANParser::handle0x18FF2021(struct Canframe *recvCanFrame) {
      // 添加调试信息
  std::cout << "Handling CAN frame ID: 0x18FF2021, DLC: " 
            << recvCanFrame->frame.can_dlc << std::endl;
  
  // 长度检查
  if (recvCanFrame->frame.can_dlc != 8) {
      std::cerr << "Unexpected CAN frame DLC: " << recvCanFrame->frame.can_dlc << std::endl;
      return;
  }

  VCU_heartbeat_cnt = 0;
  union STCanFrameMsg canFrameUnion = {{0}};
  std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
  control::canbus::Chassis::GearPosition current_gear = gear_state_to_proto(canFrameUnion.ST0X18FF2021St.V2HMI_St_CurrPRNDSt);
  {
    std::lock_guard<std::mutex> lk1(vehicleStatusMutex);
    chassisProtoMsg.set_gear_location(current_gear);
    chassisProtoMsg.set_speed_mps(canFrameUnion.ST0X18FF2021St.V2HMI_V_VehicleSpeed*0.1f/3.6f);
  }
}

void ShantuiCANParser::handle0x18FF2221(struct Canframe *recvCanFrame) {
  union STCanFrameMsg canFrameUnion = {{0}};
  std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
  {
    //std::lock_guard<std::mutex> lk1(vehicleStatusMutex);
    //chassisProtoMsg.V2HMI_St_CurrPRNDSt = canFrameUnion.ST0X18FF2221St.V2HMI_St_VehLVSt;
  }
}

void ShantuiCANParser::handle0x04854001(struct Canframe *recvCanFrame) {
  union STCanFrameMsg canFrameUnion = {{0}};
  std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
  {
    //std::lock_guard<std::mutex> lk1(vehicleStatusMutex);
    //chassisProtoMsg.V2HMI_St_CurrPRNDSt = (canFrameUnion.ST0X04854001St.Current_high<<8) | canFrameUnion.ST0X04854001St.Current_low;

  }
}
void ShantuiCANParser::handle0x18501F50(struct Canframe *recvCanFrame) {
    union STCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    control::canbus::Chassis::DrivingMode driving_mode = driving_mode_to_proto(canFrameUnion.ST0X18501F50St.VCU_FlgMdVeh);
    {
        std::lock_guard<std::mutex> lk1(vehicleStatusMutex);
        chassisProtoMsg.set_driving_mode(driving_mode);
        chassisProtoMsg.set_arm_angle(canFrameUnion.ST0X18501F50St.VCU_ArmAgl*0.1f);
        chassisProtoMsg.set_shovel_angle(canFrameUnion.ST0X18501F50St.VCU_BucketAgl*0.1f);
        chassisProtoMsg.set_steering_percentage(canFrameUnion.ST0X18501F50St.VCU_TurnLRAgl*0.1f -50.0f);
    }
}

void ShantuiCANParser::sendCtrlInfo1() { 
  static int ctrl_cmd1_cnt_ = 0;
  ST0x18501E50Struct ctrlInfo1 = {0};
  can_frame canFrame;
  if(++ctrl_cmd1_cnt_ > 15) ctrl_cmd1_cnt_ = 0;
  canFrame.can_id = 0X18501E50;
  canFrame.can_dlc = 8;
  {
    std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
    ctrlInfo1.ACU_Life1 = ctrl_cmd1_cnt_;
    ctrlInfo1.ACU_TurnCmd = ctl_cmd.steering_rate()*100.0f;
    ctrlInfo1.ACU_ArmCmd = ctl_cmd.arm_angle()*10.0f;
    ctrlInfo1.ACU_ArmEn = ctl_cmd.arm_enable();
    ctrlInfo1.ACU_BucketCmd = ctl_cmd.shovel_angle()*10.0f;
    ctrlInfo1.ACU_BucketEn = ctl_cmd.shovel_enable();
    ctrlInfo1.ACU_Tgrspd = (ctl_cmd.speed()*3.6f)*10.0f;
    ctrlInfo1.ACU_DrvModeReq = ctl_cmd.driving_mode();
    ctrlInfo1.ACU_DrvGear = ctl_cmd.gear_location();
    ctrlInfo1.ACU_EStop = ctl_cmd.estop();
    ctrlInfo1.ACU_RapidShake = ctl_cmd.rapid_shake();
  }
  memcpy(canFrame.data, &ctrlInfo1, sizeof(ST0x18501E50Struct));
  SendCanFrame(canFrame);
}

void ShantuiCANParser::sendCtrlInfo2() { 
  static int ctrl_cmd2_cnt_ = 0;
  ST0x18502E50Struct ctrlInfo2 = {0};
  can_frame canFrame;
  if(++ctrl_cmd2_cnt_ > 15) ctrl_cmd2_cnt_ = 0;
  canFrame.can_id = 0X18502E50;
  canFrame.can_dlc = 8;
  {
    std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
    ctrlInfo2.ACU_Life2 = ctrl_cmd2_cnt_;
    ctrlInfo2.ACU_KeyOn = 1;
    ctrlInfo2.ACU_KeyStart = 1;
    ctrlInfo2.ACU_ParkReq = ctl_cmd.parking_brake();
    ctrlInfo2.ACU_Horn = 0;
    ctrlInfo2.ACU_AccPedlPct = static_cast<uint16_t>(ctl_cmd.throttle()*40.0f); // 0-100
    ctrlInfo2.ACU_TurnLeftLgt = 0;
    ctrlInfo2.ACU_TurnRightLgt = 0;
    ctrlInfo2.ACU_FWorkLgt = 0;
    ctrlInfo2.ACU_BWorkLgt = 0;
    ctrlInfo2.ACU_BrakePct = static_cast<uint8_t>(ctl_cmd.brake()*2.5f); // 0-100
  }
  memcpy(canFrame.data, &ctrlInfo2, sizeof(ST0x18502E50Struct));
  SendCanFrame(canFrame);
}

void ShantuiCANParser::checkdatastructure() {
  // 确保结构体大小正确
  static_assert(sizeof(ST0x18501E50Struct) == 8, "Invalid ST0x18501E50St size");
  static_assert(sizeof(ST0x18502E50Struct) == 8, "Invalid ST0x18502E50St size");
  static_assert(sizeof(ST0X18FF2021Struct) == 8, "Invalid ST0X18FF2021St size");
  static_assert(sizeof(ST0X18FF2221Struct) == 8, "Invalid ST0X18FF2221St size");
  static_assert(sizeof(ST0X04854001Struct) == 8, "Invalid ST0X04854001St size");
  static_assert(sizeof(ST0X18501F50Struct) == 8, "Invalid ST0X18501F50St size");
  static_assert(sizeof(ST0X18502F50Struct) == 8, "Invalid ST0X18502F50St size");
  static_assert(sizeof(ST0X18503F50Struct) == 8, "Invalid ST0X18503F50St size");
  static_assert(sizeof(ST0X18DE1021Struct) == 8, "Invalid ST0X18DE1021St size");
  static_assert(sizeof(ST0X18140021Struct) == 8, "Invalid ST0X18140021St size");
  static_assert(sizeof(ST0X18140121Struct) == 8, "Invalid ST0X18140121St size");
  static_assert(sizeof(ST0X18141021Struct) == 8, "Invalid ST0X18141021St size");
  static_assert(sizeof(ST0X18141121Struct) == 8, "Invalid ST0X18141121St size");
  static_assert(sizeof(ST0X18240021Struct) == 8, "Invalid ST0X18240021St size");
  static_assert(sizeof(ST0X18240121Struct) == 8, "Invalid ST0X18240121St size");
    // std::cout << "sizeof(ST0x18501E50Struct): " << sizeof(ST0x18501E50Struct) << std::endl;
    // std::cout << "sizeof(ST0x18502E50Struct): " << sizeof(ST0x18502E50Struct) << std::endl;
    // std::cout << "sizeof(ST0X18FF2021Struct): " << sizeof(ST0X18FF2021Struct) << std::endl;
    // std::cout << "sizeof(ST0X18FF2221Struct): " << sizeof(ST0X18FF2221Struct) << std::endl;
    // std::cout << "sizeof(ST0X04854001Struct): " << sizeof(ST0X04854001Struct) << std::endl;
    // std::cout << "sizeof(ST0X18501F50Struct): " << sizeof(ST0X18501F50Struct) << std::endl;
    // std::cout << "sizeof(ST0X18502F50Struct): " << sizeof(ST0X18502F50Struct) << std::endl;
    // std::cout << "sizeof(ST0X18503F50Struct): " << sizeof(ST0X18503F50Struct) << std::endl;
    // std::cout << "sizeof(ST0X18DE1021Struct): " << sizeof(ST0X18DE1021Struct) << std::endl;
    // std::cout << "sizeof(ST0X18140021Struct): " << sizeof(ST0X18140021Struct) << std::endl;
    // std::cout << "sizeof(ST0X18140121Struct): " << sizeof(ST0X18140121Struct) << std::endl;
    // std::cout << "sizeof(ST0X18141021Struct): " << sizeof(ST0X18141021Struct) << std::endl;
    // std::cout << "sizeof(ST0X18141121Struct): " << sizeof(ST0X18141121Struct) << std::endl;
    // std::cout << "sizeof(ST0X18240021Struct): " << sizeof(ST0X18240021Struct) << std::endl;
    // std::cout << "sizeof(ST0X18240121Struct): " << sizeof(ST0X18240121Struct) << std::endl;
}

control::canbus::Chassis::GearPosition ShantuiCANParser::gear_state_to_proto(uint8_t gear_state) {
  switch (gear_state) {
    case 0:
      return control::canbus::Chassis::GEAR_NEUTRAL;
    case 1:
      return control::canbus::Chassis::GEAR_REVERSE;
    case 2:
      return control::canbus::Chassis::GEAR_NEUTRAL;
    case 3:
      return control::canbus::Chassis::GEAR_DRIVE;
    default:
      return control::canbus::Chassis::GEAR_NEUTRAL;
  }
}
control::canbus::Chassis::DrivingMode ShantuiCANParser::driving_mode_to_proto(uint8_t driving_mode) {
  switch (driving_mode) {
    case 0:
      return control::canbus::Chassis::COMPLETE_MANUAL;
    case 1:
      return control::canbus::Chassis::COMPLETE_AUTO_DRIVE;
    default:
      return control::canbus::Chassis::COMPLETE_MANUAL;
  }
}

} // namespace cannode
