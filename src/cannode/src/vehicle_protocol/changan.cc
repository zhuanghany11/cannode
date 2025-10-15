/*
#include "chassis_driver.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>

void ChanganCANParser::ChanganCANParser(std::string canPort):ChassisCan(canPort)
{
  initFuncMap();
}

void ChanganCANParser::initFuncMap()
{
  handleChassisCanFuncMap[0x181F0001] = &ChanganCANParser::handle0x181F0001;
  handleChassisCanFuncMap[0x18FFF110] = &ChanganCANParser::handle0x18FFF110;
  handleChassisCanFuncMap[0x183060A1] = &ChanganCANParser::handle0x183060A1;
}

void ChanganCANParser::SendCmdFunc()
{
  static int cnt_50ms = 0;
  // 50ms
  if (++cnt_50ms >= 5) {
    cnt_50ms = 0;
    sendVehicleHeartNTE120();
    // sendVehicleMoveCmdNTE120();
    // sendVehicleActCmdNTE120();
    // sendVehicleCmdNTE120();
    // sendVehicleRedundantCmd();
    // sendVehicleSpeedToVCU();
  }
}

void ChanganCANParser::handle0x181F0001(struct Canframe *recvCanFrame) {
  static_assert(sizeof(CA0x181F0001Struct) == 8, "Invalid CanCtrlInfo1 size");

  union CanFrameMsg canFrameUnion = {{0}};
  std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CA_CAN_DLEN);
  
  if(!CheckCanEnumMessage( canFrameUnion.CA0x181F0001St.wire_status_EBR_SystemMode, 0,5,"制动腿状态取值校验失败"))
  {
    vehicleFaultOutput.wire_status_can_fault_check_AiEr_message = 1;
    return;
  }
  else{
      if(canFrameUnion.CA0x181F0001St.wire_status_EBR_SystemMode == 4 || canFrameUnion.CA0x181F0001St.wire_status_EBR_SystemMode == 5){
      //log记录故障
      LOG_Channel_Error("canlog") << "制动腿异常工作或故障";
      vehicleFaultOutput.wire_status_can_fault_check_AiEr_status = 1;
    }
    else{
      vehicleFaultOutput.wire_status_can_fault_check_AiEr_status = 0;
    }

    vehicleFaultOutput.wire_status_can_fault_check_AiEr_message = 0;
  }

  std::lock_guard<std::mutex> lk1(vehicleStatusMutex);

  ebr_heart_cnt = 0.0f;

  vehicleStatusOutput.wire_status_CheckSum_EBR_1 =
    canFrameUnion.ntn0x18fe0621St.wire_status_CheckSum_EBR_1;
}

void changanCANParser::sendVehicleHeartNTE120()
{
  can_frame heartFrame;
  heartFrame.frame.id = 0x181F0001;
  heartFrame.frame.len = 8;
  heartFrame.frame.data[0] = 0x01;  // 心跳包标识
  heartFrame.frame.data[1] = 0x00;
  heartFrame.frame.data[2] = 0x00;
  heartFrame.frame.data[3] = 0x00;
  heartFrame.frame.data[4] = 0x00;
  heartFrame.frame.data[5] = 0x00;
  heartFrame.frame.data[6] = 0x00;
  heartFrame.frame.data[7] = 0x00;

  sendCanFrame(&heartFrame);

      std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    // 确保结构体大小正确
    static_assert(sizeof(CanCtrlCmd1) == 8, "Invalid CanCtrlCmd1 size");
    static_assert(sizeof(CanCtrlCmd2) == 8, "Invalid CanCtrlCmd2 size");
    static_assert(sizeof(CanCtrlCmd3) == 8, "Invalid CanCtrlCmd3 size");
    static_assert(sizeof(CanCtrlCmd4) == 8, "Invalid CanCtrlCmd4 size");
    
    // 创建并发送命令1
    can_frame frame1;
    frame1.can_id = CCU_CMD_1;
    frame1.can_dlc = sizeof(CanCtrlCmd1);
    memcpy(frame1.data, &ctrl_cmd1_, sizeof(CanCtrlCmd1));
    if (write(can_fd_, &frame1, sizeof(frame1)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send CCU_CMD_1");
    }
}

*/
