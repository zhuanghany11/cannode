
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <cmath>
#include <mutex> 
#include <unistd.h>
#include <sys/ioctl.h> 
#include <linux/sockios.h>
#include <vector>
#include "chassis_driver.h"
namespace cannode {
using namespace std::chrono_literals;
using namespace control::canbus;
ChassisCan::ChassisCan(std::string canPort)
    : SocketCanInterface(canPort)
{
    // 初始化控制信息
    //memset(&set_vehicle_info_, 0, sizeof(HostToVCU1));
    //memset(&set_boom_lift_info_, 0, sizeof(HostToVCU2));
    //memset(&set_hydraulic_motor_info_, 0, sizeof(HostToVCUB1EF));
    //memset(&set_walk_motor_info_, 0, sizeof(HostToVCUB0EE));
    
    // 初始化VCU上报的信息
    //memset(&chassisProtoMsg, 0, sizeof(control::canbus::Chassis));
    chassisProtoMsg.Clear();
}

ChassisCan::~ChassisCan(){}

/**
 * @brief 获取底盘状态消息, chassis中定义管理底盘can数据
 */
control::canbus::Chassis& ChassisCan::get_chassis_status_msg()
{
    std::lock_guard<std::mutex> lk(vehicleStatusMutex);
    return chassisProtoMsg;
}

/**
 * @brief WireCan::setCtlCmd 回调接收控制命令后调用
 * @param cmd
 */
void ChassisCan::setCtlCmd(const control::ControlCommand& cmd) {

  std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
  ctl_cmd = cmd; //获取控制数据

  // ctl_can将计时器清0
  // control_heart_cnt = 0.0f;
  //调试模式赋值
  // DiagTestAssignment();
}

/**
 * @brief ChassisCan::SendCmdProc,发送控制命令线程函数
*/
void ChassisCan::SendCmdProc() {
    (VCU_heartbeat_cnt++ > 200) ? VCU_Life1 = false : VCU_Life1 = true;
    SendCmdFunc();
}

/**
 * @brief ChassisCan::Initialize,msg中二阶构造完成后显式调用
 */
void ChassisCan::Initialize() {
  std::thread can_send_thread([&]() {
    //ProcessScheduler::setThreadPriority(SCHED_FIFO, 95);
    //LOG_Channel_Debug("canlog") << "[ChassisCan]--->CanInit--->Creat can_send_thread";
    uint64_t beginTime = 0;
    uint64_t endTime = 0;
    while (true)
    {
        beginTime = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch()).count();
        SendCmdProc();
        endTime = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch()).count();
                    auto taskDuration = endTime - beginTime;
        if (taskDuration < PERIODTIME_SEND_THREAD){
            std::chrono::milliseconds sleepDuration(PERIODTIME_SEND_THREAD - taskDuration);
            std::this_thread::sleep_for(sleepDuration);
        } else {
            std::cout << "cansend overtime:" << (int32_t)taskDuration << std::endl;
        }
        //std::cout << "end:" << endTime << ", begin:" << beginTime << std::endl;
        //std::cout << "cansend,taskDuration," << (int32_t)taskDuration << std::endl;
    }
  });
  // 在detach之前设置线程名称
  pthread_setname_np(can_send_thread.native_handle(), "can_send_thread");

  can_send_thread.detach();

  /****************************can接收线程****************************/
  std::thread recvThread([&]() {
    std::cout << "can1_recv_thread start." << std::endl;
    StartRecv();
  });
  // 在detach之前设置线程名称
  pthread_setname_np(recvThread.native_handle(), "can1_recv_thread");

  recvThread.detach();
  /****************************can接收线程****************************/
}

void ChassisCan::StartRecv() {
  struct Canframe oneframe;
  int nbytes;
  // 检查文件描述符有效性
  while (true) {
    oneframe = {0};

    nbytes = read(fd_can, &oneframe.frame, sizeof(oneframe.frame));

    if (nbytes < 0) {
      //LOG_Channel_Error("canlog")<<"can raw socket read";
      //std::cout << "can raw socket read error." << std::endl;
      perror("CAN read error");  // 更详细的错误信息
      std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 避免忙等待
      continue;
    }

    /* paranoid check ... */
    if (nbytes < (int)(sizeof(oneframe.frame))) {
      //LOG_Channel_Warn("canlog")
      //    << "read: incomplete CAN frame nbytes < sizeof(struct can_frame)";
      std::cout << "read: incomplete CAN error." << std::endl;
      continue;
    }

    ioctl(fd_can, SIOCGSTAMP_NEW, &oneframe.timestamp);

    HandleRecvData(&oneframe);
  }
}

/**
 * @brief WireCan::HandleRecvData
 * @param recvCanFrame
 */
void ChassisCan::HandleRecvData(struct Canframe *recvCanFrame) {
  //if(!CanA::Instance()->flag_can_adaptor_init_complete){return;}
  recvCanFrame->frame.can_id &= CAN_EFF_MASK;
  callHandleWireCanCmdFunc(recvCanFrame);
}

/**
 * @brief WireCan::callHandleWireCanCmdFunc
 * @param recvCanFrame
 */
void ChassisCan::callHandleWireCanCmdFunc(struct Canframe *recvCanFrame) {
  try {
    if((handleChassisCanFuncMap.find(recvCanFrame->frame.can_id) == handleChassisCanFuncMap.end())
      || std::isnan(recvCanFrame->frame.data[0]) || std::isnan(recvCanFrame->frame.data[1])
      || std::isnan(recvCanFrame->frame.data[2]) || std::isnan(recvCanFrame->frame.data[3])
      || std::isnan(recvCanFrame->frame.data[4]) || std::isnan(recvCanFrame->frame.data[5])
      || std::isnan(recvCanFrame->frame.data[6]) || std::isnan(recvCanFrame->frame.data[7])){
      // LOG_Channel_Warn(LOG_CHANNEL_NAME_CAN_ADAPTOR)
      //     << boost::format(
      //            "[WireCan]--->Not Found handleWireCanCmdFuncPtr By "
      //            "Cmd[0x%08x]") %
      //            recvCanFrame->frame.can_id;
      return;
    } else {
      (this->*handleChassisCanFuncMap[recvCanFrame->frame.can_id])(recvCanFrame);
      //LOG_Channel_Info("can") << boost::format("0x%08x ") % recvCanFrame->frame.can_id << boost::log::dump(recvCanFrame->frame.data, 8);
    }
  } catch (std::exception &e) {
    //LOG_Channel_Error("canlog")<<"control : Proc function error!\n"<<e.what();
    std::cout<<"callHandleWireCanCmdFunc function error!"<<std::endl;
    std::cout << e.what() << std::endl;
    std::terminate();
  }
}
}  // namespace cannode
