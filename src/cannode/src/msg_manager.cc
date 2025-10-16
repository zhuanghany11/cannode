
#include "msg_manager.h"
#include "msg_manager.h"
#include <chrono>
#include <thread>
#include <cstdlib>
#include <pthread.h>
//#include "control/control_command.pb.h"
#include "control_msgs/control_cmd.pb.h"
//#include "control/canbus/chassis.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"

namespace cannode {

MsgManager::MsgManager(): Node("chassis_node") {
  // 创建 CAN node
  //cannodeLoadPara(std::string(CAN_CFG_FILE));
  cannodeCfg();
  cannodeCreateInstance();

  // 创建底盘状态发布者
  chassis_status_pub = this->create_publisher<sa_msgs::msg::ProtoAdapter>("/SaVehicleReportV2", 10);
  chassis_cmd_sub = this->create_subscription<sa_msgs::msg::ProtoAdapter>(
      "/vehicle_command", 10, std::bind(&MsgManager::procControlImplement, this, std::placeholders::_1));

  // 预留：节点参数用于PID与零点标定配置（未来可通过 declare_parameter/get_parameter 下发到解析器）
}

MsgManager::~MsgManager() {
  if (chassisCanHandler) {
    chassisCanHandler.reset();
  }
}

bool MsgManager::init(void) {
    std::thread cannode_thread([&](){
    //ProcessScheduler::setThreadPriority(SCHED_FIFO, 95);
    uint64_t beginTime = 0;
    uint64_t endTime = 0;
    while (true)
    {
        beginTime = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch()).count();
        msgProc();
        endTime = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch()).count();
        auto taskDuration = endTime - beginTime;
        if (taskDuration < PERIODTIME) {
            std::chrono::milliseconds sleepDuration(PERIODTIME - taskDuration);
            std::this_thread::sleep_for(sleepDuration);
        } else {
            std::cout << "cannode overtime:" << (int32_t)taskDuration << std::endl;
        }
        //std::cout << "cannode, taskDuration," << (int32_t)taskDuration << std::endl;
    }
    });
    // 在detach之前设置线程名称
    pthread_setname_np(cannode_thread.native_handle(), "cannode_thread");
    cannode_thread.detach();
  return true;
}

void MsgManager::msgProc() {
    PublishChassisStatus();
}

// 接收处理控制命令信息
void MsgManager::procControlImplement(const sa_msgs::msg::ProtoAdapter::SharedPtr msg) const {
    // 从 ROS2 消息中提取数据
    std::string serialized_data(msg->pb.begin(), msg->pb.end());
    std::cout << "Received len: " << msg->pb.size() << std::endl;
    std::cout << "Received: len = " << serialized_data.size() << std::endl;
    std::cout << "rec data (hex): ";
    for (char c : serialized_data) {
      std::cout << std::hex << (static_cast<int>(c) & 0xFF) << " ";
    }
    std::cout << std::endl;
    // 使用 ParseFromString 反序列化
    control::ControlCommand protobuf_cmd;

    // both usage is ok
    //if(protobuf_msg.ParseFromArray(msg->data.data(), msg->data.size())){
    if (protobuf_cmd.ParseFromString(serialized_data)) {
    //   RCLCPP_INFO(this->get_logger(), "Received: name='%s', id=%d, value=%.2f", 
    //             protobuf_cmd.name().c_str(), protobuf_cmd.id(), protobuf_cmd.value());
    } else {
    //  RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf message");
    }

    chassisCanHandler->setCtlCmd(protobuf_cmd);
  }
/*
flatbuffers::DetachedBuffer MsgManager::genWireControlVehicleFaultOutput(void) {
  // Generate a wire control information buffer
  if (chassisCanHandler) {
    return chassisCanHandler->GenWireControlVehicleFaultOutput();
  }
  return flatbuffers::DetachedBuffer();
}

flatbuffers::DetachedBuffer MsgManager::genWireControlVehicleStatusOutput(void) {
  // Generate a wire control information buffer
  if (chassisCanHandler) {
    return chassisCanHandler->GenWireControlVehicleStatusOutput();
  }
  return flatbuffers::DetachedBuffer();
}
*/
void MsgManager::PublishChassisStatus()
{
    auto message = sa_msgs::msg::ProtoAdapter(); //ros数据msg
    control::canbus::Chassis chassisptmsg;
    std::string str;

    chassisptmsg = chassisCanHandler->get_chassis_status_msg(); // 获取最新的底盘状态消息
    
    /*debug*/
    chassisptmsg.set_gear_location(control::canbus::Chassis::GEAR_DRIVE);
    chassisptmsg.set_driving_mode(control::canbus::Chassis::COMPLETE_AUTO_DRIVE);

    //double timestamp_sec =
    //ConvertStampToTimestampPb(vehicle_report_msg->header.stamp.sec, vehicle_report_msg->header.stamp.nanosec);
    // 设置 header 信息
    //protomsg.mutable_header()->set_timestamp_sec(controlcommand.header().timestamp_sec());
    //protomsg.mutable_header()->set_frame_id(controlcommand.header().frame_id());

    // 设置高压状态
    // switch (vehicle_info_.vehicle_mode) {
    // case 0:
    //   chassisptmsg.set_driving_mode(control::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);
    //   break;
    // case 1:
    //   chassisptmsg.set_driving_mode(control::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
    //   break;
    // default:
    //   chassisptmsg.set_driving_mode(control::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);
    //   break;
    // }
    
    // 设置转向模式
    //Chassisptmsg.set_steer_mode(controlcommand.steering_model());
     chassisptmsg.SerializeToString(&str);
     uint32_t len = str.size();
     std::cout << "len: " << len << std::endl;
     message.pb.assign(str.begin(), str.end());

     chassis_status_pub->publish(message);
     RCLCPP_INFO(this->get_logger(), "Publishing: mode=%d", chassisptmsg.driving_mode());
     //std::cout << "publish data." << " mode: "<< chassisptmsg.driving_mode() <<std::endl;
}


bool MsgManager::cannodeLoadPara(std::string path) {
  /*try {
    std::ifstream ifs(path, std::ios_base::binary);
    const auto cfg_file = toml::parse(ifs);

    // find a value with the specified type from a table
    std::string title = toml::find<std::string>(cfg_file, "title");
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR) << "title: " << title;

    // find a value from a table
    vehicle_number =
        toml::find<std::int64_t>(cfg_file, "vehicle", "self", "vehicle_number");
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "vehicle_number: " << vehicle_number;

    // find parameters of PORT table
    const auto &port = toml::find(cfg_file, "CAN");
    chassis_can_port = toml::find<std::string>(port, "vcu_port", 0);
    chassis_can_baud = toml::find<int64_t>(port, "vcu_port", 1);
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "vcu_port:" << chassis_can_port << "|" << chassis_can_baud;

    // find a value from a table
    vcu_type = toml::find<int>(cfg_file, "vehicle", "self", "vcu_type");
    //    vcu_type_ = toml::find<int>(vcuType, "type");
    // std::cout<<"vcu_type : "<<vcu_type_<<std::endl;
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "vcu_type: " << vcu_type;

    return true;
  } catch (...) {
    LOG_Channel_Error(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "can adaptor read cfg file fail, pls check.";
    return false;
  }*/
  return false;
}

void MsgManager::cannodeCfg() {
  std::string cmd;

  cmd = "ifconfig " + chassis_can_port + " down";
  std::cout << cmd << std::endl;
  std::system(cmd.c_str());

  cmd = "ip link set " + chassis_can_port + " up type can bitrate " + std::to_string(chassis_can_baud) + " restart-ms 50";
  std::cout << cmd << std::endl;
  std::system(cmd.c_str());

  cmd = "ifconfig " + chassis_can_port + " up";
  std::cout << cmd << std::endl;
  std::system(cmd.c_str());
}

void MsgManager::cannodeCreateInstance() {
  switch (vcu_type) {
    case XUGONG:
      chassisCanHandler = std::make_shared<ShantuiCANParser>(chassis_can_port);
      break;
    case CHANGAN:
      chassisCanHandler = std::make_shared<ChanganCANParser>(chassis_can_port);
      break;
    case SHANTUI:
      chassisCanHandler = std::make_shared<ShantuiCANParser>(chassis_can_port);
      break;
    default:
      chassisCanHandler = std::make_shared<ShantuiCANParser>(chassis_can_port);
      break;
  }
  // 在对象创建后显式调用初始化
  chassisCanHandler->Initialize();
}

}  // namespace cannode
