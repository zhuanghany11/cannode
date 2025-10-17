
/**
 * *****************************************************************************
 * @file        msg_manager.h
 * @brief
 * @author      andy
 * @date        2025-09-15
 * @copyright Copyright (c) 2025
 * *****************************************************************************
 */
#ifndef MSG_MANAGER_H
#define MSG_MANAGER_H

#include <cstdint>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "sa_msgs/msg/proto_adapter.hpp"
#include "chassis_driver.h"
#include "vehicle_protocol/shantui.h"
#include "vehicle_protocol/changan.h"
namespace cannode {

#define CAN_CFG_FILE "config/can_adapter.yaml"
#define PERIODTIME 50  // ms
#define XUGONG 1
#define CHANGAN 2
#define SHANTUI 3

class MsgManager : public rclcpp::Node {
 public:
  /*
   * @brief constructor function
   */
  MsgManager();

  /*
   * @brief destructor function
   */
  ~MsgManager();

  /**
   * @brief Initialize msg manager
   * @param
   * @return
   */
  bool init(void);
  /**
   * @brief Main processing function for publishing ROS messages
   * @param
   * @return
   */
  void msgProc();
  /**
   * @brief Process control command information #从控制获取protobuf消息
   * @param
   * @return
   */
  void procControlImplement(const sa_msgs::msg::ProtoAdapter::SharedPtr msg) const;
  /**
   * @brief Generate a wire control information buffer # 生成线控信息反馈给控制
   * @param
   * @return
   */
  void PublishChassisStatus(); // 发布底盘状态消息

 private:
  // ROS2 发布订阅句柄指针
  rclcpp::Publisher<sa_msgs::msg::ProtoAdapter>::SharedPtr chassis_status_pub;
  rclcpp::Subscription<sa_msgs::msg::ProtoAdapter>::SharedPtr chassis_cmd_sub;
  //构建socket can节点实例所需参数，及cannode内部使用参数
  int32_t vcu_type{1};
  std::string chassis_can_port = "can0";
  int64_t chassis_can_baud{1000000};
  std::shared_ptr<ChassisCan> chassisCanHandler;

 private:
  bool cannodeLoadPara(std::string path);
  void cannodeCfg();
  void cannodeCreateInstance();
};

}  // namespace can

#endif // MSG_MANAGER_H
