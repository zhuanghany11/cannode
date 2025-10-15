/**
 * *****************************************************************************
 * @file        send_control_cmd.cc
 * @brief       测试节点，用于发送控制指令到cannode进行闭环测试
 * @author      Generated for AutoWheelLoader
 * @date        2025-10-15
 * @copyright   Copyright (c) 2025
 * *****************************************************************************
 */

#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sa_msgs/msg/proto_adapter.hpp"
#include "../protobuf/out/control_msgs/control_cmd.pb.h"
#include "../protobuf/out/common_msgs/basic_msgs/header.pb.h"

using namespace std::chrono_literals;

namespace cannode {

class ControlCommandPublisher : public rclcpp::Node
{
public:
  ControlCommandPublisher()
  : Node("send_control_cmd")
  {
    // 声明参数
    this->declare_parameter<double>("speed", 0.0);           // 目标速度 (m/s)
    this->declare_parameter<double>("arm_angle", 0.0);       // 大臂目标角度 (度)
    this->declare_parameter<double>("shovel_angle", 0.0);    // 铲斗目标角度 (度)
    this->declare_parameter<double>("steering_target", 0.0); // 转向目标角度 (度或百分比)
    this->declare_parameter<double>("acceleration", 0.0);    // 加速度 (m/s^2)
    this->declare_parameter<double>("throttle", 0.0);        // 油门 (0-100)
    this->declare_parameter<double>("brake", 0.0);           // 制动 (0-100)
    this->declare_parameter<bool>("arm_enable", true);       // 大臂使能
    this->declare_parameter<bool>("shovel_enable", true);    // 铲斗使能
    this->declare_parameter<bool>("estop", false);           // 急停
    this->declare_parameter<double>("publish_rate", 10.0);   // 发布频率 (Hz)

    // 创建发布者
    publisher_ = this->create_publisher<sa_msgs::msg::ProtoAdapter>("/vehicle_command", 10);

    // 获取发布频率并创建定时器
    double publish_rate = this->get_parameter("publish_rate").as_double();
    int period_ms = static_cast<int>(1000.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ControlCommandPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "控制指令发送节点已启动");
    print_parameters();
  }

private:
  void print_parameters()
  {
    RCLCPP_INFO(this->get_logger(), "=================================");
    RCLCPP_INFO(this->get_logger(), "控制参数配置:");
    RCLCPP_INFO(this->get_logger(), "  速度 (speed): %.2f m/s", 
                this->get_parameter("speed").as_double());
    RCLCPP_INFO(this->get_logger(), "  大臂角度 (arm_angle): %.2f 度", 
                this->get_parameter("arm_angle").as_double());
    RCLCPP_INFO(this->get_logger(), "  铲斗角度 (shovel_angle): %.2f 度", 
                this->get_parameter("shovel_angle").as_double());
    RCLCPP_INFO(this->get_logger(), "  转向角度 (steering_target): %.2f", 
                this->get_parameter("steering_target").as_double());
    RCLCPP_INFO(this->get_logger(), "  加速度 (acceleration): %.2f m/s^2", 
                this->get_parameter("acceleration").as_double());
    RCLCPP_INFO(this->get_logger(), "  油门 (throttle): %.2f %%", 
                this->get_parameter("throttle").as_double());
    RCLCPP_INFO(this->get_logger(), "  制动 (brake): %.2f %%", 
                this->get_parameter("brake").as_double());
    RCLCPP_INFO(this->get_logger(), "  大臂使能 (arm_enable): %s", 
                this->get_parameter("arm_enable").as_bool() ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  铲斗使能 (shovel_enable): %s", 
                this->get_parameter("shovel_enable").as_bool() ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  急停 (estop): %s", 
                this->get_parameter("estop").as_bool() ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  发布频率 (publish_rate): %.1f Hz", 
                this->get_parameter("publish_rate").as_double());
    RCLCPP_INFO(this->get_logger(), "=================================");
  }

  void timer_callback()
  {
    // 获取当前参数值
    double speed = this->get_parameter("speed").as_double();
    double arm_angle = this->get_parameter("arm_angle").as_double();
    double shovel_angle = this->get_parameter("shovel_angle").as_double();
    double steering_target = this->get_parameter("steering_target").as_double();
    double acceleration = this->get_parameter("acceleration").as_double();
    double throttle = this->get_parameter("throttle").as_double();
    double brake = this->get_parameter("brake").as_double();
    bool arm_enable = this->get_parameter("arm_enable").as_bool();
    bool shovel_enable = this->get_parameter("shovel_enable").as_bool();
    bool estop = this->get_parameter("estop").as_bool();

    // 创建 ControlCommand protobuf 消息
    control::ControlCommand cmd;
    
    // 设置 header
    auto* header = cmd.mutable_header();
    auto now = this->get_clock()->now();
    header->set_timestamp_sec(now.seconds());
    header->set_sequence_num(count_++);
    header->set_frame_id("vehicle_command");

    // 设置控制参数
    cmd.set_speed(speed);                       // 目标速度
    cmd.set_arm_angle(arm_angle);               // 大臂角度
    cmd.set_shovel_angle(shovel_angle);         // 铲斗角度
    cmd.set_steering_target(steering_target);   // 转向目标
    cmd.set_acceleration(acceleration);         // 加速度
    cmd.set_throttle(throttle);                 // 油门
    cmd.set_brake(brake);                       // 制动
    cmd.set_arm_enable(arm_enable);             // 大臂使能
    cmd.set_shovel_enable(shovel_enable);       // 铲斗使能
    cmd.set_estop(estop);                       // 急停

    // 设置驾驶模式和档位
    cmd.set_driving_mode(control::canbus::Chassis::COMPLETE_AUTO_DRIVE);
    cmd.set_gear_location(control::canbus::Chassis::GEAR_DRIVE);

    // 序列化 protobuf 消息
    std::string serialized_data;
    if (!cmd.SerializeToString(&serialized_data)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to serialize protobuf message");
      return;
    }

    // 创建 ROS2 消息
    auto message = sa_msgs::msg::ProtoAdapter();
    message.pb.assign(serialized_data.begin(), serialized_data.end());

    // 发布消息
    publisher_->publish(message);

    // 每隔一段时间打印一次日志（避免日志过多）
    if (count_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "发送控制指令 #%ld - 速度: %.2f m/s, 大臂: %.2f°, 铲斗: %.2f°, 转向: %.2f", 
                  count_, speed, arm_angle, shovel_angle, steering_target);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sa_msgs::msg::ProtoAdapter>::SharedPtr publisher_;
  size_t count_ = 0;
};

}  // namespace cannode

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cannode::ControlCommandPublisher>();
  
  RCLCPP_INFO(node->get_logger(), "开始发送控制指令...");
  RCLCPP_INFO(node->get_logger(), "按 Ctrl+C 停止");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

