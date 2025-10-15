
#include "src/msg_manager.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建底盘驱动节点
    auto msg_node = std::make_shared<cannode::MsgManager>();
    msg_node->init();

    // 运行节点
    rclcpp::spin(msg_node);
    rclcpp::shutdown();
    
    return 0;
}
