#!/usr/bin/env python3
"""
Launch file for sending control commands to cannode
用于向cannode发送控制指令的启动文件

Usage examples:
1. 使用默认参数启动:
   ros2 launch cannode send_control_cmd.launch.py

2. 设置速度为0.5 m/s:
   ros2 launch cannode send_control_cmd.launch.py speed:=0.5

3. 设置大臂角度为30度:
   ros2 launch cannode send_control_cmd.launch.py arm_angle:=30.0

4. 组合多个参数:
   ros2 launch cannode send_control_cmd.launch.py speed:=0.3 arm_angle:=25.0 shovel_angle:=15.0

5. 设置发布频率为20Hz:
   ros2 launch cannode send_control_cmd.launch.py publish_rate:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    declare_speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.0',
        description='目标速度 (m/s)'
    )
    
    declare_arm_angle_arg = DeclareLaunchArgument(
        'arm_angle',
        default_value='0.0',
        description='大臂目标角度 (度)'
    )
    
    declare_shovel_angle_arg = DeclareLaunchArgument(
        'shovel_angle',
        default_value='0.0',
        description='铲斗目标角度 (度)'
    )
    
    declare_steering_target_arg = DeclareLaunchArgument(
        'steering_target',
        default_value='0.0',
        description='转向目标角度 (度或百分比)'
    )
    
    declare_acceleration_arg = DeclareLaunchArgument(
        'acceleration',
        default_value='0.0',
        description='目标加速度 (m/s^2)'
    )
    
    declare_throttle_arg = DeclareLaunchArgument(
        'throttle',
        default_value='0.0',
        description='油门开度 (0-100)'
    )
    
    declare_brake_arg = DeclareLaunchArgument(
        'brake',
        default_value='0.0',
        description='制动开度 (0-100)'
    )
    
    declare_arm_enable_arg = DeclareLaunchArgument(
        'arm_enable',
        default_value='true',
        description='大臂控制使能 (true/false)'
    )
    
    declare_shovel_enable_arg = DeclareLaunchArgument(
        'shovel_enable',
        default_value='true',
        description='铲斗控制使能 (true/false)'
    )
    
    declare_estop_arg = DeclareLaunchArgument(
        'estop',
        default_value='false',
        description='急停信号 (true/false)'
    )
    
    declare_publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='控制指令发布频率 (Hz)'
    )

    # 创建节点
    send_control_cmd_node = Node(
        package='cannode',
        executable='send_control_cmd',
        name='send_control_cmd',
        output='screen',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'arm_angle': LaunchConfiguration('arm_angle'),
            'shovel_angle': LaunchConfiguration('shovel_angle'),
            'steering_target': LaunchConfiguration('steering_target'),
            'acceleration': LaunchConfiguration('acceleration'),
            'throttle': LaunchConfiguration('throttle'),
            'brake': LaunchConfiguration('brake'),
            'arm_enable': LaunchConfiguration('arm_enable'),
            'shovel_enable': LaunchConfiguration('shovel_enable'),
            'estop': LaunchConfiguration('estop'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    return LaunchDescription([
        # 声明所有参数
        declare_speed_arg,
        declare_arm_angle_arg,
        declare_shovel_angle_arg,
        declare_steering_target_arg,
        declare_acceleration_arg,
        declare_throttle_arg,
        declare_brake_arg,
        declare_arm_enable_arg,
        declare_shovel_enable_arg,
        declare_estop_arg,
        declare_publish_rate_arg,
        # 启动节点
        send_control_cmd_node,
    ])

