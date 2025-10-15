from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    log_dir = os.getenv('ROS_LOG_DIR')
    if log_dir is None:
        log_dir = os.path.join(get_package_prefix('cannode'), 'log')
    # print("Task Management Configuration File:", config_file)
    return LaunchDescription([
        # 启动C++版本的节点
        Node(
            package='cannode',
            executable='cannode',
            name='cannode',
            output='screen',
        ),

    ]) 