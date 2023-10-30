from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='yolov5_cpp',
            executable='yolov5_node',
            name='yolov5_node',
            output='screen',
        )
    ])