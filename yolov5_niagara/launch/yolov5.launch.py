from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='yolov5_niagara',
            executable='ros_recognition_yolo.py',
            output='screen',
            prefix='bash -c',
            cwd='/home/genis/ros2_ws/src/niagara_vision_processing/yolov5_niagara/scripts'

        )
    ])
