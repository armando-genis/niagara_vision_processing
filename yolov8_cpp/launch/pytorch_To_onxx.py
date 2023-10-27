from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # waypoints_path = os.path.join(get_package_share_directory('stanley_node_pkg'),'config','waypoints.yaml')
    
    return LaunchDescription([
        Node(
            package='yolov8_cpp',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen',
        )
    ])