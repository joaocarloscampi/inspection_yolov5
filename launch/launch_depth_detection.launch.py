from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='yolov5_obb', executable='depth_bounding_box.py', output='screen'),
    ])
