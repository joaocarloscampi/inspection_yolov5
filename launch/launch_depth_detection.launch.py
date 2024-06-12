from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='inspection_yolov5', executable='depth_bounding_box.py', output='screen'),
    ])
