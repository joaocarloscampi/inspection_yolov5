from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='inspection_yolov5', 
             executable='ros_detect.py', 
             name='detection',
             output='screen',
             emulate_tty=True,
             parameters=[
                 {'camera_topic': '/zedm/zed_node/left/image_rect_color'}, 
                 {'bounding_box_topic': '/yolov5_ros2/bounding_boxes'}
             ]),
    ])
