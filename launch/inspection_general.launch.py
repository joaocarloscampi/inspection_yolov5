import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	# Diretórios e arquivos
	bringup_dir = get_package_share_directory('inspection_yolov5')

	# Argumentos de lançamento
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_sim_time_arg = DeclareLaunchArgument('use_sim_time', 
											default_value='false', 
											description='Use simulation (Gazebo) clock if true')

	robot_config_file = os.path.join(bringup_dir, 'config/inspection_parameters.yaml')
	with open(robot_config_file, 'r') as file:
		robotParams = yaml.safe_load(file)
	print(robotParams)
	
	camera_topic = robotParams['camera_topic']
	bouding_box_topic = robotParams['bouding_box_topic']
	visualization_detection = robotParams['visualization_detection']
	publish_detection_bb = robotParams['publish_detection_bb']
	publish_detection_bb_topic = robotParams['publish_detection_bb_topic']
	
	print("Topico da camera: ", camera_topic)
	print("Topico da bouding boxes: ", bouding_box_topic)
	print("Visualizacao da deteccao: ", visualization_detection)
	print("Publicar imagem com deteccao: ", publish_detection_bb)
	print("Topico imagem com deteccao: ", publish_detection_bb_topic)
	
	list_launch_description = []
	
	# Launches
	detection = Node(package='inspection_yolov5', 
             executable='ros_detect.py', 
             name='detection',
             output='screen',
             emulate_tty=True,
             parameters=[
                 {'camera_topic': camera_topic}, 
                 {'bounding_box_topic': bouding_box_topic},
				 {'visualization_detection': visualization_detection},
				 {'publish_detection_bb': publish_detection_bb},
				 {'publish_detection_bb_topic': publish_detection_bb_topic}
             ])
	list_launch_description.append(detection)

	return LaunchDescription(list_launch_description)