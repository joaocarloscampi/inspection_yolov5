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

	robot_config_file = os.path.join(bringup_dir, 'config/robot_parameters.yaml')
	with open(robot_config_file, 'r') as file:
		robotParams = yaml.safe_load(file)
	print(robotParams)
	
	robot = robotParams['robot']
	zed_scan = robotParams['zed_scan']
	zed_camera_model = robotParams['zed_camera_model']
	lidar = robotParams['lidar']
	filter_lidar = robotParams['filter_lidar']
	slam_toolbox = robotParams['slam_toolbox']
	teleop_keyboard = robotParams['teleop_keyboard']
	rviz_enable = robotParams['rviz']
	amcl_localization = robotParams['amcl_localization']
	
	print("Robot: ", robot)
	print("Zed scan enable: ", zed_scan)
	print("Zed model: : ", zed_camera_model)
	print("Lidar enable: ", lidar)
	print("Lidar filter enable: ", filter_lidar)
	print("slam_toolbox enable: ", slam_toolbox)
	
	list_launch_description = []
	
	# Launches
	

	return LaunchDescription(list_launch_description)