import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Paths to URDF and RViz config
    default_xacro_model_path = os.path.join(
        get_package_share_directory('yahboom_description'), 'urdf', 'yahboom_xgo_rviz.xacro')
    default_rviz_config_path = os.path.join(
        get_package_share_directory('champ_config'), 'config/rviz2', 'xgo.rviz')

    # Launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Declare launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_xacro_model_path_cmd = DeclareLaunchArgument(
        name='xacro_model',
        default_value=default_xacro_model_path,
        description='Absolute path to robot xacro file')

    # Process xacro to robot_description
    robot_description_config = xacro.process_file(default_xacro_model_path)
    robot_desc = robot_description_config.toxml()

    # Start robot_state_publisher
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )

    # Start RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        declare_rviz_config_file_cmd,
        declare_xacro_model_path_cmd,
        start_robot_state_publisher_cmd,
        start_rviz_cmd,
    ])
