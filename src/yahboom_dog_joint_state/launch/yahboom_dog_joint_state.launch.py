import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro



def generate_launch_description():
    imunodeNode = Node(
        package="yahboom_dog_joint_state",
        executable="imunode",
        name="imunode",
        output="screen")
        

    yahboom_dog_joint_state_cmd = Node(
            package="yahboom_dog_joint_state",
            executable="yahboom_dog_joint_state",
            name="yahboom_dog_joint_state_publisher",
            output="screen")


    odometry = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="yahboom_dog_joint_state",
                executable="odometry",
                name="odometry",
                output="screen")
        ]
    )

    return LaunchDescription([
        imunodeNode,
        yahboom_dog_joint_state_cmd,
        odometry,
        ])

            

