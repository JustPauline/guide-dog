from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    laser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("oradar_lidar"),
            "launch",
            "ms200_scan.launch.py",
        )),
    )

    yahboom_dog_joint_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("yahboom_dog_joint_state"),
            "yahboom_dog_joint_state.launch.py",
        )),
    )

    speech = Node(
        package="voice_command",
        executable="voice_command",
        name="voice_command",
        output="screen"
    )

    yahboom_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("yahboom_base"),
            "yahboom_base.launch.py",
        )),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('yahboom_base'),
            'config',
            'ekf.yaml'
        )],
    )


    slam_toolbox_node_delayed = TimerAction(
        period=10.0,  # Delay SLAM by 5 seconds to allow IMU/odom to initialize
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[os.path.join(
                    get_package_share_directory('bringup'),
                    'config',
                    'slam_params.yaml'
                )],

            )
        ]
    )

    controllaunch = TimerAction(
        period=25.0,
        actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("controller"),
            "customnav.launch.py",
        )),
        )
        ]
    )

    controllernode = TimerAction(
        period=20.0,  
        actions=[
            Node(
                package='controller',
                executable='controller',
                name='controller',
                output='screen',
                parameters=[{
                    'lookahead_distance': 0.6,
                    'linear_gain': 0.8,
                    'angular_gain': 1.5,
                    'goal_tolerance': 0.25
                }]
            )
        ]
    )

    
    xgo_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("yahboom_description"),
            "yahboom_urdf.launch.py",
        )),
    )


    startcam = Node(
        package='usb_cam', 
        executable='usb_cam_node_exe', 
        name='camera_driver',
    )

    chairfinder = Node(
        package='chairdetection',
        executable='chair_detector',
        name='chair_detector',
        output='screen',
    )


    return LaunchDescription([
        yahboom_dog_joint_state,
        yahboom_base,
        laser,
        ekf_node,
        slam_toolbox_node_delayed,
        controllaunch,
        xgo_description,
        controllernode,
        speech,
        startcam,
        chairfinder,
    ])
