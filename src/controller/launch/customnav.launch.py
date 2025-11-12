from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', 'nav2_planner:=debug']
    )


    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['planner_server']
        }]
    )

    goal_to_plan = Node(
        package='controller', 
        executable='goal2plan',
        name='goal2plan',
        output='screen'
    )

    return LaunchDescription([
        planner_server,
        lifecycle_manager,
        goal_to_plan
    ])
