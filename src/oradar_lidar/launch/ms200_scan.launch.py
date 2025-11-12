#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
parameters=[
    {'device_model': 'MS200'},
    {'frame_id': 'laser_frame'},
    {'scan_topic': 'MS200/scan'},
    {'port_name': '/dev/ttyAMA1 '},
    {'baudrate': 230400},
    {'angle_min': 0.0},
    {'angle_max': 360.0},
    {'range_min': 0.05},
    {'range_max': 20.0},
    {'clockwise': False},
    {'motor_speed': 10}
]
'''

def generate_launch_description():
    # LiDAR publisher node
    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'laser_frame'},
            {'scan_topic': '/scan'},
            {'port_name': '/dev/ttyAMA1'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.10},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10}
        ]
    )

    # Define LaunchDescription and add actions
    return LaunchDescription([
        ordlidar_node,
    ])
