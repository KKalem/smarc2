#!/usr/bin/env python3
"""
Launch file for LoLo SMaRC Topics Publisher
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulator topics (true) or real hardware topics (false)'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='lolo',
        description='Name of the robot for namespacing'
    )
    
    # Create node
    smarc_topics_publisher_node = Node(
        package='lolo_topic_bridge',
        executable='smarc_topics_publisher',
        name='lolo_smarc_topics_publisher',
        output='screen',
        parameters=[{
            'use_sim': LaunchConfiguration('use_sim'),
            'robot_name': LaunchConfiguration('robot_name'),
        }],
        remappings=[
            # Add any necessary topic remappings here
        ]
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,
        smarc_topics_publisher_node,
    ])
