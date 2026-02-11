from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='floatsam_usv'
    )

    loiter_server_node = Node(
        package='floatsam_move_to',
        namespace=robot_ns,
        executable='floatsam_loiter_action_server',
        name='floatsam_loiter_action_server',
        parameters=[{
            'robot_name': robot_ns,
            'loiter_tolerance': 5.0,  # meters - loiter circle radius
            'loiter_reposition_tolerance': 0.5,  # meters - strict tolerance for move_to
            'loiter_speed': 1.0,  # m/s - repositioning speed
            'loiter_move_to_speed': 'fast'  # move_to speed: 'slow', 'standard', or 'fast'
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        loiter_server_node
    ])
