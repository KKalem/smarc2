from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='floatsam_usv', description='Robot namespace')

    robot_name = LaunchConfiguration('robot_name')

    node = Node(
        package='floatsam_move_to',
        executable='floatsam_move_to_action_server',
        name='floatsam_move_to_action_server',
        namespace=robot_name,
        parameters=[{'robot_name': robot_name}],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        node
    ])
