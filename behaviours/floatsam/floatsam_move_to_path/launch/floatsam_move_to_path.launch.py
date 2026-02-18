from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='floatsam_usv', description='Robot namespace')

    robot_name = LaunchConfiguration('robot_name')

    node = Node(
        package='floatsam_move_to_path',
        executable='floatsam_move_to_path_action_server',
        name='floatsam_move_to_path_action_server',
        namespace=robot_name,
        parameters=[{
            'robot_name': robot_name,
            "yaw_p_gain": 0.3,
            "yaw_i_gain": 0.0,
            "yaw_d_gain": 0.1,  
            "yaw_threshold": 0.5,
            
            "yawrate_p_gain": 300.0,
            "yawrate_i_gain": 0.0,
            "yawrate_d_gain": 30.0,  
            
            "velocity_p_gain": 500.0,
            "velocity_i_gain": 10.0,
            "velocity_d_gain": 0.0
            }],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        node
    ])