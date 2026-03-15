from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='floatsam_usv', description='Robot namespace')

    robot_name = LaunchConfiguration('robot_name')

    node = Node(
            package='floatsam_controllers',
            executable='rvo_service_node',
            name='rvo_service_node',
            namespace=robot_name,
            parameters=[{
                'robot_name': 'floatsam_usv_0',
                'time_horizon': 1.0,                # [s]
                'safety_margin': 1.0,               # [m]
                'max_speed': 2.0,                   # [m/s]
                'update_rate': 20.0,                # [Hz]
                'num_robot': 3                      #integer
            }],
        )
    
    return LaunchDescription([
    robot_name_arg,
    node
    ])