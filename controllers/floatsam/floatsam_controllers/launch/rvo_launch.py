from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_rvo_package',
            executable='rvo_manager_node',
            name='rvo_manager',
            parameters=[{
                'robot_name': 'floatsam_usv_0',
                'time_horizon': 2.0,                # [s]
                'safety_margin': 5.0,               # [m]
                'max_speed': 3.0,                   # [m/s]
                'update_rate': 20.0,                # [Hz]
                'num_robot': 3                      #integer
            }],
        )
    ])