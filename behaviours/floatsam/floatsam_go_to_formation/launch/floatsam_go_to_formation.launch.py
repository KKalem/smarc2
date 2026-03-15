from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """
    Launch the go_to_formation behavior tree action server.

    Each floatsam USV is brought up with floatsam_bringup.sh <id>, giving names
    floatsam_usv_0, floatsam_usv_1, ... floatsam_usv_(num_robots-1).
    Pass robot_name as the full name of the robot running this algorithm (e.g. floatsam_usv_0).
    The server derives the base name (floatsam_usv) from it to enumerate the whole fleet.
    """

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='floatsam_usv_0',
        description='Full name of the robot running this algorithm (e.g. floatsam_usv_0)'
    )

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='3',
        description='Total number of floatsam USVs in the fleet (IDs will be 0 .. num_robots-1)'
    )

    collision_radius_arg = DeclareLaunchArgument(
        'collision_radius',
        default_value='2.0',
        description='Minimum distance between robots to avoid collisions (in meters)'
    )

    max_num_collisions_arg = DeclareLaunchArgument(
        'max_num_collisions',
        default_value='4',
        description='Maximum number of collisions before giving up and aborting the action'
    )

    waypoints_step_size_arg = DeclareLaunchArgument(
        'waypoints_step_size',
        default_value='5.0',
        description='Distance between waypoints in the generated path (in meters)'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='2.0',
        description='Maximum speed [m/s] used by the furthest robot; others scale to arrive at the same time'
    )

    last_point_tolerance_move_path_arg = DeclareLaunchArgument(
        'last_point_tolerance_move_path',
        default_value='0.5',
        description='Tolerance to consider the last point of the path reached (in meters)'
    )

    robot_name = LaunchConfiguration('robot_name')
    num_robots = LaunchConfiguration('num_robots')
    collision_radius = LaunchConfiguration('collision_radius')
    max_num_collisions = LaunchConfiguration('max_num_collisions')
    waypoints_step_size = LaunchConfiguration('waypoints_step_size')
    max_velocity = LaunchConfiguration('max_velocity')
    last_point_tolerance_move_path = LaunchConfiguration('last_point_tolerance_move_path')

    node = GroupAction([
        PushRosNamespace(robot_name),
        Node(
            package='floatsam_go_to_formation',
            executable='floatsam_go_to_formation_action_server',
            name='floatsam_go_to_formation_action_server',
            parameters=[{
                'robot_name': robot_name,
                'num_robots': num_robots,
                'collision_radius': collision_radius,
                'max_num_collisions': max_num_collisions,
                'waypoints_step_size': waypoints_step_size,
                'max_velocity': max_velocity,
                'last_point_tolerance_move_path': last_point_tolerance_move_path
            }],
            output='screen'
        )
    ])

    return LaunchDescription([
        robot_name_arg,
        num_robots_arg,
        collision_radius_arg,
        max_num_collisions_arg,
        waypoints_step_size_arg,
        max_velocity_arg,
        last_point_tolerance_move_path_arg,
        node
    ])


