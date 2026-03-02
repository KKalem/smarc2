import numpy as np 
from rclpy.node import Node

from .floatsam_common import FloatSam
from floatsam_interfaces.srv import GetSafeVelocity #to be added 
from tf2_geometry_msgs import do_transform_pose_stamped

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint




class RVOservice(Node):
    def __init__(self):
        super().__init__("RVO_service_node")
        self.logger = self.get_logger()
        self.srv = self.create_service(GetSafeVelocity, 'get_safe_velocity', self.compute_safe_velocity_callback)
        self.get_logger().info('RVO Safe Velocity Service is ready.')
        self.get_node_parametrs()
        self._floatsam = FloatSam(self, self.this_robot_name)

        self._odom_subscribers = {}
        self._robot_positions = {}
        self._robot_velocities = {}
        self._odometry_subscriptions()
    
    def compute_safe_velocity_callback(self):
        pass

    def declare_node_parameters(self):
        """Declare all configurable parameters for PIDs and mixer"""
        self.declare_parameter("time_horizon", 2.0)
        self.declare_parameter("safety_margin", 5.0)
        self.declare_parameter("max_speed", 3.0)
        self.declare_parameter("update_rate", 0.0)
        self.declare_parameter("num_robot", 3)

    def get_node_parametrs(self):
        self.this_robot_name = str(self.get_parameter("robot_name").value)
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.safety_margin = float(self.get_parameter("safety_margin").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.time_horizon = float(self.get_parameter("time_horizon").value)
        self.num_robot = int(self.get_parameter("num_robot").value)
        self.robot_ids = range(self.num_robot)
        self.robot_base_name = '_'.join(self.this_robot_name.split('_')[:-1])
    
    def _odometry_subscriptions(self):
        
        for robot_id in self.robot_ids:
            odom_topic = f'/{self.robot_base_name}_{robot_id}/smarc/odom'
            
            subscriber = self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                10
            )
            
            self._odom_subscribers[robot_id] = subscriber
            self.get_logger().info(f'Subscribed to {odom_topic}')
    
    def _odom_callback(self, msg: Odometry, robot_id: int):
        """
        Update robot position in blackboard when odometry is received.
        """
        pose_in_odom = PoseStamped()
        pose_in_odom.header = msg.header
        pose_in_odom.pose = msg.pose.pose
        velocity_in_odom = msg.twist.twist.linear

        try:
            pose_in_map = do_transform_pose_stamped(
                pose_in_odom, self._floatsam._odom_to_map_tf
            )
        except Exception as e:
            self.get_logger().error(
                f"Error transforming odom to map for robot {robot_id}: {e}"
            )
            return
        
        self._robot_positions[f'{self.robot_base_name}_{robot_id}'] = pose_in_map
