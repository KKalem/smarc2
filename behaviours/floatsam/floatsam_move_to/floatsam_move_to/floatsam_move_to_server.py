#!/usr/bin/python

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration

import traceback

from .floatsam_common import FloatSam

#from std_msgs.msg import Float32
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from geometry_msgs.msg import  PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_action_base.gentler_action_server import GentlerActionServer
import time

class MoveToActionFloatSam():
    def __init__(self,
                 node: Node):
        self._node : Node = node
        
        # Get robot name from node parameter
        self._robot_name : str = self._node.get_parameter('robot_name').value
        
        self.MAP_FRAME : str = self._robot_name + '/map'
        self._floatsam = FloatSam(node, self._robot_name)
        
        self._node.get_logger().info(f"FloatSam move_to server initialized for robot: {self._robot_name}")

        self._default_goal_tolerance = 1  
        self._default_speed_threshold = 10  # start slowing down when within 10m of goal 

        # Publishers use FloatsamTopics constants (relative paths get robot namespace)
        self._yaw_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.YAW_SETPOINT, 10)

        self._speed_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, 10)

        # create the gentler action server to expose 'move_to'
        self._as = GentlerActionServer(
            node,
            "move_to",
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=10
        )

        # timer: when the action server starts, print the floatsam position read from odom_gt
        # tries every 0.5s and times out after 5 seconds
        self._initial_pos_deadline = int(self._node.get_clock().now().nanoseconds * 1e-9) + 5
        self._initial_pos_timer = self._node.create_timer(0.5, self._check_initial_position)

    @property
    def now_stamp(self):
        return self._node.get_clock().now().to_msg()
    
    @property
    def now_time(self):
        return self.now_stamp.sec + self.now_stamp.nanosec * 1e-9
    
    def log(self, msg: str):
        self._node.get_logger().info(msg)

    def _check_initial_position(self):
        """Timer callback: print the first floatsam position received from odom_gt (or timeout)."""
        if self._floatsam.floatsam_in_map is not None:
            p = self._floatsam.floatsam_in_map.pose.position
            self._node.get_logger().info(f"Floatsam position from odom_gt: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]")
            try:
                self._initial_pos_timer.cancel()
            except Exception:
                pass
        else:
            now = int(self._node.get_clock().now().nanoseconds * 1e-9)
            if now > self._initial_pos_deadline:
                self._node.get_logger().warning("Timed out waiting for floatsam position from odom_gt")
                try:
                    self._initial_pos_timer.cancel()
                except Exception:
                    pass

    def _on_goal_received(self, goal_request: dict) -> bool:
        
        self._node.get_logger().info(f"Goal request received: {goal_request} piselli")

        try:
            # first transform the latlon goal into UTM
            gp : GeoPoint = GeoPoint()
            gp.latitude = goal_request['waypoint']['latitude']
            gp.longitude = goal_request['waypoint']['longitude']
            
            self._goal_in_map = self._floatsam.convert_geopoint_to_map_pose_stamped(gp)

            self._goal_tolerance = float(goal_request['waypoint']['tolerance']) if 'tolerance' in goal_request else self._default_goal_tolerance
           
            try:
                self._goal_speed = goal_request['speed']
                if self._goal_speed == "standard":
                    self._goal_speed = 2.0  

                elif self._goal_speed == "slow":
                    self._goal_speed = 1.0  

                elif self._goal_speed == "fast":
                    self._goal_speed = 5.0 
                
                else:
                    self._goal_speed = 2.0  

            except:
                self._node.get_logger().info(f" no valid speed")


            #self._goal_speed = goal_request['speed']

            pos = self._goal_in_map.pose.position
            
            self._node.get_logger().info(f"Received goal in map: [{pos.x:.2f},{pos.y:.2f},{pos.z:.2f}], tolerance: {self._goal_tolerance}, speed: {self._goal_speed}")
            
            return True
        
        except:
            self._node.get_logger().error("Failed to parse goal request")
            traceback.print_exc()
            return False


    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info("Cancel requested, stopping...")
        self._goal_in_map = None
        return True

    def _prepare_loop(self) -> None:
        self._distance_remaining = None
        return

    def _loop_inner(self) -> bool|None:
        if self._goal_in_map is None:
            self._node.get_logger().info("No goal set, failing...")
            return False

        if self._goal_tolerance is None:
            self._node.get_logger().info("No goal tolerance set, failing...")
            return False

        if self._floatsam.floatsam_in_map is None:
            self._node.get_logger().info("No floatsam position available yet, waiting...")
            return None
        
        goal_position = np.array([self._goal_in_map.pose.position.x,
                                  self._goal_in_map.pose.position.y])
        
        self_position = np.array([self._floatsam.floatsam_in_map.pose.position.x,
                                  self._floatsam.floatsam_in_map.pose.position.y])

    
        self._node.get_logger().info(f"Current position: [{self_position[0]:.2f}, {self_position[1]:.2f}]")
        self._node.get_logger().info(f"Goal position:    [{goal_position[0]:.2f}, {goal_position[1]:.2f}]")
        
        goal_error = goal_position - self_position
        goal_error_mag = np.linalg.norm(goal_error)
        self._distance_remaining = float(goal_error_mag)

        if self._distance_remaining <= self._goal_tolerance:
            self._node.get_logger().info(f"Reached goal within tolerance {self._goal_tolerance}m")
            return True
        
        if self._distance_remaining <= self._default_speed_threshold:
            # slow down when close to goal
            self._desired_speed = (self._distance_remaining / self._default_speed_threshold) * self._goal_speed
            self._node.get_logger().info(f"Slowing down, new speed: {self._desired_speed:.2f}")
        else:
            self._desired_speed = self._goal_speed
    
        self._node.get_logger().info(f"The desired speed is {self._desired_speed:.2f} m/s")
        #calcuate error heading and speed
        error_heading = float(np.arctan2(goal_error[1], goal_error[0]))
        #error_heading = float(np.pi/2 - error_heading)  # convert to floatsam heading frame ---->>>>>>>>>>>>>
        self._node.get_logger().info(f"The distance remaining is {self._distance_remaining:.2f} m")
        speed = float(self._desired_speed)
        
        yaw_msg = FloatStamped()
        speed_msg = FloatStamped()
        now = self._node.get_clock().now().to_msg()
        yaw_msg.header.stamp = now
        yaw_msg.data = error_heading
        speed_msg.header.stamp = now
        speed_msg.data = speed
        self._yaw_reference_publisher.publish(yaw_msg)
        self._speed_reference_publisher.publish(speed_msg)

        return None

    def _give_feedback(self) -> str:
        if self._distance_remaining is not None:
            return f"Distance remaining: {self._distance_remaining:.2f} (tolerance: {self._goal_tolerance:.2f}m)"
        else:
            return "No distance remaining info"
        

def main(args=None):
    rclpy.init(args=args)
    
    # Create a temporary node to read robot_name parameter
    temp_node = Node("temp_param_reader")
    temp_node.declare_parameter('robot_name', 'floatsam_usv')
    robot_name = temp_node.get_parameter('robot_name').value
    temp_node.destroy_node()
    
    # Create the actual node with proper namespace
    node = Node("floatsam_move_to_action_server", namespace=robot_name)
    node.declare_parameter('robot_name', robot_name)
    
    move_to_action = MoveToActionFloatSam(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()