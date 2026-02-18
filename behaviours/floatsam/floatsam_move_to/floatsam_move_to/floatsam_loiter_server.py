#!/usr/bin/python

import numpy as np
import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.time import Time, Duration
import traceback

from .floatsam_common import FloatSam

from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry
from smarc_msgs.action import BaseAction
from std_msgs.msg import String


from smarc_action_base.gentler_action_server import GentlerActionServer



class LoiterActionFloatSam():
    """
    Loiter action server that maintains FloatSam position within a tolerance circle.
    When the vehicle drifts outside the tolerance, it triggers move_to action with strict tolerance.
    """
    def __init__(self, node: Node):
        self._node: Node = node
        
        # Declare node parameters (internal configuration)
        self.declare_node_parameters()

        # --- PID parameters and threshold for captain ---
        self._loiter_yaw_p_gain = str(self._node.get_parameter('yaw_p_gain').value)
        self._loiter_yaw_i_gain = str(self._node.get_parameter('yaw_i_gain').value)
        self._loiter_yaw_d_gain = str(self._node.get_parameter('yaw_d_gain').value)
        self._loiter_yaw_threshold = str(self._node.get_parameter('yaw_threshold').value)

        self._loiter_yawrate_p_gain = str(self._node.get_parameter('yawrate_p_gain').value)
        self._loiter_yawrate_i_gain = str(self._node.get_parameter('yawrate_i_gain').value)
        self._loiter_yawrate_d_gain = str(self._node.get_parameter('yawrate_d_gain').value)

        self._loiter_velocity_p_gain = str(self._node.get_parameter('velocity_p_gain').value)
        self._loiter_velocity_i_gain = str(self._node.get_parameter('velocity_i_gain').value)
        self._loiter_velocity_d_gain = str(self._node.get_parameter('velocity_d_gain').value)
        
        # Get robot name from node parameter
        self._robot_name: str = self._node.get_parameter('robot_name').value
        
        self.MAP_FRAME: str = self._robot_name + '/map'
        self._floatsam = FloatSam(node, self._robot_name)
        
        self._node.get_logger().info(f"FloatSam loiter server initialized for robot: {self._robot_name}")

        # Internal configuration (from node parameters)
        self._loiter_tolerance = float(self._node.get_parameter('loiter_tolerance').value)
        self._reposition_tolerance = float(self._node.get_parameter('loiter_reposition_tolerance').value)
        self._loiter_move_to_speed = str(self._node.get_parameter('loiter_move_to_speed').value)
        
        self._node.get_logger().info(
            f"Loiter configuration: tolerance={self._loiter_tolerance}m, "
            f"reposition_tolerance={self._reposition_tolerance}m, move_to_speed={self._loiter_move_to_speed}"
        )
        
        # State variables
        self._loiter_center_in_map: PoseStamped | None = None
        self._loiter_center_geopoint: GeoPoint | None = None  # Store original geopoint
        self._current_gps: GeoPoint | None = None  # Current GPS position from topic
        self._distance_from_center: float | None = None
        self._timeout: float | None = None  # seconds
        self._start_time: float | None = None  # timestamp when loiter started
        self._last_reposition_trigger: float = 0.0  # Prevent rapid retriggering
        self._move_to_goal_handle = None  # Track active move_to goal
        self._move_to_result_future = None  # Track move_to completion
        
        # Subscribe to GPS topic to get current lat/lon position
        gps_topic = f"/{self._robot_name}/smarc/latlon"
        self._node.create_subscription(
            GeoPoint,
            gps_topic,
            self._gps_callback,
            10
        )
        self._node.get_logger().info(f"Subscribed to GPS: {gps_topic}")
        
        # Action client to call move_to when out of bounds
        self._move_to_client = ActionClient(
            self._node,
            BaseAction,
            'move_to'
        )
        
        # Get the full action name for debugging
        action_name = self._node.get_namespace() + '/move_to' if self._node.get_namespace() != '/' else '/move_to'
        self._node.get_logger().info(f"Waiting for move_to action server at: {action_name}")
        server_available = self._move_to_client.wait_for_server(timeout_sec=5.0)
        if server_available:
            self._node.get_logger().info(f"move_to action server available at {action_name}!")
        else:
            self._node.get_logger().error(f"move_to action server NOT available at {action_name}! Loiter will not work properly.")
            self._node.get_logger().error("Make sure the floatsam_move_to_action_server is running.")
        
        # Publishers for direct control (when inside tolerance)
        self._speed_reference_publisher = self._node.create_publisher(
            FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, 10
        )

        # publisher for captain parameters 
        self._captain_parameters_publisher = self._node.create_publisher(
            String, 
            'captain_parameters',
            10
        )
        
        # Create the loiter action server
        self._as = GentlerActionServer(
            node,
            "loiter",
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=10  # Check position at 10Hz
        )
        
        # Timer for initial position check
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
    
    def _gps_callback(self, msg: GeoPoint):
        """Store current GPS position."""
        self._current_gps = msg

    def _check_initial_position(self):
        """Timer callback: print the first floatsam position received from odom."""
        if self._floatsam.floatsam_in_map is not None:
            p = self._floatsam.floatsam_in_map.pose.position
            self._node.get_logger().info(f"FloatSam initial position: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]")
            try:
                self._initial_pos_timer.cancel()
            except Exception:
                pass
        else:
            now = int(self._node.get_clock().now().nanoseconds * 1e-9)
            if now > self._initial_pos_deadline:
                self._node.get_logger().warning("Timed out waiting for floatsam position")
                try:
                    self._initial_pos_timer.cancel()
                except Exception:
                    pass

    def _on_goal_received(self, goal_request: dict) -> bool:
        """Parse loiter goal: only timeout parameter. Loiter center = current position."""
        self._node.get_logger().info(f"Loiter goal received: {goal_request}")

        try:
            # Parse timeout (only parameter from action goal - standardized convention)
            self._timeout = float(goal_request['timeout'])
            
            self._node.get_logger().info(f"Loiter timeout: {self._timeout} seconds")
            
            # Wait for current position
            if self._floatsam.floatsam_in_map is None:
                self._node.get_logger().warning("No floatsam position available yet, waiting...")
                # Give it a moment to receive odometry
                import time
                for _ in range(10):
                    if self._floatsam.floatsam_in_map is not None:
                        break
                    time.sleep(0.1)
                
                if self._floatsam.floatsam_in_map is None:
                    self._node.get_logger().error("Failed to get current position for loiter center")
                    return False
            
            # Set loiter center to CURRENT position (like lolo does)
            self._loiter_center_in_map = PoseStamped()
            self._loiter_center_in_map.header = self._floatsam.floatsam_in_map.header
            self._loiter_center_in_map.pose = self._floatsam.floatsam_in_map.pose
            
            # Get current GPS position from GPS topic (published by topic bridge)
            if self._current_gps is None:
                self._node.get_logger().warning("No GPS data available yet, waiting...")
                import time
                for _ in range(10):
                    if self._current_gps is not None:
                        break
                    time.sleep(0.1)
                
                if self._current_gps is None:
                    self._node.get_logger().error("Failed to get GPS position for loiter center")
                    return False
            
            # Use current GPS as loiter center (for move_to repositioning)
            self._loiter_center_geopoint = GeoPoint()
            self._loiter_center_geopoint.latitude = self._current_gps.latitude
            self._loiter_center_geopoint.longitude = self._current_gps.longitude
            self._loiter_center_geopoint.altitude = self._current_gps.altitude

            pos = self._loiter_center_in_map.pose.position
            
            self._node.get_logger().info(
                f"Loiter center (current position) in map: [{pos.x:.2f}, {pos.y:.2f}], "
                f"latlon: [{self._loiter_center_geopoint.latitude:.6f}, {self._loiter_center_geopoint.longitude:.6f}], "
                f"tolerance: {self._loiter_tolerance}m, "
                f"reposition_tolerance: {self._reposition_tolerance}m, "
                f"timeout: {self._timeout}s"
            )
            
            return True
        
        except Exception as e:
            self._node.get_logger().error(f"Failed to parse loiter goal request: {e}")
            traceback.print_exc()
            return False

    def _on_cancel_received(self) -> bool:
        """Handle cancellation request."""
        self._node.get_logger().info("Loiter cancel requested, stopping...")
        self._loiter_center_in_map = None
        self._loiter_center_geopoint = None
        # Cancel active move_to goal if it exists
        if self._move_to_goal_handle is not None:
            self._node.get_logger().info("Cancelling active move_to goal...")
            cancel_future = self._move_to_goal_handle.cancel_goal_async()
            self._move_to_goal_handle = None
        
        self._move_to_result_future = None
        
        # Stop the vehicle
        self._publish_zero_setpoints()
        
        return True

    def _prepare_loop(self) -> None:
        """Initialize loop variables."""
        self._distance_from_center = None
        self._start_time = self.now_time  # Record start time for timeout
        self._last_reposition_trigger = 0.0
        return

    def _loop_inner(self) -> bool | None:
        """
        Main loiter loop:
        1. Check if timeout exceeded -> return success
        2. Check if vehicle is within tolerance circle
        3. If outside, trigger move_to action to return to center
        4. If inside, maintain position (publish zero velocity)
        """
        if self._loiter_center_in_map is None:
            self._node.get_logger().error("No loiter center set, failing...")
            return False

        if self._floatsam.floatsam_in_map is None:
            self._node.get_logger().info("No floatsam position available yet, waiting...")
            return None
        
        # Check timeout (like lolo does)
        elapsed_time = self.now_time - self._start_time
        time_remaining = self._timeout - elapsed_time
        
        if elapsed_time >= self._timeout:
            if self._move_to_goal_handle is not None:
                self._node.get_logger().info("Cancelling active move_to goal...")
                cancel_future = self._move_to_goal_handle.cancel_goal_async()
                self._move_to_goal_handle = None
                self._move_to_pending = False
            # Timeout reached - check if within tolerance circle
            if self._distance_from_center is not None and self._distance_from_center <= self._loiter_tolerance:
                self._node.get_logger().info(f"Loiter timeout reached ({self._timeout}s) and within tolerance - completing successfully")
                return True  # Success
            else:
                self._node.get_logger().warning(f"Loiter timeout reached ({self._timeout}s) but NOT within tolerance (distance={self._distance_from_center:.2f}m, tolerance={self._loiter_tolerance}m) - failing")
                return False  # Failure

        
        # Calculate distance from loiter center
        center_position = np.array([
            self._loiter_center_in_map.pose.position.x,
            self._loiter_center_in_map.pose.position.y
        ])
        
        current_position = np.array([
            self._floatsam.floatsam_in_map.pose.position.x,
            self._floatsam.floatsam_in_map.pose.position.y
        ])
        
        error_vector = center_position - current_position
        self._distance_from_center = float(np.linalg.norm(error_vector))
        
        self._node.get_logger().info(
            f"Loitering: time remaining={time_remaining:.1f}s, "
            f"distance from center: {self._distance_from_center:.2f}m "
            f"(tolerance: {self._loiter_tolerance}m)"
        )
        
        # Check if we're outside the tolerance circle
        if self._distance_from_center > self._loiter_tolerance:
            # Check if move_to is already running
            if self._move_to_goal_handle is not None:
                # move_to is active - check if it's done
                if self._move_to_result_future is not None and self._move_to_result_future.done():
                    try:
                        result = self._move_to_result_future.result()
                        if result.result.success:
                            self._node.get_logger().info("move_to completed successfully - returned to center")
                        else:
                            self._node.get_logger().warning("move_to failed - will retry on next loop")
                        # Clear the goal handle so we can trigger again if needed
                        self._move_to_goal_handle = None
                        self._move_to_result_future = None
                    except Exception as e:
                        self._node.get_logger().error(f"Error getting move_to result: {e}")
                        self._move_to_goal_handle = None
                        self._move_to_result_future = None
                else:
                    # move_to still running - DON'T publish anything, let move_to control the robot
                    self._node.get_logger().debug("move_to action still running, waiting for completion...")
                    return None
            else:
                # No move_to running - trigger it
                self._node.get_logger().warning(
                    f"Outside loiter tolerance! Triggering move_to to return to center..."
                )
                self._trigger_move_to_center()
                self._last_reposition_trigger = self.now_time
                
            
            # Continue loitering (don't end the action)
            return None
        
        else:
            # Inside tolerance circle - only publish zero setpoints if move_to is NOT running
            if self._move_to_goal_handle is None:
                self._node.get_logger().info("Within loiter tolerance, maintaining position")
                self._publish_zero_setpoints()
            else:
                self._node.get_logger().debug("Within tolerance but move_to still active, not interfering")
            
            # Continue loitering indefinitely
            return None

    def _trigger_move_to_center(self):
        """Send move_to goal to return to loiter center and track the goal handle."""
        try:
            if self._loiter_center_geopoint is None:
                self._node.get_logger().error("No loiter center geopoint available")
                return
            
            # Create the goal message
            goal_msg = BaseAction.Goal()
            goal_dict = {
                'waypoint': {
                    'latitude': self._loiter_center_geopoint.latitude,
                    'longitude': self._loiter_center_geopoint.longitude,
                    'tolerance': self._reposition_tolerance
                },
                'speed': self._loiter_move_to_speed
            }
            
            goal_msg.goal = String(data=json.dumps(goal_dict))
            
            self._node.get_logger().info(
                f"Sending move_to goal: lat={self._loiter_center_geopoint.latitude:.6f}, "
                f"lon={self._loiter_center_geopoint.longitude:.6f}, "
                f"tolerance={self._reposition_tolerance}m, speed={self._loiter_move_to_speed}"
            )
            
            # Send goal and track it - we need to wait for completion
            send_future = self._move_to_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self._move_to_goal_response_callback)
            
        except Exception as e:
            self._node.get_logger().error(f"Error triggering move_to: {e}")
            traceback.print_exc()
    
    def _move_to_goal_response_callback(self, future):
        """Callback when move_to server accepts/rejects the goal."""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self._node.get_logger().info("move_to goal accepted by server")
                # Store the goal handle and get result future
                self._move_to_goal_handle = goal_handle
                self._move_to_result_future = goal_handle.get_result_async()
                self._node.get_logger().info("Waiting for move_to to complete...")
            else:
                self._node.get_logger().error("move_to goal REJECTED by server")
                self._move_to_goal_handle = None
                self._move_to_result_future = None
        except Exception as e:
            self._node.get_logger().error(f"Error in move_to goal response callback: {e}")
            traceback.print_exc()
            self._move_to_goal_handle = None
            self._move_to_result_future = None

    def _publish_zero_setpoints(self):
        """Publish zero velocity and maintain current heading."""
        speed_msg = FloatStamped()
        speed_msg.header.stamp = self.now_stamp
        speed_msg.data = 0.0
        self._speed_reference_publisher.publish(speed_msg)
        self._publish_captain_parametrs()

    def _give_feedback(self) -> str:
        """Provide feedback about loiter status (time remaining, like lolo)."""
        if self._start_time is not None and self._timeout is not None:
            elapsed_time = self.now_time - self._start_time
            time_remaining = self._timeout - elapsed_time
            return f"time_remaining: {time_remaining:.1f}s, distance: {self._distance_from_center:.2f}m"
        else:
            return "Loiter not started"
        
    def declare_node_parameters(self) -> None:
        self._node.declare_parameter('loiter_tolerance', 5.0)  # meters - loiter circle radius
        self._node.declare_parameter('loiter_reposition_tolerance', 0.5)  # meters - strict tolerance for move_to
        self._node.declare_parameter('loiter_move_to_speed', 'fast')  # move_to speed: 'slow', 'standard', or 'fast'

        self._node.declare_parameter("yaw_p_gain", 0.3)
        self._node.declare_parameter("yaw_i_gain", 0.0)
        self._node.declare_parameter("yaw_d_gain", 0.1)
        self._node.declare_parameter("yaw_threshold", 0.5)

        self._node.declare_parameter("yawrate_p_gain", 300.0)
        self._node.declare_parameter("yawrate_i_gain", 0.0)
        self._node.declare_parameter("yawrate_d_gain", 30.0)

        self._node.declare_parameter("velocity_p_gain", 500.0)
        self._node.declare_parameter("velocity_i_gain", 10.0)
        self._node.declare_parameter("velocity_d_gain", 0.0)

    def _publish_captain_parametrs(self):
        """It publish the message containing the parameters for captain node"""
        parameters = {
            "yaw_p_gain" : self._loiter_yaw_p_gain,
            "yaw_i_gain" : self._loiter_yaw_i_gain,
            "yaw_d_gain" : self._loiter_yaw_d_gain,
            "yaw_threshold" : self._loiter_yaw_threshold,
            "yawrate_p_gain" : self._loiter_yawrate_p_gain,
            "yawrate_i_gain" : self._loiter_yawrate_i_gain,
            "yawrate_d_gain" : self._loiter_yaw_d_gain,
            "velocity_p_gain" : self._loiter_velocity_p_gain, 
            "velocity_i_gain" : self._loiter_velocity_i_gain, 
            "velocity_d_gain" : self._loiter_velocity_d_gain
        }
        msg = String()
        msg.data = json.dumps(parameters)
        self._captain_parameters_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create a temporary node to read robot_name parameter
    temp_node = Node("temp_param_reader")
    temp_node.declare_parameter('robot_name', 'floatsam_usv')
    robot_name = temp_node.get_parameter('robot_name').value
    temp_node.destroy_node()
    
    # Create the actual node with proper namespace
    node = Node("floatsam_loiter_action_server", namespace=robot_name)
    node.declare_parameter('robot_name', robot_name)
    
    loiter_action = LoiterActionFloatSam(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    # Cancel active move_to goal if it exists
    if loiter_action._move_to_goal_handle is not None:
        loiter_action._node.get_logger().info("Cancelling active move_to goal...")
        cancel_future = loiter_action._move_to_goal_handle.cancel_goal_async()
        loiter_action._move_to_goal_handle = None
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
