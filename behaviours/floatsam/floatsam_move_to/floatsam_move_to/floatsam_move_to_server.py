#!/usr/bin/python

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration

import traceback

from .floatsam_common import FloatSam

from geometry_msgs.msg import  PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_action_base.gentler_action_server import GentlerActionServer
import time

class MoveToActionFloatSam():
    def __init__(self,
                 node: Node):
        self._node : Node = node
        self._robot_name : str = "floatsam_usv"
        self.MAP_FRAME : str = self._robot_name + '/map'
        self._floatsam = FloatSam(node, self._robot_name)

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

    @property
    def now_stamp(self):
        return self._node.get_clock().now().to_msg()
    
    @property
    def now_time(self):
        return self.now_stamp.sec + self.now_stamp.nanosec * 1e-9
    
    def log(self, msg: str):
        self._node.get_logger().info(msg)

    
    def _on_goal_received(self, goal_request: dict) -> bool:
        self.log("PISELLI\n")


    def _on_cancel_received(self) -> bool:
        self.log("Cancel requested, stopping...")
        self._goal_in_map = None
        return True

    def _prepare_loop(self) -> None:
        self._distance_remaining = None
        return

    def _loop_inner(self) -> bool|None:
        if self._goal_in_map is None:
            self.log("No goal set, failing...")
            return False

        if self._goal_tolerance is None:
            self.log("No goal tolerance set, failing...")
            return False

        if self._floatsam.floatsam_in_map is None:
            self.log("No floatsam position available yet, waiting...")
            return None
        return None

    def _give_feedback(self) -> str:
        if self._distance_remaining is not None:
            return f"Distance remaining: {self._distance_remaining:.2f} (tolerance: {self._goal_tolerance:.2f}m)"
        else:
            return "No distance remaining info"
        

def main(args=None):
    rclpy.init(args=args)
    node = Node("floatsam_move_to_action_server")
    move_to_action = MoveToActionFloatSam(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()