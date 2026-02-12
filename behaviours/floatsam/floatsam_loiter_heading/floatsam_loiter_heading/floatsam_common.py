#!/usr/bin/python

from rclpy.node import Node
from rclpy.time import Time, Duration

from geometry_msgs.msg import  PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_utilities.georef_utils import convert_latlon_to_utm

class FloatSam():
    def __init__(self,
                 node: Node,
                 robot_name: str):
        
        self._node : Node = node
        self._floatsam_in_map : None | PoseStamped = None

        self.MAP_FRAME : str = robot_name + '/map'
        self.ODOM_FRAME : str = 'unity_origin'

        self._tf_buffer : Buffer = Buffer()
        self._tf_listener : TransformListener = TransformListener(self._tf_buffer, self._node, spin_thread=True)

        found = False
        while not found:
            try:
                self._odom_to_map_tf = self._tf_buffer.lookup_transform(self.MAP_FRAME, self.ODOM_FRAME, Time(), Duration(seconds=1))
                found = True
            except Exception as e:
                self._node.get_logger().info(f"Waiting for transform from {self.ODOM_FRAME} to {self.MAP_FRAME}...")
        

        # Subscribe to odometry with robot namespace
        odom_topic = f"/{robot_name}/smarc/odom"
        self._node.create_subscription(Odometry,
                                       odom_topic,
                                       self._odom_cb,
                                       10)
        self._node.get_logger().info(f"Subscribed to odometry: {odom_topic}")
        
    def _odom_cb(self, msg_odom: Odometry):
        floatsam_in_odom : PoseStamped = PoseStamped()
        floatsam_in_odom.header = msg_odom.header
        floatsam_in_odom.pose = msg_odom.pose.pose
        try:
            self._floatsam_in_map = do_transform_pose_stamped(floatsam_in_odom, self._odom_to_map_tf)
            #print(f"Floatsam in map: {self._floatsam_in_map}")
        except Exception as e:
            self._node.get_logger().error(f"Error transforming drone pose from odom to map: {e}")

    @property
    def floatsam_in_map(self) -> PoseStamped|None:
        return self._floatsam_in_map    
    

    def convert_geopoint_to_map_pose_stamped(self, gp: GeoPoint) -> PoseStamped:
        in_utm : PointStamped = convert_latlon_to_utm(gp)
        in_utm_pose : PoseStamped = PoseStamped()
        in_utm_pose.header = in_utm.header
        in_utm_pose.pose.position = in_utm.point
        in_utm_pose.pose.position.z = gp.altitude  # keep the altitude from the GeoPoint as is

        self._node.get_logger().info(f"Converting GeoPoint -> UTM frame '{in_utm.header.frame_id}' and then to map '{self.MAP_FRAME}'")

        source_frame = in_utm.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=self.MAP_FRAME,
                source_frame=source_frame,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
        except Exception as e:
            # first fallback: try generic 'utm' frame which some setups publish
            self._node.get_logger().warning(
                f"Lookup for transform from '{source_frame}' to '{self.MAP_FRAME}' failed: {e}. Trying fallback 'utm' frame."
            )
            try:
                tf = self._tf_buffer.lookup_transform(
                    target_frame=self.MAP_FRAME,
                    source_frame='utm',
                    time=Time(seconds=0),
                    timeout=Duration(seconds=1)
                )
                self._node.get_logger().info("Fallback to 'utm' frame successful.")
            except Exception as e2:
                # give a more informative error for callers
                err_msg = (
                    f"Failed to find a transform from any UTM frame to '{self.MAP_FRAME}'. "
                    f"Tried '{source_frame}' and 'utm'. Original error: {e}. Fallback error: {e2}"
                )
                self._node.get_logger().error(err_msg)
                raise

        in_map = do_transform_pose_stamped(in_utm_pose, tf)
        in_map.pose.position.z = gp.altitude  # ensure altitude is preserved
        return in_map