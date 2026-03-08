#!/usr/bin/python

from rclpy.node import Node
from rclpy.time import Time, Duration

from geometry_msgs.msg import  PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_utilities.georef_utils import convert_latlon_to_utm, convert_utm_to_latlon

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

    def convert_map_point_to_geopoint(self, x: float, y: float, z: float = 0.0) -> GeoPoint:
        """Convert a point in the map frame back to a GeoPoint (lat/lon)."""
        in_map = PoseStamped()
        in_map.header.frame_id = self.MAP_FRAME
        in_map.pose.position.x = float(x)
        in_map.pose.position.y = float(y)
        in_map.pose.position.z = float(z)

        # Cache del frame UTM per non dover interrogare l'albero TF ad ogni singolo waypoint
        if not hasattr(self, '_utm_frame_cache') or self._utm_frame_cache is None:
            # Lista dei frame UTM più comuni in SMARC e nel mondo (es. utm_33, utm_32, ecc.)
            candidates = ['utm_33_V', 'utm'] + [f'utm_{i}' for i in range(1, 61)]
            
            source_frame = None
            for candidate in candidates:
                try:
                    # Controllo rapidissimo (timeout=0) per vedere se il frame esiste nel buffer
                    self._tf_buffer.lookup_transform(
                        target_frame=candidate,
                        source_frame=self.MAP_FRAME,
                        time=Time(seconds=0),
                        timeout=Duration(seconds=0)
                    )
                    source_frame = candidate
                    break  # Trovato! Usciamo dal ciclo
                except Exception:
                    continue
                    
            if source_frame is None:
                raise RuntimeError(f"Could not find a TF from '{self.MAP_FRAME}' to any valid UTM frame (e.g., 'utm_33').")
            
            # Salviamo il frame corretto per i calcoli futuri
            self._utm_frame_cache = source_frame

        # Ora eseguiamo la trasformazione reale usando il frame UTM corretto
        try:
            tf_inv = self._tf_buffer.lookup_transform(
                target_frame=self._utm_frame_cache,
                source_frame=self.MAP_FRAME,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
        except Exception as e:
            self._utm_frame_cache = None  # Resettiamo la cache se per qualche motivo fallisce
            raise RuntimeError(f"Failed to transform map to {self._utm_frame_cache}: {e}")

        in_utm = do_transform_pose_stamped(in_map, tf_inv)
        in_utm.header.frame_id = self._utm_frame_cache
        
        # Ora in_utm.header.frame_id conterrà qualcosa come "utm_33", e il convertitore non crasherà!
        return convert_utm_to_latlon(in_utm)