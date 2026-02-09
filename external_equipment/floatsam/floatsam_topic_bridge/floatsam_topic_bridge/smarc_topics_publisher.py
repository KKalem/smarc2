#!/usr/bin/env python3
"""
SMaRC Topics Publisher Node for Floatsam
Converts simulator or real hardware topics to standard SMaRC topics
"""

import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math
import importlib

# ROS message types
from sensor_msgs.msg import NavSatFix, Imu, FluidPressure, Range, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from geographic_msgs.msg import GeoPoint
from tf_transformations import euler_from_quaternion


class SmarcTopicsPublisher(Node):
    """
    Bridge node that converts Floatsam-specific topics (sim or real) to standard SMaRC topics.
    Uses a switch case (if/else) based on use_sim parameter to load appropriate config.
    """
    
    def __init__(self):
        super().__init__('floatsam_smarc_topics_publisher')
        
        # Declare parameters
        self.declare_parameter('use_sim', True)
        self.declare_parameter('robot_name', 'floatsam_usv')
        
        self.use_sim = self.get_parameter('use_sim').value
        self.robot_name = self.get_parameter('robot_name').value
        
        # Switch case: Load appropriate configuration
        if self.use_sim:
            config_file = 'sim_topics.yaml'
            self.get_logger().info('SIMULATION MODE: Using simulator topics')
        else:
            config_file = 'real_topics.yaml'
            self.get_logger().info('REAL HARDWARE MODE: Using real robot topics')
        
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Replace {robot_name} placeholder in all topic paths
        self._substitute_robot_name()
        
        # Storage for latest messages
        self.latest_odom = None
        self.latest_gps_left = None
        self.latest_gps_right = None
        self.latest_rtk_left = None
        self.latest_rtk_right = None
        
        # Create publishers for derived values (with vehicle namespace)
        self.heading_pub = self.create_publisher(Float32, f'{self.robot_name}/smarc/heading', 10)
        self.course_pub = self.create_publisher(Float32, f'{self.robot_name}/smarc/course', 10)
        self.speed_pub = self.create_publisher(Float32, f'{self.robot_name}/smarc/speed', 10)
        self.latlon_pub = self.create_publisher(GeoPoint, f'{self.robot_name}/smarc/latlon', 10)
        
        # Create subscribers and publishers
        self._setup_topic_bridges()
        
        self.get_logger().info(f'Floatsam SMaRC Topics Publisher started for: {self.robot_name}')
    
    def _load_config(self, config_file):
        """Load YAML configuration file"""
        try:
            package_dir = get_package_share_directory('floatsam_topic_bridge')
            config_path = os.path.join(package_dir, 'config', config_file)
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.get_logger().info(f' Loaded configuration from: {config_file}')
            return config
        except Exception as e:
            self.get_logger().error(f' Failed to load config file {config_file}: {e}')
            return {'sensors': {}, 'actuators': {}, 'payload': {}}
    
    def _substitute_robot_name(self):
        """Replace {robot_name} placeholder in all topic paths with actual robot name"""
        for category in ['sensors', 'actuators', 'payload']:
            if category in self.config:
                for topic_name, topic_config in self.config[category].items():
                    if 'input_topic' in topic_config:
                        topic_config['input_topic'] = topic_config['input_topic'].replace('{robot_name}', self.robot_name)
                    if 'output_topic' in topic_config:
                        topic_config['output_topic'] = topic_config['output_topic'].replace('{robot_name}', self.robot_name)
        self.get_logger().info(f' Substituted {{robot_name}} with: {self.robot_name}')
    
    def _get_message_class(self, msg_type_str):
        """Dynamically import and return message class from string like 'std_msgs/Float32'"""
        try:
            pkg, msg = msg_type_str.split('/')
            module = importlib.import_module(f'{pkg}.msg')
            return getattr(module, msg)
        except Exception as e:
            self.get_logger().error(f'Failed to import message type {msg_type_str}: {e}')
            return None
    
    def _create_passthrough_callback(self, publisher):
        """Create a generic passthrough callback"""
        def callback(msg):
            publisher.publish(msg)
        return callback
    
    def _setup_topic_bridges(self):
        """Set up subscribers and publishers for all configured topics"""
        sensors = self.config.get('sensors', {})
        actuators = self.config.get('actuators', {})
        payload = self.config.get('payload', {})
        
        # Helper to add vehicle namespace
        def namespaced_topic(topic):
            """Add vehicle namespace if not already present"""
            if topic.startswith('/') or topic.startswith(self.robot_name):
                return topic
            return f'{self.robot_name}/{topic}'
        
        # GPS sensors (dual)
        if 'gps_left' in sensors:
            self.create_subscription(NavSatFix, sensors['gps_left']['input_topic'], 
                                     self._gps_left_callback, 10)
            self.gps_left_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['gps_left']['output_topic']), 10)
            self.get_logger().info(f'  GPS Left: {sensors["gps_left"]["input_topic"]} → {namespaced_topic(sensors["gps_left"]["output_topic"])}')
        
        if 'gps_right' in sensors:
            self.create_subscription(NavSatFix, sensors['gps_right']['input_topic'], 
                                     self._gps_right_callback, 10)
            self.gps_right_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['gps_right']['output_topic']), 10)
            self.get_logger().info(f'  GPS Right: {sensors["gps_right"]["input_topic"]} → {namespaced_topic(sensors["gps_right"]["output_topic"])}')
        
        # RTK GPS (high precision)
        if 'rtk_left' in sensors:
            self.create_subscription(NavSatFix, sensors['rtk_left']['input_topic'], 
                                     self._rtk_left_callback, 10)
            self.rtk_left_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['rtk_left']['output_topic']), 10)
            self.get_logger().info(f'  RTK Left: {sensors["rtk_left"]["input_topic"]} → {namespaced_topic(sensors["rtk_left"]["output_topic"])}')
        
        if 'rtk_right' in sensors:
            self.create_subscription(NavSatFix, sensors['rtk_right']['input_topic'], 
                                     self._rtk_right_callback, 10)
            self.rtk_right_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['rtk_right']['output_topic']), 10)
            self.get_logger().info(f'  RTK Right: {sensors["rtk_right"]["input_topic"]} → {namespaced_topic(sensors["rtk_right"]["output_topic"])}')
        
        # IMU (raw data only, heading comes from odom)
        if 'imu' in sensors:
            self.create_subscription(Imu, sensors['imu']['input_topic'], self._imu_callback, 10)
            self.imu_pub = self.create_publisher(Imu, namespaced_topic(sensors['imu']['output_topic']), 10)
            self.get_logger().info(f'  IMU (raw): {sensors["imu"]["input_topic"]} → {namespaced_topic(sensors["imu"]["output_topic"])}')
        
        # Depth Pressure
        if 'depth_pressure' in sensors:
            self.create_subscription(FluidPressure, sensors['depth_pressure']['input_topic'], 
                                     self._depth_callback, 10)
            self.depth_pub = self.create_publisher(Float32, namespaced_topic(sensors['depth_pressure']['output_topic']), 10)
            self.get_logger().info(f'  Depth: {sensors["depth_pressure"]["input_topic"]} → {namespaced_topic(sensors["depth_pressure"]["output_topic"])}')
        
        # DVL
        if 'dvl' in sensors:
            self.create_subscription(Range, sensors['dvl']['input_topic'], self._dvl_callback, 10)
            self.dvl_pub = self.create_publisher(Range, namespaced_topic(sensors['dvl']['output_topic']), 10)
            self.get_logger().info(f'  DVL: {sensors["dvl"]["input_topic"]} → {namespaced_topic(sensors["dvl"]["output_topic"])}')
        
        # Leak sensor
        if 'leak' in sensors:
            self.create_subscription(Bool, sensors['leak']['input_topic'], self._leak_callback, 10)
            self.leak_pub = self.create_publisher(Bool, namespaced_topic(sensors['leak']['output_topic']), 10)
            self.get_logger().info(f'  Leak: {sensors["leak"]["input_topic"]} → {namespaced_topic(sensors["leak"]["output_topic"])}')
        
        # Odometry (computes heading, course, speed, latlon)
        if 'odom_gt' in sensors:
            odom_config = sensors['odom_gt']
        elif 'odom' in sensors:
            odom_config = sensors['odom']
        else:
            odom_config = None
            
        if odom_config:
            self.create_subscription(Odometry, odom_config['input_topic'], self._odom_callback, 10)
            self.odom_pub = self.create_publisher(Odometry, namespaced_topic(odom_config['output_topic']), 10)
            self.get_logger().info(f'  Odom: {odom_config["input_topic"]} → {namespaced_topic(odom_config["output_topic"])}')
            self.get_logger().info(f'  ↳ Also computing heading, course, speed, and latlon from best GPS')
        
        # Battery
        if 'battery' in sensors:
            self.create_subscription(Float32, sensors['battery']['input_topic'], 
                                     self._battery_callback, 10)
            self.battery_pub = self.create_publisher(Float32, namespaced_topic(sensors['battery']['output_topic']), 10)
            self.get_logger().info(f'  Battery: {sensors["battery"]["input_topic"]} → {namespaced_topic(sensors["battery"]["output_topic"])}')
        
        # Actuators (passthrough with dynamic typing)
        for actuator_name, config in actuators.items():
            msg_class = self._get_message_class(config['msg_type'])
            if msg_class:
                pub = self.create_publisher(msg_class, namespaced_topic(config['output_topic']), 10)
                self.create_subscription(msg_class, config['input_topic'], 
                                        self._create_passthrough_callback(pub), 10)
                self.get_logger().info(f'  Actuator {actuator_name}: {config["input_topic"]} → {namespaced_topic(config["output_topic"])}')
        
        # Payload sensors (passthrough with dynamic typing)
        for payload_name, config in payload.items():
            msg_class = self._get_message_class(config['msg_type'])
            if msg_class:
                pub = self.create_publisher(msg_class, namespaced_topic(config['output_topic']), 10)
                self.create_subscription(msg_class, config['input_topic'], 
                                        self._create_passthrough_callback(pub), 10)
                self.get_logger().info(f'  Payload {payload_name}: {config["input_topic"]} → {namespaced_topic(config["output_topic"])}')
    
    def _gps_left_callback(self, msg: NavSatFix):
        """Store and publish GPS left"""
        self.latest_gps_left = msg
        self.gps_left_pub.publish(msg)
        self._publish_best_gps()
    
    def _gps_right_callback(self, msg: NavSatFix):
        """Store and publish GPS right"""
        self.latest_gps_right = msg
        self.gps_right_pub.publish(msg)
        self._publish_best_gps()
    
    def _rtk_left_callback(self, msg: NavSatFix):
        """Store and publish RTK left"""
        self.latest_rtk_left = msg
        self.rtk_left_pub.publish(msg)
        self._publish_best_gps()
    
    def _rtk_right_callback(self, msg: NavSatFix):
        """Store and publish RTK right"""
        self.latest_rtk_right = msg
        self.rtk_right_pub.publish(msg)
        self._publish_best_gps()
    
    def _publish_best_gps(self):
        """Publish best available GPS to smarc/latlon (RTK preferred)"""
        # Priority: RTK > GPS
        best_gps = self.latest_rtk_left or self.latest_rtk_right or \
                   self.latest_gps_left or self.latest_gps_right
        
        if best_gps:
            geopoint = GeoPoint()
            geopoint.latitude = best_gps.latitude
            geopoint.longitude = best_gps.longitude
            geopoint.altitude = best_gps.altitude
            self.latlon_pub.publish(geopoint)
    
    def _imu_callback(self, msg: Imu):
        """Pass through raw IMU data (heading comes from odom)"""
        self.imu_pub.publish(msg)
    
    def _depth_callback(self, msg: FluidPressure):
        """Convert pressure to depth (assuming seawater)"""
        # Approximate: depth (m) = (pressure - atmospheric) / (water_density * g)
        # Atmospheric pressure ~101325 Pa
        # Seawater density * g ≈ 10000 Pa/m (more accurate for ocean)
        atmospheric_pressure = 101325.0  # Pa
        water_pressure_per_meter = 10000.0  # Pa/m (seawater)
        
        depth_m = (msg.fluid_pressure - atmospheric_pressure) / water_pressure_per_meter
        depth_m = max(0.0, depth_m)  # Depth cannot be negative
        
        depth_msg = Float32()
        depth_msg.data = depth_m
        self.depth_pub.publish(depth_msg)
    
    def _dvl_callback(self, msg: Range):
        """Pass through DVL data"""
        self.dvl_pub.publish(msg)
    
    def _leak_callback(self, msg: Bool):
        """Pass through leak sensor data"""
        self.leak_pub.publish(msg)
        if msg.data:
            self.get_logger().warn('  LEAK DETECTED!')
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry and compute derived values (heading, course, speed)"""
        # Store for later use
        self.latest_odom = msg
        
        # Publish odometry
        self.odom_pub.publish(msg)
        
        # Extract heading from odom orientation (quaternion to yaw)
        orientation_list = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        # Convert to degrees and normalize to [0, 360)
        heading_deg = math.degrees(yaw)
        if heading_deg < 0:
            heading_deg += 360.0
        
        heading_msg = Float32()
        heading_msg.data = 90 - heading_deg
        self.heading_pub.publish(heading_msg)
        
        # Compute and publish course (direction of travel in degrees)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        course_rad = math.atan2(vy, vx)
        course_deg = math.degrees(course_rad)
        if course_deg < 0:
            course_deg += 360.0
        
        course_msg = Float32()
        course_msg.data = course_deg
        self.course_pub.publish(course_msg)
        
        # Compute and publish speed (magnitude of velocity)
        speed = math.sqrt(vx**2 + vy**2)
        speed_msg = Float32()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
    
    def _battery_callback(self, msg: Float32):
        """Pass through battery percentage"""
        self.battery_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SmarcTopicsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
