#!/usr/bin/env python3
"""
SMaRC Topics Publisher Node for LoLo
Converts simulator or real hardware topics to standard SMaRC topics
Note: LoLo simulator already publishes many topics in SMaRC format!
"""

import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import importlib

# ROS message types
from sensor_msgs.msg import NavSatFix, Imu, FluidPressure, Range, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from geographic_msgs.msg import GeoPoint
import importlib


class SmarcTopicsPublisher(Node):
    """
    Bridge node that converts LoLo-specific topics (sim or real) to standard SMaRC topics.
    Uses a switch case (if/else) based on use_sim parameter to load appropriate config.
    
    Note: The LoLo simulator already publishes many topics in SMaRC format, so this node
    primarily acts as a passthrough for those, and converts raw sensor data.
    """
    
    def __init__(self):
        super().__init__('lolo_smarc_topics_publisher')
        
        # Declare parameters
        self.declare_parameter('use_sim', True)
        self.declare_parameter('robot_name', 'lolo_auv_v1')
        
        self.use_sim = self.get_parameter('use_sim').value
        self.robot_name = self.get_parameter('robot_name').value
        
        # Switch case: Load appropriate configuration
        if self.use_sim:
            config_file = 'sim_topics.yaml'
            self.get_logger().info('🎮 SIMULATION MODE: Using simulator topics')
        else:
            config_file = 'real_topics.yaml'
            self.get_logger().info('🤖 REAL HARDWARE MODE: Using real robot topics')
        
        # Load configuration
        self.config = self._load_config(config_file)
        
        # Replace {robot_name} placeholder in all topic paths
        self._substitute_robot_name()
        
        # Create subscribers and publishers
        self._setup_topic_bridges()
        
        self.get_logger().info(f'✅ LoLo SMaRC Topics Publisher started for: {self.robot_name}')
    
    def _load_config(self, config_file):
        """Load YAML configuration file"""
        try:
            package_dir = get_package_share_directory('lolo_topic_bridge')
            config_path = os.path.join(package_dir, 'config', config_file)
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.get_logger().info(f'📄 Loaded configuration from: {config_file}')
            return config
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load config file {config_file}: {e}')
            return {'sensors': {}, 'smarc_format_topics': {}, 'actuators': {}, 'payload': {}}
    
    def _substitute_robot_name(self):
        """Replace {robot_name} placeholder in all topic paths with actual robot name"""
        for category in ['sensors', 'smarc_format_topics', 'actuators', 'payload']:
            if category in self.config:
                for topic_name, topic_config in self.config[category].items():
                    if 'input_topic' in topic_config:
                        topic_config['input_topic'] = topic_config['input_topic'].replace('{robot_name}', self.robot_name)
                    if 'output_topic' in topic_config:
                        topic_config['output_topic'] = topic_config['output_topic'].replace('{robot_name}', self.robot_name)
        self.get_logger().info(f'🔄 Substituted {{robot_name}} with: {self.robot_name}')
    
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
        """Create a generic passthrough callback that just republishes the message"""
        def callback(msg):
            publisher.publish(msg)
        return callback
    
    def _setup_topic_bridges(self):
        """Set up subscribers and publishers for all configured topics"""
        
        # Helper to add vehicle namespace
        def namespaced_topic(topic):
            """Add vehicle namespace if not already present"""
            if topic.startswith('/') or topic.startswith(self.robot_name):
                return topic
            return f'{self.robot_name}/{topic}'
        
        # Handle SMaRC format topics (simple passthrough)
        smarc_topics = self.config.get('smarc_format_topics', {})
        for topic_name, topic_config in smarc_topics.items():
            msg_class = self._get_message_class(topic_config['msg_type'])
            if msg_class is None:
                continue
            
            publisher = self.create_publisher(msg_class, namespaced_topic(topic_config['output_topic']), 10)
            self.create_subscription(msg_class, topic_config['input_topic'],
                                    self._create_passthrough_callback(publisher), 10)
            self.get_logger().info(f'  {topic_name.upper()}: {topic_config["input_topic"]} → {namespaced_topic(topic_config["output_topic"])}')
        
        # Handle raw sensor topics
        sensors = self.config.get('sensors', {})
        
        # GPS
        if 'gps' in sensors:
            self.create_subscription(NavSatFix, sensors['gps']['input_topic'], self._gps_callback, 10)
            self.gps_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['gps']['output_topic']), 10)
            self.get_logger().info(f'  GPS (raw): {sensors["gps"]["input_topic"]} → {namespaced_topic(sensors["gps"]["output_topic"])}')
        
        # IMU
        if 'imu' in sensors:
            self.create_subscription(Imu, sensors['imu']['input_topic'], self._imu_callback, 10)
            self.imu_pub = self.create_publisher(Imu, namespaced_topic(sensors['imu']['output_topic']), 10)
            self.get_logger().info(f'  IMU (raw): {sensors["imu"]["input_topic"]} → {namespaced_topic(sensors["imu"]["output_topic"])}')
        
        # Depth Pressure
        if 'depth_pressure' in sensors:
            self.create_subscription(FluidPressure, sensors['depth_pressure']['input_topic'],
                                    self._depth_pressure_callback, 10)
            self.depth_pressure_pub = self.create_publisher(FluidPressure, namespaced_topic(sensors['depth_pressure']['output_topic']), 10)
            self.get_logger().info(f'  Depth Pressure (raw): {sensors["depth_pressure"]["input_topic"]} → {namespaced_topic(sensors["depth_pressure"]["output_topic"])}')
        
        # DVL
        if 'dvl' in sensors:
            self.create_subscription(Range, sensors['dvl']['input_topic'], self._dvl_callback, 10)
            self.dvl_pub = self.create_publisher(Range, namespaced_topic(sensors['dvl']['output_topic']), 10)
            self.get_logger().info(f'  DVL (raw): {sensors["dvl"]["input_topic"]} → {namespaced_topic(sensors["dvl"]["output_topic"])}')
        
        # Leak sensor
        if 'leak' in sensors:
            self.create_subscription(Bool, sensors['leak']['input_topic'], self._leak_callback, 10)
            self.leak_pub = self.create_publisher(Bool, namespaced_topic(sensors['leak']['output_topic']), 10)
            self.get_logger().info(f'  Leak: {sensors["leak"]["input_topic"]} → {namespaced_topic(sensors["leak"]["output_topic"])}')
        
        # Actuators (passthrough with dynamic typing)
        actuators = self.config.get('actuators', {})
        for actuator_name, config in actuators.items():
            msg_class = self._get_message_class(config['msg_type'])
            if msg_class:
                pub = self.create_publisher(msg_class, namespaced_topic(config['output_topic']), 10)
                self.create_subscription(msg_class, config['input_topic'],
                                        self._create_passthrough_callback(pub), 10)
                self.get_logger().info(f'  Actuator {actuator_name}: {config["input_topic"]} → {namespaced_topic(config["output_topic"])}')
        
        # Payload sensors (passthrough with dynamic typing)
        payload = self.config.get('payload', {})
        for payload_name, config in payload.items():
            msg_class = self._get_message_class(config['msg_type'])
            if msg_class:
                pub = self.create_publisher(msg_class, namespaced_topic(config['output_topic']), 10)
                self.create_subscription(msg_class, config['input_topic'],
                                        self._create_passthrough_callback(pub), 10)
                self.get_logger().info(f'  Payload {payload_name}: {config["input_topic"]} → {namespaced_topic(config["output_topic"])}')
    
    def _gps_callback(self, msg: NavSatFix):
        """Pass through raw GPS data"""
        self.gps_pub.publish(msg)
    
    def _imu_callback(self, msg: Imu):
        """Pass through raw IMU data"""
        self.imu_pub.publish(msg)
    
    def _depth_pressure_callback(self, msg: FluidPressure):
        """Pass through raw depth pressure data"""
        self.depth_pressure_pub.publish(msg)
    
    def _dvl_callback(self, msg: Range):
        """Pass through raw DVL data"""
        self.dvl_pub.publish(msg)
    
    def _leak_callback(self, msg: Bool):
        """Pass through leak sensor data with warning"""
        self.leak_pub.publish(msg)
        if msg.data:
            self.get_logger().warn('⚠️  LEAK DETECTED!')


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
