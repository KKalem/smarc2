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
from rclpy.qos import qos_profile_sensor_data

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
        self.latest_rtk_position = None
        self.is_receiving_rtk_heading = False
        self.latest_rtk_heading = None
        self.latest_port_cmd = 0.0
        self.latest_strb_cmd = 0.0
        
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
        # GPS Left Example
        if 'gps_left' in sensors:
            # Dynamically load the correct class (NavSatFix for Sim, SensorGps for Real)
            msg_class = self._get_message_class(sensors['gps_left']['msg_type'])
            
            self.create_subscription(msg_class, sensors['gps_left']['input_topic'], 
                                     self._gps_left_callback, qos_profile_sensor_data)
            
            self.gps_left_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['gps_left']['output_topic']), qos_profile_sensor_data)
            
        if 'gps_right' in sensors:
            # Dynamically load the correct class (NavSatFix for Sim, SensorGps for Real)
            msg_class = self._get_message_class(sensors['gps_right']['msg_type'])
            
            self.create_subscription(msg_class, sensors['gps_right']['input_topic'], 
                                     self._gps_right_callback, qos_profile_sensor_data)
            self.gps_right_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['gps_right']['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  GPS Right: {sensors["gps_right"]["input_topic"]} → {namespaced_topic(sensors["gps_right"]["output_topic"])}')
        
        # RTK GPS (high precision)
        if 'rtk_position' in sensors:
            msg_class = self._get_message_class(sensors['rtk_position']['msg_type'])
            
            self.create_subscription(msg_class, sensors['rtk_position']['input_topic'], 
                                     self._rtk_position_callback, 10)
            self.rtk_position_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['rtk_position']['output_topic']), 10)
            self.get_logger().info(f'  RTK Position: {sensors["rtk_position"]["input_topic"]} → {namespaced_topic(sensors["rtk_position"]["output_topic"])}')
        
        if 'rtk_heading' in sensors:
            msg_class = self._get_message_class(sensors['rtk_heading']['msg_type'])
            
            self.create_subscription(msg_class, sensors['rtk_heading']['input_topic'], 
                                     self._rtk_heading_callback, 10)
            self.rtk_heading_pub = self.create_publisher(NavSatFix, namespaced_topic(sensors['rtk_heading']['output_topic']), 10)
            self.get_logger().info(f'  RTK heading: {sensors["rtk_heading"]["input_topic"]} → {namespaced_topic(sensors["rtk_heading"]["output_topic"])}')
        
        # IMU (raw data only, heading comes from odom)
        if 'imu' in sensors:
            # Dynamically load the correct class (Imu for Sim, SensorImu for Real)
            msg_class = self._get_message_class(sensors['imu']['msg_type'])
            
            self.create_subscription(msg_class, sensors['imu']['input_topic'], self._imu_callback, 10)
            self.imu_pub = self.create_publisher(Imu, namespaced_topic(sensors['imu']['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  IMU (raw): {sensors["imu"]["input_topic"]} → {namespaced_topic(sensors["imu"]["output_topic"])}')
        
        # Depth Pressure
        if 'depth_pressure' in sensors:
            # Dynamically load the correct class (FluidPressure for Sim, SensorFluidPressure for Real)
            msg_class = self._get_message_class(sensors['depth_pressure']['msg_type'])
            
            self.create_subscription(msg_class, sensors['depth_pressure']['input_topic'], 
                                     self._depth_callback, 10)
            self.depth_pub = self.create_publisher(Float32, namespaced_topic(sensors['depth_pressure']['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  Depth: {sensors["depth_pressure"]["input_topic"]} → {namespaced_topic(sensors["depth_pressure"]["output_topic"])}')
        
        # DVL
        if 'dvl' in sensors:
            # Dynamically load the correct class (Range for Sim, SensorDvl for Real)
            msg_class = self._get_message_class(sensors['dvl']['msg_type'])
            
            self.create_subscription(msg_class, sensors['dvl']['input_topic'], self._dvl_callback, 10)
            self.dvl_pub = self.create_publisher(Range, namespaced_topic(sensors['dvl']['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  DVL: {sensors["dvl"]["input_topic"]} → {namespaced_topic(sensors["dvl"]["output_topic"])}')
        
        # Leak sensor
        if 'leak' in sensors:
            # Dynamically load the correct class (Bool for Sim, SensorLeak for Real)
            msg_class = self._get_message_class(sensors['leak']['msg_type'])
            
            self.create_subscription(msg_class, sensors['leak']['input_topic'], self._leak_callback, 10)
            self.leak_pub = self.create_publisher(msg_class, namespaced_topic(sensors['leak']['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  Leak: {sensors["leak"]["input_topic"]} → {namespaced_topic(sensors["leak"]["output_topic"])}')
        
        # Odometry (computes heading, course, speed, latlon)
        if 'odom_gt' in sensors:
            odom_config = sensors['odom_gt']
        elif 'odom' in sensors:
            odom_config = sensors['odom']
        else:
            odom_config = None
            
        if odom_config:
            # Dynamically load the correct class (Odometry for Sim, SensorOdometry for Real)
            msg_class = self._get_message_class(odom_config['msg_type'])
            
            self.create_subscription(msg_class, odom_config['input_topic'], self._odom_callback, 10)
            self.odom_pub = self.create_publisher(msg_class, namespaced_topic(odom_config['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  Odom: {odom_config["input_topic"]} → {namespaced_topic(odom_config["output_topic"])}')
            self.get_logger().info(f'  ↳ Also computing heading, course, speed, and latlon from best GPS')
        
        # Battery
        if 'battery' in sensors:
            # Dynamically load the correct class (Float32 for Sim, SensorBattery for Real)
            msg_class = self._get_message_class(sensors['battery']['msg_type'])
            
            self.create_subscription(msg_class, sensors['battery']['input_topic'], 
                                     self._battery_callback, 10)
            self.battery_pub = self.create_publisher(Float32, namespaced_topic(sensors['battery']['output_topic']), qos_profile_sensor_data)
            self.get_logger().info(f'  Battery: {sensors["battery"]["input_topic"]} → {namespaced_topic(sensors["battery"]["output_topic"])}')
        
        # --- Actuators (Coupled vs Decoupled Logic) ---
        if 'thruster_port_cmd' in actuators and 'thruster_strb_cmd' in actuators:
            # Always subscribe to the separate input commands
            self.create_subscription(Float32, actuators['thruster_port_cmd']['input_topic'], 
                                     self._port_cmd_callback, 10)
            self.create_subscription(Float32, actuators['thruster_strb_cmd']['input_topic'], 
                                     self._strb_cmd_callback, 10)
            
            if self.use_sim:
                # SIMULATION: Publish to decoupled topics
                self.sim_port_pub = self.create_publisher(Float32, namespaced_topic(actuators['thruster_port_cmd']['output_topic']), 10)
                self.sim_strb_pub = self.create_publisher(Float32, namespaced_topic(actuators['thruster_strb_cmd']['output_topic']), 10)
                self.get_logger().info('  Actuators: Bridged as DECOUPLED (Sim Mode)')
            else:
                # REAL HARDWARE: Publish to coupled PX4 array
                msg_class = self._get_message_class(actuators['px4_motors']['msg_type'])
                self.px4_motors_pub = self.create_publisher(msg_class, actuators['px4_motors']['output_topic'], qos_profile_sensor_data)
                self.get_logger().info('  Actuators: Bridged as COUPLED (PX4 Mode)')
        
        # Payload sensors (passthrough with dynamic typing)
        for payload_name, config in payload.items():
            msg_class = self._get_message_class(config['msg_type'])
            if msg_class:
                pub = self.create_publisher(msg_class, namespaced_topic(config['output_topic']), 10)
                self.create_subscription(msg_class, config['input_topic'], 
                                        self._create_passthrough_callback(pub), qos_profile_sensor_data)
                self.get_logger().info(f'  Payload {payload_name}: {config["input_topic"]} → {namespaced_topic(config["output_topic"])}')
    
    def _port_cmd_callback(self, msg: Float32):
        self.latest_port_cmd = msg.data
        self._publish_actuators()

    def _strb_cmd_callback(self, msg: Float32):
        self.latest_strb_cmd = msg.data
        self._publish_actuators()

    def _publish_actuators(self):
        """Sends commands as either separate topics (Sim) or one array (PX4)"""
        if self.use_sim:
            # Decoupled
            port_msg = Float32()
            port_msg.data = self.latest_port_cmd
            strb_msg = Float32()
            strb_msg.data = self.latest_strb_cmd
            
            self.sim_port_pub.publish(port_msg)
            self.sim_strb_pub.publish(strb_msg)
        else:
            # Coupled for PX4
            px4_msg = self._get_message_class('px4_msgs/ActuatorMotors')()
            px4_msg.control = [0.0] * 12 # Initialize empty array
            
            # Map port and strb to the correct PX4 motor indexes (check which indexes)
            px4_msg.control[0] = float(self.latest_port_cmd)
            px4_msg.control[1] = float(self.latest_strb_cmd)
            
            self.px4_motors_pub.publish(px4_msg)

    def _gps_left_callback(self, msg):
        """Translate PX4 SensorGps into standard ROS 2 NavSatFix"""
        if self.use_sim:
            std_msg = msg
        else:
            std_msg = NavSatFix()
            # PX4 stores lat/lon as integers (degrees * 1e7)
            std_msg.latitude = msg.lat / 1e7
            std_msg.longitude = msg.lon / 1e7
            # PX4 stores altitude in millimeters
            std_msg.altitude = msg.alt / 1000.0
            
        self.latest_gps_left = std_msg
        self.gps_left_pub.publish(std_msg)
        self._publish_best_gps()
    
    def _gps_right_callback(self, msg):
        """Translate PX4 SensorGps into standard ROS 2 NavSatFix"""
        if self.use_sim:
            std_msg = msg
        else:
            std_msg = NavSatFix()
            # PX4 stores lat/lon as integers (degrees * 1e7)
            std_msg.latitude = msg.lat / 1e7
            std_msg.longitude = msg.lon / 1e7
            # PX4 stores altitude in millimeters
            std_msg.altitude = msg.alt / 1000.0
            
        self.latest_gps_right = std_msg
        self.gps_right_pub.publish(std_msg)
        self._publish_best_gps()
    
    def _rtk_position_callback(self, msg: NavSatFix):
        """Store and publish RTK position"""
        self.latest_rtk_position = msg
        self.rtk_position_pub.publish(msg)
        self._publish_best_gps()
    
    def _rtk_heading_callback(self, msg):
        """Store and publish RTK heading, and lock out Odometry heading"""
        
        self.is_receiving_rtk_heading = True 
        heading_msg = Float32()
        heading_msg.data = float(msg.data) 
        self.heading_pub.publish(heading_msg)
    
    def _publish_best_gps(self):
        """Publish best available GPS POSITION to smarc/latlon"""
        # For the map, we ONLY care about the physical position, not the heading.
        # Priority: RTK Position > Main PX4 Left GPS > Main PX4 Right GPS
        best_position = self.latest_rtk_position or self.latest_gps_left or self.latest_gps_right
        
        if best_position:
            geopoint = GeoPoint()
            geopoint.latitude = best_position.latitude
            geopoint.longitude = best_position.longitude
            geopoint.altitude = best_position.altitude
            self.latlon_pub.publish(geopoint)
    
    def _imu_callback(self, msg):
        """Translate PX4 SensorCombined into standard ROS 2 Imu"""
        if self.use_sim:
            std_msg = msg
        else:
            std_msg = Imu()
            # Gyro (Angular Velocity) in rad/s
            std_msg.angular_velocity.x = float(msg.gyro_rad[0])
            std_msg.angular_velocity.y = float(msg.gyro_rad[1])
            std_msg.angular_velocity.z = float(msg.gyro_rad[2])
            
            # Accelerometer (Linear Acceleration) in m/s^2
            std_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
            std_msg.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
            std_msg.linear_acceleration.z = float(msg.accelerometer_m_s2[2])
            
        self.imu_pub.publish(std_msg)
    
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
    
    def _odom_callback(self, msg):
        """Handle odometry from either Simulator (Standard) or PX4 (px4_msgs)"""
        
        if self.use_sim:
            # SIMULATION: Message is already standard nav_msgs/Odometry
            std_msg = msg
        else:
            # REAL HARDWARE: Translate px4_msgs/VehicleOdometry to standard
            std_msg = Odometry()
            
            # Position (x, y, z)
            std_msg.pose.pose.position.x = float(msg.position[0])
            std_msg.pose.pose.position.y = float(msg.position[1])
            std_msg.pose.pose.position.z = float(msg.position[2])
            
            # Orientation Quaternion (PX4 order is w, x, y, z)
            std_msg.pose.pose.orientation.w = float(msg.q[0])
            std_msg.pose.pose.orientation.x = float(msg.q[1])
            std_msg.pose.pose.orientation.y = float(msg.q[2])
            std_msg.pose.pose.orientation.z = float(msg.q[3])
            
            # Velocity (vx, vy, vz)
            std_msg.twist.twist.linear.x = float(msg.velocity[0])
            std_msg.twist.twist.linear.y = float(msg.velocity[1])
            std_msg.twist.twist.linear.z = float(msg.velocity[2])
        
        # Store and publish the standard message
        self.latest_odom = std_msg
        self.odom_pub.publish(std_msg)
        
        # Compute heading and speed based on the standard message
        self._compute_and_publish_derived_odom(std_msg) 
        
    def _compute_and_publish_derived_odom(self, std_msg: Odometry):
        """Helper function to calculate SMaRC derived values"""
        # Extract heading from odom orientation (quaternion to yaw)
        orientation_list = [
            std_msg.pose.pose.orientation.x,
            std_msg.pose.pose.orientation.y,
            std_msg.pose.pose.orientation.z,
            std_msg.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        # Convert to degrees and normalize to [0, 360)
        heading_deg = math.degrees(yaw)
        if heading_deg < 0:
            heading_deg += 360.0
        
        # Only publish Odometry heading if RTK heading is missing
        if not self.is_receiving_rtk_heading:
            heading_msg = Float32()
            heading_msg.data = 90.0 - heading_deg
            self.heading_pub.publish(heading_msg)
        
        # Compute and publish course (direction of travel in degrees)
        vx = std_msg.twist.twist.linear.x
        vy = std_msg.twist.twist.linear.y
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
    
    def _battery_callback(self, msg):
        """Translate PX4 BatteryStatus into a standard Float32 percentage"""
        std_msg = Float32()
        
        if self.use_sim:
            std_msg.data = msg.data
        else:
            # Convert 0.0-1.0 ratio to 0-100%
            std_msg.data = float(msg.remaining * 100.0)
            
        self.battery_pub.publish(std_msg)


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
