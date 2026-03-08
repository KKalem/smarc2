import numpy as np
import rclpy
from rclpy.node import Node

from .floatsam_common import FloatSam
from floatsam_interfaces.srv import GetSafeVelocity #to be added 
from tf2_geometry_msgs import do_transform_pose_stamped

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics


class RVOservice(Node):
    def __init__(self):
        super().__init__("RVO_service_node")
        self.logger = self.get_logger()
        self.srv = self.create_service(GetSafeVelocity, 'get_safe_velocity', self.compute_safe_velocity_callback)
        self.get_logger().info('RVO Safe Velocity Service is ready.')
        self.declare_node_parameters()
        self.get_node_parametrs()
        self._floatsam = FloatSam(self, self.this_robot_name)

        self._odom_subscribers = {}
        self._robot_positions = {}
        self._robot_velocities = {}
        self._odometry_subscriptions()
        self.create_subscription(FloatStamped, FloatsamTopics.YAW_SETPOINT, self.yaw_setpoint_cb, 1)
        self.create_subscription(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, self.velocity_setpoint_cb, 1)

        speed_samples = np.arange(0.5, self.max_speed + 0.5, 0.5)  
        self.velocity_sample = self.velocity_samples(speed_samples)
        

    
    def compute_safe_velocity_callback(self, request, response):
        # request.robot_id
        # request.pref_velocity

        self.safety_margin = 2 * self.safety_margin / self.time_horizon

        self.get_logger().info('The service has been activated')

        self.this_robot_id = request.robot_id
        self.pref_velocity = request.pref_velocity

        this_robot_position = self._robot_positions[self.this_robot_name].pose.position
        this_robot_velocity = self._robot_velocities[self.this_robot_name]
        self.this_robot_position = np.array([this_robot_position.x, this_robot_position.y])
        self.this_robot_velocity = np.array([this_robot_velocity.x, this_robot_velocity.y])

        # --- First check if the preferred velocity is safe against all robots ---
        pref_velocity_vec = np.array(self.pref_velocity)  
        pref_is_safe = True
        for idx in self.robot_ids:
            if f'{self.robot_base_name}_{idx}' == self.this_robot_name:
                continue
            robot_velocity = np.array([self._robot_velocities[f'{self.robot_base_name}_{idx}'].x,
                                       self._robot_velocities[f'{self.robot_base_name}_{idx}'].y])
            v_apex = (self.this_robot_velocity + robot_velocity) / 2
            if self.is_in_cone(idx, v_apex, pref_velocity_vec):
                self.get_logger().info('The DESIRED velocity is NOT safe')
                pref_is_safe = False
                break

        if pref_is_safe:
            self.get_logger().info('The DESIRED velocity is safe')
            pref_speed = float(np.linalg.norm(pref_velocity_vec))
            pref_angle = float(np.arctan2(pref_velocity_vec[1], pref_velocity_vec[0]))

            response.safe_velocity = [pref_speed, pref_angle]
            response.success = True
            response.change = False
            return response

        # --- Search for the closest safe velocity sample ---
        best_velocity = None
        best_distance = np.inf

        for velocity in self.velocity_sample:
            projected_velocity = np.array([velocity[0] * np.cos(velocity[1]), velocity[0] * np.sin(velocity[1])])
            safe = True
            for idx in self.robot_ids:
                if f'{self.robot_base_name}_{idx}' == self.this_robot_name:
                    continue
                robot_velocity = np.array([self._robot_velocities[f'{self.robot_base_name}_{idx}'].x,
                                           self._robot_velocities[f'{self.robot_base_name}_{idx}'].y])
                v_apex = (self.this_robot_velocity + robot_velocity) / 2
                if self.is_in_cone(idx, v_apex, projected_velocity):
                    safe = False
                    break

            if not safe:
                continue

            distance = np.linalg.norm(pref_velocity_vec - projected_velocity)
            if distance < best_distance:
                best_distance = distance
                best_velocity = velocity

        if best_velocity is None:
            response.success = False
            self.get_logger().info('The COMPUTED velocity is NONE')
            return response
        
        self.get_logger().info('The COMPUTED velocity is SAFE')
        response.safe_velocity = best_velocity
        response.success = True
        response.change = True 

        # response.safe_velocity = calculated_velocity
        # response.success = True
        return response


    def is_in_cone(self, idx, v_apex, projected_velocity):
        position = self._robot_positions[f'{self.robot_base_name}_{idx}'].pose.position
        position = np.array([position.x, position.y])

        rp = position - self.this_robot_position  # <-- FIXED: toward the other robot

        distance = np.linalg.norm(rp)

        if distance < 1e-6:
            return True

        #cone_apex = rp / self.time_horizon + v_apex

        relative_velocity = projected_velocity - v_apex

        ratio = np.clip(self.safety_margin / distance, -1.0, 1.0)
        alpha = np.arcsin(ratio)

        return self.comput_angle(rp, relative_velocity) < alpha

        
    def comput_angle(self, v_apex, projected_velocity):
        norm_v_apex = np.linalg.norm(v_apex)
        norm_projected_velocity = np.linalg.norm(projected_velocity)
        if norm_v_apex < 1e-9 or norm_projected_velocity < 1e-9:
            return 0.0  
        dot_product = np.dot(v_apex, projected_velocity)
        cos_theta = dot_product / (norm_v_apex * norm_projected_velocity)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        return np.arccos(cos_theta)

    def velocity_samples(self, speed_samples):
        num_angles = 120
        angles = np.linspace(0, 2*np.pi, num_angles, endpoint=False)

        velocity_samples = [
            (speed, angle)
            for speed in speed_samples
            for angle in angles
        ]
        return velocity_samples


# --- Nodes parameters --- #
    def declare_node_parameters(self):
        """Declare all configurable parameters for PIDs and mixer"""
        self.declare_parameter("robot_name", "floatsam_0")
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
    

# --- msgs Callbacks --- #
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
        self._robot_velocities[f'{self.robot_base_name}_{robot_id}'] = velocity_in_odom

    def yaw_setpoint_cb(self, msg):

        self.last_yaw_setpoint_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.yaw_setpoint = msg.data

    def velocity_setpoint_cb(self, msg):
        self.last_velocity_setpoint_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.velocity_setpoint_input = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = RVOservice()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
