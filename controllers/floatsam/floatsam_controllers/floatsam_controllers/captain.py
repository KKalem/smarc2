#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# FloatSam Captain - Integrated controller for surface USV

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

from smarc_msgs.msg import Topics as SmarcTopics
from smarc_msgs.msg import FloatStamped
from smarc_control_msgs.msg import Topics as ControlTopics
from floatsam_msgs.msg import Topics as FloatsamTopics

# Import PID class and geometry utilities from local package
from floatsam_controllers.pid import PID
import floatsam_controllers.geometry as geom


class Captain(Node):
    """
    Captain is the main controller for FloatSam USV.
    
    It integrates:
    - 3 PID controllers (yaw, yaw_rate, velocity) as Python objects
    - Control signal mixing (differential thrust)
    - Delta RPM rate limiting for safety
    
    All PID gains are configurable via launch file parameters.
    """
    def __init__(self):
        super().__init__("captain_node")
        self.logger = self.get_logger()
        self.logger.info("Initializing FloatSam Captain node!")

        self.declare_node_parameters()

        self.update_rate = float(self.get_parameter("update_rate").value)
        self.logger.info(f"Update rate: {self.update_rate} Hz")
        self.robot_name = self.get_parameter("robot_name").value

        # ============================================
        # Initialize PID controllers as Python objects
        # ============================================
        
        # Yaw PID: converts heading error to yaw_rate setpoint
        self.yaw_pid = PID(
            kP=self.get_parameter("yaw_p_gain").value,
            kI=self.get_parameter("yaw_i_gain").value,
            kD=self.get_parameter("yaw_d_gain").value,
            max_output=self.get_parameter("yaw_output_limit").value
        )
        
        # Yaw Rate PID: converts yaw_rate error to yaw actuation
        self.yawrate_pid = PID(
            kP=self.get_parameter("yawrate_p_gain").value,
            kI=self.get_parameter("yawrate_i_gain").value,
            kD=self.get_parameter("yawrate_d_gain").value,
            max_output=self.get_parameter("yawrate_output_limit").value
        )
        
        # Velocity PID: converts velocity error to RPM setpoint
        self.velocity_pid = PID(
            kP=self.get_parameter("velocity_p_gain").value,
            kI=self.get_parameter("velocity_i_gain").value,
            kD=self.get_parameter("velocity_d_gain").value,
            max_output=self.get_parameter("velocity_output_limit").value
        )
        
        self.logger.info("Initialized 3 PID controllers with configurable gains")

        # ============================================
        # Mixer parameters
        # ============================================
        
        self.rpm_deadband = self.get_parameter("rpm_deadband").value
        self.thruster_limit = self.get_parameter("thruster_limit").value
        
        # Delta RPM rate limiting (health check)
        self.max_delta_rpm = self.get_parameter("max_delta_rpm").value
        self.last_thruster_port_cmd = 0.0
        self.last_thruster_strb_cmd = 0.0
        
        # ============================================
        # State variables for sensor feedback
        # ============================================
        
        self.yaw_measurement = 0.0
        self.yaw_rate_measurement = 0.0
        self.velocity_measurement = 0.0
        
        self.yaw_setpoint = 0.0
        self.velocity_setpoint = 0.0
        
        # Timeouts for safety
        self.last_yaw_meas_time = 0.0
        self.last_yawrate_meas_time = 0.0
        self.last_velocity_meas_time = 0.0
        self.last_yaw_setpoint_time = 0.0
        self.last_velocity_setpoint_time = 0.0

        # ============================================
        # Subscribers: Sensor feedback from odom_splitter
        # ============================================
        
        self.create_subscription(Float32, ControlTopics.CONTROL_YAW_TOPIC,
                                 self.yaw_meas_cb, 1)
        self.create_subscription(Float32, ControlTopics.CONTROL_YAW_RATE_TOPIC,
                                 self.yawrate_meas_cb, 1)
        self.create_subscription(Float32, ControlTopics.CONTROL_SURGE_RATE_TOPIC,
                                 self.velocity_meas_cb, 1)
        
        # ============================================
        # Subscribers: Setpoints from behavior layer
        # ============================================
        
        self.create_subscription(FloatStamped, FloatsamTopics.YAW_SETPOINT,
                                 self.yaw_setpoint_cb, 1)
        self.create_subscription(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT,
                                 self.velocity_setpoint_cb, 1)

        # ============================================
        # Publishers: Thruster commands
        # ============================================
        
        self.thruster_port_msg = Float32()
        self.thruster_strb_msg = Float32()
        
        self.thruster_port_pub = self.create_publisher(Float32,
                                                       FloatsamTopics.THRUSTER_PORT_CMD, 1)
        self.thruster_strb_pub = self.create_publisher(Float32,
                                                       FloatsamTopics.THRUSTER_STRB_CMD, 1)

    def time_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def declare_node_parameters(self):
        """Declare all configurable parameters for PIDs and mixer"""
        self.declare_parameter("robot_name", "floatsam_usv")
        self.declare_parameter("update_rate", 20.0)
        
        # Yaw PID parameters
        self.declare_parameter("yaw_p_gain", 0.15)
        self.declare_parameter("yaw_i_gain", 0.0)
        self.declare_parameter("yaw_d_gain", 0.0)
        self.declare_parameter("yaw_output_limit", 0.1)  # rad/s
        
        # Yaw Rate PID parameters
        self.declare_parameter("yawrate_p_gain", 20.0)
        self.declare_parameter("yawrate_i_gain", 0.0)
        self.declare_parameter("yawrate_d_gain", 0.0)
        self.declare_parameter("yawrate_output_limit", 800.0)  # RPM
        
        # Velocity PID parameters
        self.declare_parameter("velocity_p_gain", 500.0)
        self.declare_parameter("velocity_i_gain", 10.0)
        self.declare_parameter("velocity_d_gain", 0.0)
        self.declare_parameter("velocity_output_limit", 800.0)  # RPM
        
        # Mixer parameters
        self.declare_parameter("rpm_deadband", 50.0)  # RPM
        self.declare_parameter("thruster_limit", 1000.0)  # RPM
        self.declare_parameter("max_delta_rpm", 200.0)  # RPM per control cycle

    # ============================================
    # Callbacks: Sensor measurements
    # ============================================
    
    def yaw_meas_cb(self, msg):
        self.last_yaw_meas_time = self.time_now()
        self.yaw_measurement = msg.data
        self.logger.info("pisello 3 ")


    def yawrate_meas_cb(self, msg):
        self.last_yawrate_meas_time = self.time_now()
        self.yaw_rate_measurement = msg.data
        self.logger.info("pisello 4 ")


    def velocity_meas_cb(self, msg):
        self.last_velocity_meas_time = self.time_now()
        self.velocity_measurement = msg.data
        self.logger.info("pisello 5 ")


    # ============================================
    # Callbacks: Setpoints from behavior layer
    # ============================================
    
    def yaw_setpoint_cb(self, msg):
        self.last_yaw_setpoint_time = self.time_now()
        self.yaw_setpoint = msg.data
        self.logger.info("pisello 1 ")

    def velocity_setpoint_cb(self, msg):
        self.last_velocity_setpoint_time = self.time_now()
        self.velocity_setpoint_input = msg.data
        self.logger.info("pisello 2 ")



    # ============================================
    # Rate limiter (delta RPM health check)
    # ============================================
    
    def apply_rate_limit(self, new_cmd, last_cmd, name):
        """
        Limit the rate of change of thruster commands.
        
        Args:
            new_cmd: Newly computed thruster command (RPM)
            last_cmd: Previous thruster command (RPM)
            name: Thruster name for logging
            
        Returns:
            Rate-limited command (RPM)
        """
        delta = new_cmd - last_cmd
        
        if abs(delta) > self.max_delta_rpm:
            # Limit the change
            limited_delta = self.max_delta_rpm if delta > 0 else -self.max_delta_rpm
            limited_cmd = last_cmd + limited_delta
            
            self.logger.warn(
                f"{name}: Delta RPM {delta:.1f} exceeds limit {self.max_delta_rpm:.1f}. "
                f"Limiting to {limited_cmd:.1f}",
                throttle_duration_sec=2.0
            )
            return limited_cmd
        
        return new_cmd

    # ============================================
    # Main control update loop
    # ============================================



    def update(self):
        """
        Main control loop: Run PIDs, mix signals, apply safety limits.
        
        Control flow:
        1. Check timeouts on all inputs
        2. Run Yaw PID (angle → rate)
        3. Run Yaw Rate PID (rate → actuation)
        4. Run Velocity PID (velocity → RPM)
        5. Mix yaw actuation + velocity RPM into differential thrust
        6. Apply delta RPM rate limiting
        7. Apply saturation and deadband
        8. Publish thruster commands
        """
        now = self.time_now()
        timeout = 1.0  # seconds
        
        # ============================================
        # Safety: Check all input timeouts
        # ============================================
        
        measurements_ok = (
            (now - self.last_yaw_meas_time) < timeout and
            (now - self.last_yawrate_meas_time) < timeout and
            (now - self.last_velocity_meas_time) < timeout
        )
        
        self.logger.info(f"meas {measurements_ok}")

        setpoints_ok = (
            (now - self.last_yaw_setpoint_time) < timeout and
            (now - self.last_velocity_setpoint_time) < timeout
        )
        
        self.logger.info(f"setpoint {setpoints_ok}")
        
        if not measurements_ok or not setpoints_ok:
            # Safety: stop thrusters if we lose any input
            self.logger.warn("Control input timeout - stopping thrusters", throttle_duration_sec=1.0)
            self.thruster_port_msg.data = 0.0
            self.thruster_strb_msg.data = 0.0
            self.thruster_port_pub.publish(self.thruster_port_msg)
            self.thruster_strb_pub.publish(self.thruster_strb_msg)
            
            # Reset last commands for next cycle
            self.last_thruster_port_cmd = 0.0
            self.last_thruster_strb_cmd = 0.0
            return

        # ============================================
        # PID Control Cascade
        # ============================================
        
        # Step 1: Yaw PID - convert angle error to yaw_rate setpoint
        # Use vector-based angle difference for wraparound handling
        setpoint_vec = np.array([np.cos(self.yaw_setpoint), np.sin(self.yaw_setpoint)])
        measurement_vec = np.array([np.cos(self.yaw_measurement), np.sin(self.yaw_measurement)])
        yaw_error = -geom.vec2_directed_angle(setpoint_vec, measurement_vec)
        
        yaw_rate_setpoint = self.yaw_pid.update_error(yaw_error, now)
        
        # Step 2: Yaw Rate PID - convert rate error to actuation signal
        yaw_rate_error = yaw_rate_setpoint - self.yaw_rate_measurement
        yaw_actuation = self.yawrate_pid.update_error(yaw_rate_error, now)
        
        # Step 3: Velocity PID - convert velocity error to RPM setpoint
        velocity_error = self.velocity_setpoint_input - self.velocity_measurement
        velocity_rpm_setpoint = self.velocity_pid.update_error(velocity_error, now)

        # ============================================
        # Mixing: Differential thrust
        # ============================================
        
        yaw_correction = yaw_actuation
        
        # Base RPM for both thrusters, then add/subtract for steering
        thruster_port_raw = velocity_rpm_setpoint - yaw_correction
        thruster_strb_raw = velocity_rpm_setpoint + yaw_correction

        # ============================================
        # Health Check: Delta RPM rate limiting
        # ============================================
        
        thruster_port = self.apply_rate_limit(
            thruster_port_raw, 
            self.last_thruster_port_cmd,
            "Port"
        )
        
        thruster_strb = self.apply_rate_limit(
            thruster_strb_raw,
            self.last_thruster_strb_cmd,
            "Starboard"
        )

        # ============================================
        # Saturation and deadband
        # ============================================
        
        # Apply thruster limits
        thruster_port = max(-self.thruster_limit, min(self.thruster_limit, thruster_port))
        thruster_strb = max(-self.thruster_limit, min(self.thruster_limit, thruster_strb))

        # Apply deadband to avoid very small commands that might stall motors
        if abs(thruster_port) < self.rpm_deadband:
            thruster_port = 0.0
        if abs(thruster_strb) < self.rpm_deadband:
            thruster_strb = 0.0

        # ============================================
        # Publish and save for next cycle
        # ============================================
        
        self.thruster_port_msg.data = thruster_port
        self.thruster_strb_msg.data = thruster_strb
        
        self.thruster_port_pub.publish(self.thruster_port_msg)
        self.thruster_strb_pub.publish(self.thruster_strb_msg)
        
        # Save for delta RPM calculation next cycle
        self.last_thruster_port_cmd = thruster_port
        self.last_thruster_strb_cmd = thruster_strb

        # Debug logging (uncomment for detailed feedback)
        # self.logger.info(
        #     f"Yaw: {np.rad2deg(self.yaw_measurement):.1f}° → {np.rad2deg(self.yaw_setpoint):.1f}°, "
        #     f"Vel: {self.velocity_measurement:.2f} → {self.velocity_setpoint_input:.2f} m/s | "
        #     f"Thrusters: P={thruster_port:.0f}, S={thruster_strb:.0f} RPM",
        #     throttle_duration_sec=0.5
        # )


def main(args=None, namespace=None):
    rclpy.init(args=args)
    captain_node = Captain()

    captain_node.create_timer(1.0/captain_node.update_rate, captain_node.update)
    executor = MultiThreadedExecutor()
    executor.add_node(captain_node)
    executor.spin()

if __name__ == '__main__':
    main()
