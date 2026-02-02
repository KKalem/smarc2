# This file defines all the topics used on FloatSam USV

# FloatSam Abort
string FLOATSAM_ABORT = 'abort'

# PID controllers setpoints. All Float32s.
string YAW_SETPOINT = 'ctrl/yaw_setpoint'
string VELOCITY_SETPOINT = 'ctrl/velocity_setpoint'

# Thruster commands. All Float32s (RPM).
string THRUSTER_PORT_CMD = 'actuators/thruster_port_cmd'
string THRUSTER_STRB_CMD = 'actuators/thruster_strb_cmd'

# Thruster feedback. All Float32s (RPM).
string THRUSTER_PORT_FB = 'actuators/thruster_port_fb'
string THRUSTER_STRB_FB = 'actuators/thruster_strb_fb'
