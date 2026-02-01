# LoLo External Equipment

Packages for the LoLo AUV platform.

## lolo_topic_bridge

Topic bridge node that converts LoLo-specific topics (from simulator or real hardware) to standard SMaRC topics.

### Features

- **Switch between simulation and real hardware** via `use_sim` parameter
- **Configurable via YAML files** for easy topic mapping updates
- **Smart handling of simulator topics**: The LoLo simulator already publishes many topics in SMaRC format, so this node acts as a passthrough for those
- **Converts raw sensor data** and republishes SMaRC-formatted topics:
  - All `/lolo_auv_v1/smarc/*` topics → `smarc/*` (passthrough)
  - Raw sensors → `lolo/raw/*` for logging/debugging

### Usage

**Simulation mode (default):**
```bash
ros2 launch lolo_topic_bridge lolo_bridge.launch.py use_sim:=true
```

**Real hardware mode:**
```bash
ros2 launch lolo_topic_bridge lolo_bridge.launch.py use_sim:=false
```

⚠️ **Note:** Before using real hardware mode, update [config/real_topics.yaml](lolo_topic_bridge/config/real_topics.yaml) with actual hardware topic names!

### Configuration Files

- **[sim_topics.yaml](lolo_topic_bridge/config/sim_topics.yaml)** - Simulator topic mappings (ready to use)
- **[real_topics.yaml](lolo_topic_bridge/config/real_topics.yaml)** - Real hardware topic mappings (⚠️ PLACEHOLDERS - must be updated!)

### Topics Published

**Standard SMaRC topics** (from [smarc_msgs/msg/Topics.msg](/messages/smarc_msgs/msg/Topics.msg)):

- `smarc/altitude` (std_msgs/Float32)
- `smarc/battery_percent` (std_msgs/Float32)
- `smarc/course` (std_msgs/Float32)
- `smarc/depth` (std_msgs/Float32)
- `smarc/heading` (std_msgs/Float32)
- `smarc/latlon` (geographic_msgs/GeoPoint)
- `smarc/odom` (nav_msgs/Odometry)
- `smarc/speed` (std_msgs/Float32)

**Raw sensor topics** (for logging/debugging):

- `lolo/raw/gps` (sensor_msgs/NavSatFix)
- `lolo/raw/imu` (sensor_msgs/Imu)
- `lolo/raw/depth_pressure` (sensor_msgs/FluidPressure)
- `lolo/raw/dvl` (sensor_msgs/Range)
- `lolo/leak_status` (std_msgs/Bool)
