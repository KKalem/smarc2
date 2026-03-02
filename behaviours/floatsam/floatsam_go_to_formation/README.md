# floatsam_go_to_formation

A ROS 2 action server that commands a fleet of Floatsam USVs into a geographic formation using a **py_trees behavior tree**. Each robot is assigned an optimal formation slot via the Hungarian algorithm, and the fleet then navigates and loiters at the assigned geopoints with the requested headings.

---

## Overview

```
Goal (JSON via BaseAction)
    └── formation_points: [{latitude, longitude, heading}, ...]

Action server: /go_to_formation
Action type:   smarc_action_base/action/BaseAction
```

When a goal is accepted the server:

1. Waits for all robots to publish odometry.
2. Builds a py_trees behavior tree and ticks it at **10 Hz**.
3. Runs the **Hungarian assignment** to map each robot to the nearest formation slot.
4. Handles **collision avoidance** (deconfliction with priority/counter logic).
5. Sends `move_to_path` action goals to drive each robot to its assigned slot.
6. Once every robot arrives, sends `loiter_with_heading` action goals to hold position and orientation.

---

## Behavior Tree Structure

```
MainTree  [Sequence]
├── FirstSelector  [Selector]
│   ├── HaveGoal              – succeeds if assignments already exist
│   └── HungarianAssignment   – assigns robots to slots via optimal assignment
├── SecondSelector  [Selector]
│   ├── CollisionCheck        – succeeds if no collision risk
│   └── ThirdSelector  [Selector]
│       ├── PriorityCheck     – succeeds if this robot has right-of-way
│       └── FourthSelector  [Selector]
│           ├── FirstSequence  [Sequence]
│           │   ├── CounterCheck
│           │   └── MoveToSide
│           └── Wait
└── SecondSequence  [Sequence]
    ├── FifthSelector  [Selector]
    │   ├── ArrivalCheck        – succeeds if robot already at target
    │   └── MoveToPathClient    – calls move_to_path action server
    └── SixthSelector  [Selector]
        ├── AllArrivalCheck          – succeeds when every robot has arrived
        └── LoiterWithHeadingClient  – calls loiter_with_heading action server
```

---

## Package Structure

```
floatsam_go_to_formation/
├── floatsam_go_to_formation/
│   ├── __init__.py
│   ├── behaviours.py                       # All py_trees behaviour classes
│   ├── floatsam_common.py                  # FloatSam helper (TF, geo conversion)
│   └── floatsam_go_to_formation_server.py  # BTActionServer node (entry point)
├── launch/
│   └── floatsam_go_to_formation.launch.py
├── package.xml
└── setup.py
```

---

## Dependencies

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS 2 Python client |
| `py_trees` / `py_trees_ros` | Behavior tree framework |
| `smarc_action_base` | `GentlerActionServer` + `BaseAction` message type |
| `smarc_utilities` | `georef_utils.convert_latlon_to_utm` |
| `scipy` | `linear_sum_assignment` (Hungarian algorithm) |
| `nav_msgs`, `geometry_msgs`, `geographic_msgs` | ROS 2 message types |
| `tf2_ros`, `tf2_geometry_msgs` | Coordinate frame transforms |

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `robot_name` | `floatsam_usv` | Base robot name. Robots are expected as `<robot_name>_0`, `<robot_name>_1`, … |
| `num_robots` | `3` | Total number of USVs in the fleet. IDs are `0 .. num_robots-1`. |

---

## Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/<robot_name>_N/smarc/odom_gt` | `nav_msgs/Odometry` | Ground-truth odometry for each robot (one per robot) |

---

## Action Server

| Name | Type | Description |
|---|---|---|
| `/go_to_formation` | `smarc_action_base/action/BaseAction` | Accepts a JSON-encoded formation goal |

### Goal Format

The goal is a JSON string carried in the `BaseAction.Goal.goal.data` field:

```json
{
  "formation_points": [
    { "latitude": <float>, "longitude": <float>, "heading": <float> },
    { "latitude": <float>, "longitude": <float>, "heading": <float> },
    ...
  ]
}
```

| Field | Type | Range | Description |
|---|---|---|---|
| `latitude` | float | −90 … 90 | WGS-84 latitude in decimal degrees |
| `longitude` | float | −180 … 180 | WGS-84 longitude in decimal degrees |
| `heading` | float | 0 … 360 | Desired robot heading in degrees (0 = North, 90 = East) |

The list must contain **exactly as many points as there are robots** (`num_robots`). The server uses an optimal assignment so the order of points does not matter.

---

## Build

```bash
cd <workspace_root>
colcon build --packages-select floatsam_go_to_formation
source install/setup.bash
```

---

## Running

### Via launch file (recommended)

```bash
# Default: floatsam_usv_0 and floatsam_usv_1  (num_robots=2)
ros2 launch floatsam_go_to_formation floatsam_go_to_formation.launch.py

# Three robots with a custom base name
ros2 launch floatsam_go_to_formation floatsam_go_to_formation.launch.py \
    robot_name:=floatsam_usv \
    num_robots:=3
```

### Directly

```bash
ros2 run floatsam_go_to_formation floatsam_go_to_formation_action_server \
    --ros-args -p robot_name:=floatsam_usv -p num_robots:=3
```

---

## Sending Goals — CLI Examples

All examples use `ros2 action send_goal`. The goal string is a YAML representation of the `BaseAction.Goal` message; the `data` field contains the JSON payload.

### Two robots — simple triangle formation

```bash
ros2 action send_goal /floatsam_usv_0/go_to_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"formation_points\": [{\"latitude\": 58.8405360306434, \"longitude\": 17.6520998616085, \"heading\": 0.0}, {\"latitude\": 58.8405960738135, \"longitude\": 17.6518292606662, \"heading\": 0.0}]}'}}"
```

### Three robots — line abreast, facing east (90°)

```bash
ros2 action send_goal /floatsam_usv_0/go_to_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"formation_points\": [{\"latitude\": 58.8405258584503, \"longitude\": 17.6516992496307, \"heading\": 90.0}, {\"latitude\": 58.8405428888343, \"longitude\": 17.6518663726091, \"heading\": 90.0}, {\"latitude\": 58.8405878677775, \"longitude\": 17.6517617503888, \"heading\": 90.0}]}'}}"
```

### Three robots — wedge formation, facing north (0°)

```bash
ros2 action send_goal /floatsam_usv_0/go_to_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"formation_points\": [{\"latitude\": 57.7092, \"longitude\": 11.9452, \"heading\": 0.0}, {\"latitude\": 57.7090, \"longitude\": 11.9450, \"heading\": 0.0}, {\"latitude\": 57.7090, \"longitude\": 11.9454, \"heading\": 0.0}]}'}}"
```

### Cancel a running goal

```bash
# List active goals first
ros2 action show /floatsam_usv_0/go_to_formation

# Cancel by goal UUID (replace <UUID> with actual value from above)
ros2 action cancel /floatsam_usv_0/go_to_formation <UUID>
```

---

## Calling the Server from Python

```python
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smarc_action_base.action import BaseAction
from std_msgs.msg import String


class FormationClient(Node):
    def __init__(self):
        super().__init__('formation_client')
        self._client = ActionClient(self, BaseAction, 'go_to_formation')

    def send_formation(self, formation_points: list[dict]):
        """
        formation_points: list of dicts with keys latitude, longitude, heading.
        E.g. [{'latitude': 57.71, 'longitude': 11.95, 'heading': 90.0}, ...]
        """
        goal_msg = BaseAction.Goal()
        goal_msg.goal = String(data=json.dumps({'formation_points': formation_points}))

        self._client.wait_for_server()
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.get_logger().info(f'Result: {f.result().result.result.data}')
        )


def main():
    rclpy.init()
    client = FormationClient()

    # Three-robot line formation facing east
    client.send_formation([
        {'latitude': 57.7089, 'longitude': 11.9450, 'heading': 90.0},
        {'latitude': 57.7090, 'longitude': 11.9450, 'heading': 90.0},
        {'latitude': 57.7091, 'longitude': 11.9450, 'heading': 90.0},
    ])

    rclpy.spin(client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Notes

- The number of formation points **must equal `num_robots`**. Providing more or fewer points will result in goal rejection.
- Heading `0°` = North, `90°` = East, `180°` = South, `270°` = West.
- The server waits up to **5 seconds** at start for all robot odometry topics to publish before accepting the goal. If any robot is missing its position data the goal is aborted.
- Collision avoidance (`CollisionCheck`, `PriorityCheck`, `MoveToSide`) and individual arrival checking (`ArrivalCheck`, `AllArrivalCheck`) are currently scaffolded and marked as `TODO`.


