# floatsam_go_to_formation

A ROS 2 action server that commands a fleet of Floatsam USVs into a geographic formation using a **py_trees behavior tree**. Each robot is assigned an optimal formation slot via the Hungarian algorithm, and the fleet navigates to and loiters at the assigned geopoints with synchronized speeds and active collision avoidance.

---

## Overview

    Goal (JSON via BaseAction)
        └── formation_points: [{latitude, longitude, heading}, ...]

    Action server: /go_to_formation
    Action type:   smarc_action_base/action/BaseAction

When a goal is accepted, the server:

1. Waits for all robots to publish valid odometry in the map frame.
2. Builds a py_trees behavior tree and ticks it at **10 Hz**.
3. Runs the **Hungarian algorithm** to optimally assign each robot to a formation slot.
4. Continuously evaluates **collision avoidance**, determining right-of-way priority and executing evasion maneuvers when necessary.
5. Sends `move_path` action goals to drive each robot to its assigned slot, dynamically overriding speeds to ensure synchronized arrival.
6. Once every robot arrives, sends `loiter_heading` action goals to hold position and orientation for 400 seconds.

---

## Core Functionalities

### 1. Optimal Target Assignment
The `HungarianAssignment` behavior maps each robot to a formation point. It computes a cost matrix based on the squared Euclidean distance between every robot's current position and every goal point, ensuring the fleet travels the minimum collective distance.

### 2. Collision Avoidance Policy
Unlike previous versions, collision avoidance is now fully implemented with a multi-step deconfliction policy:
* **Detection (`CollisionFreeCheck`)**: The system continuously calculates the distance between robots. If another robot enters the defined `collision_radius`, a potential collision is flagged.
* **Right-of-Way (`PriorityCheck`)**: When a collision risk is detected, priority is determined by projecting both robots' positions onto a line relative to the approach direction to the formation center. The robot with the smaller projection distance gets priority, while the other must yield.
* **Yielding (`Wait` & `CounterCheck`)**: The yielding robot pauses by publishing a zero-velocity setpoint. A counter tracks how long the robot has been waiting.
* **Evasion (`MoveToSide`)**: If the yielding robot waits longer than the `max_num_collisions` threshold, it breaks the deadlock. It evaluates clearance on its left and right sides, selects the safest evasion point 3.0 meters away, and sends a constant-speed `move_to` goal to maneuver out of the way.

### 3. Synchronized Speed Control
To ensure the fleet arrives simultaneously, the `MoveToPathClient` dynamically scales each robot's speed. 
* The system identifies the robot furthest from its goal and commands it to travel at `max_velocity`.
* All other robots scale their velocity proportionally based on their relative distance to their respective goals (e.g., `(my_distance / max_distance) * max_velocity`).

### 4. Arrival and Loitering
The `AllArrivalCheck` behavior monitors the loiter feedback for all robots. Once every robot confirms arrival, the `LoiterWithHeadingClient` executes, keeping the robots at their designated coordinates and headings for 400 seconds.

---

## Behavior Tree Structure

    MainTree  [Sequence]
    ├── FirstSelector  [Selector]
    │   ├── HaveGoal              – succeeds if assignments already exist
    │   └── HungarianAssignment   – assigns robots to slots via optimal assignment
    ├── SecondSelector  [Selector]
    │   ├── CollisionFreeCheck    – succeeds if no collision risk
    │   └── ThirdSelector  [Selector]
    │       ├── PriorityCheck     – succeeds if this robot has right-of-way
    │       └── FourthSelector  [Selector]
    │           ├── FirstSequence  [Sequence]
    │           │   ├── CounterCheck
    │           │   └── MoveToSide
    │           └── Wait          – stops the robot if yielding
    └── SecondSequence  [Sequence]
        ├── FifthSelector  [Selector]
        │   ├── ArrivalCheck        – succeeds if robot already at target
        │   └── MoveToPathClient    – calls move_path with synchronized speed
        └── SixthSelector  [Selector]
            ├── AllArrivalCheck          – succeeds when every robot has arrived
            └── LoiterWithHeadingClient  – calls loiter_heading action server

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `robot_name` | `floatsam_usv_0` | Base name of the robot executing the server. |
| `num_robots` | `3` | Total number of USVs in the fleet. IDs are `0 .. num_robots-1`. |
| `collision_radius` | `1.0` | Distance (meters) to trigger collision avoidance. |
| `max_num_collisions` | `3` | Number of wait cycles before triggering a side evasion maneuver. |
| `waypoints_step_size` | `0.5` | Distance (meters) between generated path waypoints. |
| `max_velocity` | `2.0` | Maximum speed override for the fleet. |

---

## Running and Activating the Server

### 1. Launch the Server

```bash
# Default parameters
ros2 launch floatsam_go_to_formation floatsam_go_to_formation.launch.py

# Custom fleet size
ros2 launch floatsam_go_to_formation floatsam_go_to_formation.launch.py \
    robot_name:=floatsam_usv \
    num_robots:=5

```
### 2. Send a Formation Mission (Terminal Command)

You can activate the server and assign a mission using the ROS 2 CLI. The goal string must be a YAML representation containing the JSON payload. Ensure the number of coordinate sets exactly matches num_robots.

#### Example: Five-Robot Formation Mission

ros2 action send_goal /floatsam_usv_0/go_to_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"formation_points\": [{\"latitude\": 58.8408761736411, \"longitude\": 17.6513505304756, \"heading\": 90.0}, {\"latitude\": 58.8410112590599, \"longitude\": 17.6515537892541, \"heading\": 90.0}, {\"latitude\": 58.8409725040742, \"longitude\": 17.6519172840096, \"heading\": 90.0}, {\"latitude\": 58.8408817099164, \"longitude\": 17.6522351146409, \"heading\": 90.0}, {\"latitude\": 58.8406603387832, \"longitude\": 17.6523177281681, \"heading\": 90.0}]}'}}"

Cancel a Running Goal

#### List active goals first to find the UUID
ros2 action show /floatsam_usv_0/go_to_formation

#### Cancel by goal UUID
ros2 action cancel /floatsam_usv_0/go_to_formation <UUID>