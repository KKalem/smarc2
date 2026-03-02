import py_trees
import numpy as np
import json
from scipy.optimize import linear_sum_assignment
from rclpy.action import ActionClient
from smarc_msgs.action import BaseAction
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from geographic_msgs.msg import GeoPoint


class CollisionPlaceholder():
    def __init__(self):
        self.counter = 0
        self.is_colliding = False

class HaveGoal(py_trees.behaviour.Behaviour):
    """Check if a goal has been assigned to this agent."""
    
    def __init__(self, name="HaveGoal"):
        super().__init__(name)
        self.node = None  
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("formation_points", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Check if formation points exist and assignments have been made."""
        # Check if we have formation points
        if not hasattr(self.blackboard, 'formation_points') or self.blackboard.formation_points is None:
            self.node.get_logger().info(f"{self.name}: No formation points received")
            return py_trees.common.Status.FAILURE
        
        # Check if assignments have been created
        if not hasattr(self.blackboard, 'robot_assignments') or self.blackboard.robot_assignments is None:
            self.node.get_logger().info(f"{self.name}: No robot assignments yet")
            return py_trees.common.Status.FAILURE
        
        # Check if assignments dictionary is not empty
        if len(self.blackboard.robot_assignments) == 0:
            self.node.get_logger().info(f"{self.name}: Robot assignments dictionary is empty")
            return py_trees.common.Status.FAILURE
            
        self.node.get_logger().info(f"{self.name}: Have goal - {len(self.blackboard.robot_assignments)} robots assigned")
        return py_trees.common.Status.SUCCESS


class HungarianAssignment(py_trees.behaviour.Behaviour):
    """Perform Hungarian algorithm to assign targets to agents."""
    
    def __init__(self, name="HungarianAssignment"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("formation_points", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Execute Hungarian assignment algorithm."""
        if not hasattr(self.blackboard, 'formation_points'):
            self.node.get_logger().info(f"{self.name}: The formation points are not available yet")
            return py_trees.common.Status.RUNNING
        if not hasattr(self.blackboard, 'robot_positions'):
            self.node.get_logger().info(f"{self.name}: The robot positions are not available yet")
            return py_trees.common.Status.RUNNING

        try: 
            self.node.get_logger().info(f"{self.name}: Running assignment algorithm...")
            robot_names = sorted(self.blackboard.robot_positions.keys())
            cost_matrix = self.compute_cost_matrix(self.blackboard.formation_points, self.blackboard.robot_positions, robot_names)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            for i in range(len(row_ind)):
                robot_key = robot_names[row_ind[i]]
                task_idx = col_ind[i]
                self.node.get_logger().info(f"{self.name}: {robot_key} assigned to goal_{task_idx}")
                self.blackboard.robot_assignments[robot_key] = f'goal_{task_idx}'

            
            return py_trees.common.Status.SUCCESS
        
        except Exception as e:
            self.node.get_logger().error(f"{self.name}: Exception: {e}")
            return py_trees.common.Status.FAILURE

    def compute_cost_matrix(self, formation_points, robot_positions, robot_names):
        size = len(robot_names)
        cost_matrix = np.zeros((size, size))
        for i, name in enumerate(robot_names):
            for j in range(size):
                cost_matrix[i][j] = self.compute_distance(robot_position=robot_positions[name], goal_position=formation_points[f'goal_{j}'])
        
        return cost_matrix
    
    def compute_distance(self, robot_position, goal_position):
        rx = robot_position.pose.position.x
        ry = robot_position.pose.position.y
        return (rx - goal_position[0])**2 + (ry - goal_position[1])**2


class CollisionFreeCheck(py_trees.behaviour.Behaviour):
    """Check if agent is in collision or potential collision state."""
    
    def __init__(self, name="CollisionFreeCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_radius", access=py_trees.common.Access.READ)
        self.blackboard.register_key("colliding_list", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.this_robot_name = self.blackboard.this_robot_name
        self.collision_radius = self.blackboard.collision_radius
        self.collision_streak = {}
        for robot_name in self.blackboard.robot_positions.keys():
            if robot_name == self.this_robot_name:
                continue
            cp = CollisionPlaceholder()
            self.collision_streak[robot_name] = cp
        self.blackboard.collision_streak = self.collision_streak

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        self.blackboard.colliding_list = []

    def update(self):
        """Check for collision risks between this robot and the others."""
    
        robot_positions = self.blackboard.robot_positions
        self.this_position = robot_positions[self.this_robot_name]
        self.node.get_logger().info(f"{self.name}: Checking for collisions...")

        for key, position in robot_positions.items():
            if key != self.this_robot_name:
                if self.compute_distance(self.this_position, position) < self.collision_radius:
                    self.node.get_logger().info(f"{self.name}: Robot {self.this_robot_name} is colliding with {key}")
                    self.blackboard.colliding_list.append(key)
                else:
                    self.blackboard.collision_streak[key].is_colliding = False 

        if self.blackboard.colliding_list:
            self.node.get_logger().info(f"{self.this_robot_name} is colliding with: {self.blackboard.colliding_list}")
            return py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().info(f"{self.this_robot_name} is collision free")
            return py_trees.common.Status.SUCCESS

    def compute_distance(self, this_robot_position, other_robot_position):
        rx = this_robot_position.pose.position.x
        ry = this_robot_position.pose.position.y
        ox = other_robot_position.pose.position.x
        oy = other_robot_position.pose.position.y
        return np.sqrt((rx - ox)**2 + (ry - oy)**2)


class PriorityCheck(py_trees.behaviour.Behaviour):
    """Check if this agent has priority in collision scenario."""
    
    def __init__(self, name="PriorityCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("colliding_list", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("formation_cluster_centre", access=py_trees.common.Access.READ)
        self.blackboard.register_key("approach_direction", access=py_trees.common.Access.READ)
        
        # signed projection distance along the approach line from formation centre
        self.fc = self.blackboard.formation_cluster_centre
        self.this_robot_name = self.blackboard.this_robot_name
        self.not_priority = False 
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        self.not_priority = False 
            
    def update(self):
        self.node.get_logger().info(f"{self.name}: Checking priority...")
        self.colliding_list = self.blackboard.colliding_list
        self.theta = self.blackboard.approach_direction  # radians
        this_position = self.blackboard.robot_positions[self.this_robot_name]
        if not self.colliding_list:
            self.node.get_logger().error(f"{self.name}: The BT entered the leaf but the colliding_list is empty")
            return py_trees.common.Status.FAILURE
        
        for robot in self.colliding_list:
            if self.priority(this_position, self.blackboard.robot_positions[robot]):
                self.node.get_logger().info(f"{self.this_robot_name} has the priority over {robot}")
                self.blackboard.collision_streak[robot].is_colliding = False
                self.blackboard.collision_streak[robot].counter = 0
            else:
                self.node.get_logger().info(f"{self.this_robot_name} has not the priority over {robot}")
                self.not_priority = True
                if self.blackboard.collision_streak[robot].is_colliding == False: 
                   self.blackboard.collision_streak[robot].is_colliding = True 
                   self.blackboard.collision_streak[robot].counter += 1

        if self.not_priority:
            return py_trees.common.Status.FAILURE
        self.node.get_logger().info(f"{self.name}: ORA RITORNO SUCCESSSSSO ")
        return py_trees.common.Status.SUCCESS
            
    def calculate_proj_dist(self, this_position):
        rx, ry = this_position.pose.position.x, this_position.pose.position.y
        # signed projection distance along the approach line 
        proj = np.abs((rx - self.fc[0]) * np.cos(self.theta) + (ry - self.fc[1]) * np.sin(self.theta))
        return proj
    
    def priority(self, this_position, other_position):
        this_proj = self.calculate_proj_dist(this_position)
        other_proj = self.calculate_proj_dist(other_position)
        if this_proj <= other_proj:
            return True
        else:
            return False


class CounterCheck(py_trees.behaviour.Behaviour):
    """Check counter for collision avoidance maneuvers."""
    
    def __init__(self, name="CounterCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("max_num_collisions", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Check counter value."""
        self.node.get_logger().info(f"{self.name}: Checking counter for collision avoidance...")
        for key in self.blackboard.collision_streak.keys():
            if self.blackboard.collision_streak[key].counter > self.blackboard.max_num_collisions:
                self.node.get_logger().info(f"{self.blackboard.this_robot_name} has reached the collision streak with {key}")
                return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE


class MoveToSide(py_trees.behaviour.Behaviour):
    """Execute a side movement for collision avoidance.

    Computes a perpendicular evasion waypoint on the side that is free of
    collisions, then sends a move_to goal (with constant_speed=True so the
    robot does not decelerate on approach).
    """

    # How far (metres) from the current position the evasion point is placed
    EVASION_DISTANCE = 3.0
    # Tolerance (metres) for the evasion waypoint
    EVASION_TOLERANCE = 2.0

    def __init__(self, name="MoveToSide"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("colliding_list", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("collision_radius", access=py_trees.common.Access.READ)

        self._action_client = None

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self._floatsam = kwargs['node']._floatsam
        self._action_client = ActionClient(self.node, BaseAction, 'move_to')
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        """Reset action-client state each time the BT enters this node."""
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._goal_accepted = False
        self._goal_done = False
        self._goal_succeeded = False

    def update(self):
        """Compute collision-free side, send evasion goal via move_to."""

        # ── 1. Send goal (first tick) ────────────────────────────────────────
        if self._send_goal_future is None:
            my_name = self.blackboard.this_robot_name
            my_pos = self.blackboard.robot_positions.get(my_name)
            if my_pos is None:
                self.node.get_logger().warn(f"{self.name}: No position for {my_name}")
                return py_trees.common.Status.FAILURE

            rx = my_pos.pose.position.x
            ry = my_pos.pose.position.y

            # Determine colliding robots
            colliding_list = self.blackboard.colliding_list
            if not colliding_list:
                self.node.get_logger().warn(f"{self.name}: No colliding robots")
                return py_trees.common.Status.FAILURE

            # Average direction toward all colliding robots
            dx_sum, dy_sum = 0.0, 0.0
            for other_name in colliding_list:
                other_pos = self.blackboard.robot_positions.get(other_name)
                if other_pos is not None:
                    dx_sum += other_pos.pose.position.x - rx
                    dy_sum += other_pos.pose.position.y - ry

            if abs(dx_sum) < 1e-6 and abs(dy_sum) < 1e-6:
                # Robots are on top of each other; pick an arbitrary direction
                dx_sum, dy_sum = 1.0, 0.0

            # Heading toward collision cluster
            heading_to_collision = np.arctan2(dy_sum, dx_sum)

            # Two candidate evasion headings: ±90° from collision direction
            left_heading = heading_to_collision + np.pi / 2
            right_heading = heading_to_collision - np.pi / 2

            left_point = np.array([rx + self.EVASION_DISTANCE * np.cos(left_heading),
                                   ry + self.EVASION_DISTANCE * np.sin(left_heading)])
            right_point = np.array([rx + self.EVASION_DISTANCE * np.cos(right_heading),
                                    ry + self.EVASION_DISTANCE * np.sin(right_heading)])

            # Pick the side that is furthest from all other robots (least collision risk)
            evasion_point = self._pick_free_side(left_point, right_point)

            # Convert evasion point to lat/lon for the move_to goal
            try:
                gp = self._floatsam.convert_map_point_to_geopoint(float(evasion_point[0]),
                                                                   float(evasion_point[1]))
            except Exception as e:
                self.node.get_logger().error(f"{self.name}: Failed to convert evasion point to geopoint: {e}")
                return py_trees.common.Status.FAILURE

            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn(f"{self.name}: move_to action server not available")
                return py_trees.common.Status.FAILURE

            goal_dict = {
                "waypoint": {
                    "latitude": gp.latitude,
                    "longitude": gp.longitude,
                    "tolerance": self.EVASION_TOLERANCE,
                },
                "speed": "standard",
                "constant_speed": True,  # do NOT decelerate on approach
            }
            goal_msg = BaseAction.Goal()
            goal_msg.goal.data = json.dumps(goal_dict)

            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self._goal_response_cb)
            self.node.get_logger().info(
                f"{self.name}: Evasion goal sent to move_to "
                f"(map [{evasion_point[0]:.2f}, {evasion_point[1]:.2f}])"
            )
            return py_trees.common.Status.RUNNING

        # ── 2. Waiting for acceptance ────────────────────────────────────────
        if not self._goal_accepted:
            if self._goal_done:  # rejected
                self.node.get_logger().warn(f"{self.name}: Evasion goal rejected")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # ── 3. Goal finished ─────────────────────────────────────────────────
        if self._goal_done:
            if self._goal_succeeded:
                self.node.get_logger().info(f"{self.name}: Evasion move complete")
                # Reset collision streak counters for the robots we evaded
                for other_name in self.blackboard.colliding_list:
                    if other_name in self.blackboard.collision_streak:
                        self.blackboard.collision_streak[other_name].counter = 0
                        self.blackboard.collision_streak[other_name].is_colliding = False
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().warn(f"{self.name}: Evasion move failed")
                return py_trees.common.Status.FAILURE

        # ── 4. Still running ─────────────────────────────────────────────────
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancel goal if the BT preempts this node."""
        if self._goal_handle is not None and not self._goal_done:
            self._goal_handle.cancel_goal_async()

    # ── helpers ──────────────────────────────────────────────────────────────

    def _pick_free_side(self, left_point: np.ndarray, right_point: np.ndarray) -> np.ndarray:
        """Return whichever candidate evasion point is further from all other robots."""
        robot_positions = self.blackboard.robot_positions
        my_name = self.blackboard.this_robot_name

        def min_distance_to_others(point):
            min_d = float('inf')
            for name, pos in robot_positions.items():
                if name == my_name or pos is None:
                    continue
                d = np.sqrt((pos.pose.position.x - point[0]) ** 2 +
                            (pos.pose.position.y - point[1]) ** 2)
                min_d = min(min_d, d)
            return min_d

        left_clearance = min_distance_to_others(left_point)
        right_clearance = min_distance_to_others(right_point)

        if left_clearance >= right_clearance:
            self.node.get_logger().info(f"{self.name}: Evading LEFT (clearance L={left_clearance:.2f}, R={right_clearance:.2f})")
            return left_point
        else:
            self.node.get_logger().info(f"{self.name}: Evading RIGHT (clearance L={left_clearance:.2f}, R={right_clearance:.2f})")
            return right_point

    # ── action client callbacks ──────────────────────────────────────────────

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_done = True
            return
        self._goal_handle = goal_handle
        self._goal_accepted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self._goal_succeeded = result.success
        self._goal_done = True


class Wait(py_trees.behaviour.Behaviour):
    """Stop the agent by publishing zero velocity.

    Publishes speed = 0.0 to the VELOCITY_SETPOINT topic every tick so that
    the low-level controller keeps the robot stationary.  Returns RUNNING so
    the behaviour tree will keep ticking this node (and re-evaluating the
    collision selectors above it) on the next cycle.
    """

    def __init__(self, name="Wait"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self._speed_pub = None

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self._speed_pub = self.node.create_publisher(
            FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, 10
        )
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Publish zero velocity every tick."""
        speed_msg = FloatStamped()
        speed_msg.header.stamp = self.node.get_clock().now().to_msg()
        speed_msg.data = 0.0
        self._speed_pub.publish(speed_msg)
        self.node.get_logger().info(f"{self.name}: Publishing zero velocity (waiting)")
        return py_trees.common.Status.RUNNING


class ArrivalCheck(py_trees.behaviour.Behaviour):
    """Check if agent has arrived at assigned target position."""
    
    def __init__(self, name="ArrivalCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_arrived_flag", access=py_trees.common.Access.READ)
        self.this_robot_name = self.blackboard.this_robot_name


    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Check if arrived at target."""
        self.node.get_logger().info(f"{self.name}: Update begin")

        if not hasattr(self.blackboard, 'this_robot_arrived_flag') or self.blackboard.this_robot_arrived_flag == False:
            self.node.get_logger().info(f"{self.this_robot_name}: Is not arrived yet")
            return py_trees.common.Status.FAILURE
        
        self.node.get_logger().info(f"{self.this_robot_name}: Is arrived")
        return py_trees.common.Status.SUCCESS


class MoveToPathClient(py_trees.behaviour.Behaviour):
    """Action client to call move_to_path action server with coordinated speed override.

    Speed logic (published to SPEED_OVERRIDE each tick while goal is active and not
    yet decelerating):
    - Compute the Euclidean distance to their assigned formation goal for every robot
      that has a known position and assignment.
    - The furthest robot publishes max_velocity.
    - Every other robot publishes (my_distance / max_distance) * max_velocity so that
      all robots arrive at their goals at the same time.
    - Once the action server reports decelerating=True in its feedback, stop publishing
      (let the action server's own ramp handle the final approach).
    """

    def __init__(self, name="MoveToPathClient"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_assignments",       access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions",         access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points",        access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points_latlon", access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_name",         access=py_trees.common.Access.READ)
        self.blackboard.register_key("max_velocity",            access=py_trees.common.Access.READ)
        self.blackboard.register_key("waypoints_step_size",     access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_arrived_flag", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="last_point_tolerance_move_path", access=py_trees.common.Access.READ)

        self._action_client = None
        self._speed_pub     = None
        self.blackboard.this_robot_arrived_flag = False

    def setup(self, **kwargs):
        """Called once when the tree is set up. Creates the ROS action client and publisher."""
        self.node = kwargs['node']
        self._floatsam = kwargs['node']._floatsam  # FloatSam instance on the server node
        self._action_client = ActionClient(self.node, BaseAction, 'move_path')
        self._speed_pub = self.node.create_publisher(
            FloatStamped, FloatsamTopics.SPEED_OVERRIDE, 10
        )
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        """Reset state each time the BT enters this behaviour from a non-RUNNING state."""
        self._send_goal_future  = None
        self._get_result_future = None
        self._goal_handle       = None
        self._goal_accepted     = False
        self._goal_done         = False
        self._goal_succeeded    = False
        self._decelerating      = False

    def update(self):
        self.node.get_logger().info(f"{self.name}: Update begin")
        # ── 1. Send goal (first tick) ────────────────────────────────────────────
        if self._send_goal_future is None:
            my_name     = self.blackboard.this_robot_name
            assignments = self.blackboard.robot_assignments
            if my_name not in assignments:
                self.node.get_logger().warn(f"{self.name}: No assignment for {my_name}")
                return py_trees.common.Status.FAILURE

            my_goal_key = assignments[my_name]

            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn(f"{self.name}: move_path action server not available")
                return py_trees.common.Status.FAILURE

            waypoints = self._build_waypoints(my_name, my_goal_key)
            if waypoints is None:
                return py_trees.common.Status.FAILURE

            goal_dict = {
                "waypoints": waypoints,
                "speed": "override"
            }
            goal_msg = BaseAction.Goal()
            goal_msg.goal.data = json.dumps(goal_dict)

            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self._feedback_cb
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
            self.node.get_logger().info(f"{self.name}: Goal sent to move_path ({my_goal_key})")
            return py_trees.common.Status.RUNNING

        # ── 2. Waiting for server to accept the goal ─────────────────────────────
        if not self._goal_accepted:
            self.node.get_logger().info(f"{self.name}: Waiting for goal acceptance...")
            if self._goal_done:  # rejected
                self.node.get_logger().info(f"{self.name}: Goal was rejected by the server")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        # ── 3. Goal finished ─────────────────────────────────────────────────────
        if self._goal_done:
            self.node.get_logger().info(f"{self.name}: Goal finished with status: {'SUCCEEDED' if self._goal_succeeded else 'FAILED'}")
            return (
                py_trees.common.Status.SUCCESS
                if self._goal_succeeded
                else py_trees.common.Status.FAILURE
            )

        # ── 4. Goal running: publish coordinated speed override ──────────────────
        if not self._decelerating:
            self.node.get_logger().info(f"{self.name}: Goal running, publishing speed override...")
            speed = self._compute_speed_override()
            if speed is not None:
                msg = FloatStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.data = float(speed)
                self._speed_pub.publish(msg)
        else:
            self.node.get_logger().info(f"{self.name}: Decelerating, not publishing speed override")

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancel the goal if the BT aborts this behaviour while it is running."""
        self.node.get_logger().info(f"{self.name}: Terminate called with new_status={new_status}")
        if self._goal_handle is not None and not self._goal_done:
            self._goal_handle.cancel_goal_async()

    # ── Callbacks ────────────────────────────────────────────────────────────────

    def _feedback_cb(self, feedback_msg):
        """Parse JSON feedback and update _decelerating flag."""
        try:
            fb = json.loads(feedback_msg.feedback.feedback.data)
            self._decelerating = bool(fb.get("decelerating", False))
        except Exception:
            pass

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name}: Goal rejected by move_path server")
            self._goal_done = True
            return
        self._goal_handle   = goal_handle
        self._goal_accepted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.node.get_logger().info(f"{self.name}: Goal result received, result: {future.result().result}")
        result = future.result().result
        self._goal_succeeded = result.success
        self._goal_done      = True
        if self._goal_succeeded:
            # Safety check: verify actual distance before declaring arrival
            try:
                my_name  = self.blackboard.this_robot_name
                goal_key = self.blackboard.robot_assignments[my_name]
                pos      = self.blackboard.robot_positions.get(my_name)
                goal_xy  = self.blackboard.formation_points.get(goal_key)
                if pos is not None and goal_xy is not None:
                    dx = pos.pose.position.x - goal_xy[0]
                    dy = pos.pose.position.y - goal_xy[1]
                    dist = np.sqrt(dx * dx + dy * dy)
                    tolerance = float(self.blackboard.last_point_tolerance_move_path) * 3.0
                    if dist > tolerance:
                        self.node.get_logger().error(
                            f"{self.name}: move_path reported SUCCESS but "
                            f"{my_name} is still {dist:.2f}m from goal "
                            f"(tolerance {tolerance:.2f}m). NOT marking as arrived."
                        )
                        self._goal_succeeded = False
                        return
            except Exception as e:
                self.node.get_logger().warn(
                    f"{self.name}: Could not verify arrival distance: {e}"
                )
            self.blackboard.this_robot_arrived_flag = True

    # ── Waypoint generation ───────────────────────────────────────────────────────

    def _build_waypoints(self, my_name: str, my_goal_key: str) -> list | None:
        """
        Build an ordered list of waypoints from the robot's current map position to
        its formation goal, placing one intermediate waypoint every waypoints_step_size
        metres along the straight line.  The final point is always the exact goal.
        Each waypoint is a dict with latitude, longitude and tolerance keys.
        """
        #try:
        step     = float(self.blackboard.waypoints_step_size)
        goal_xy  = self.blackboard.formation_points[my_goal_key]        # [x, y] in map
        my_pos   = self.blackboard.robot_positions[my_name]             # PoseStamped in map

        if my_pos is None:
            self.node.get_logger().warn(f"{self.name}: No position for {my_name}")
            return None

        sx = my_pos.pose.position.x
        sy = my_pos.pose.position.y
        ex, ey = goal_xy[0], goal_xy[1]

        dx   = ex - sx
        dy   = ey - sy
        dist = np.sqrt(dx * dx + dy * dy)

        waypoints = []

        if dist > step:
            n_steps = int(dist / step)  # number of intermediate points
            for k in range(1, n_steps + 1):
                t   = k * step / dist
                wx  = sx + t * dx
                wy  = sy + t * dy
                gp  = self._floatsam.convert_map_point_to_geopoint(wx, wy)
                waypoints.append({
                    "latitude":  gp.latitude,
                    "longitude": gp.longitude,
                    "tolerance": step * 0.75   # generous tolerance for intermediate points
                })

        # Always add the exact final goal
        final_latlon = self.blackboard.formation_points_latlon[my_goal_key]
        waypoints.append({
            "latitude":  final_latlon["latitude"],
            "longitude": final_latlon["longitude"],
            "tolerance": self.blackboard.last_point_tolerance_move_path  
        })

        self.node.get_logger().info(
            f"{self.name}: Built {len(waypoints)} waypoints "
            f"(step={step}m, total dist={dist:.2f}m)"
        )
        return waypoints

        #except Exception as e:
        #    self.node.get_logger().error(f"{self.name}: _build_waypoints error: {e}")
        #    return None

    # ── Speed logic ───────────────────────────────────────────────────────────────

    def _compute_speed_override(self) -> float | None:
        """
        Returns the speed this robot should publish so that all robots arrive
        at their goals at the same time.
        The furthest robot goes at max_velocity; all others scale proportionally.
        """
        try:
            robot_positions  = self.blackboard.robot_positions
            formation_points = self.blackboard.formation_points
            assignments      = self.blackboard.robot_assignments
            my_name          = self.blackboard.this_robot_name
            max_vel          = float(self.blackboard.max_velocity)

            distances = {}
            for robot_name, goal_key in assignments.items():
                pos     = robot_positions.get(robot_name)
                goal_xy = formation_points.get(goal_key)
                if pos is not None and goal_xy is not None:
                    dx = pos.pose.position.x - goal_xy[0]
                    dy = pos.pose.position.y - goal_xy[1]
                    distances[robot_name] = np.sqrt(dx * dx + dy * dy)
                    self.node.get_logger().info(f"CIAOOO distances: {distances[robot_name]:.2f}m for {robot_name}")

            if not distances or my_name not in distances:
                return max_vel  # fallback: go at max speed

            max_dist = max(distances.values())
            my_dist  = distances[my_name]

            if max_dist < 0.01:
                return 0.0  # everyone is already at their goal

            self.node.get_logger().warn(f"{my_name}'s velocity: {(my_dist / max_dist) * max_vel}, distance to goal: {my_dist:.2f}m, max distance: {max_dist:.2f}m")


            # speed_i = (d_i / d_max) * max_vel  →  same ETA for all robots
            return (my_dist / max_dist) * max_vel

        except Exception as e:
            self.node.get_logger().warn(f"{self.name}: Speed compute error: {e}")
            return None


class AllArrivalCheck(py_trees.behaviour.Behaviour):
    """Check if all agents in formation have arrived at their targets."""
    
    def __init__(self, name="AllArrivalCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Check if all agents have arrived."""
        # TODO: Implement logic to check all agents' arrival status
        # Read robot_positions and robot_assignments
        # Check if ALL robots are at their assigned targets
        self.node.get_logger().info(f"{self.name}: Checking if all agents arrived...")
        return py_trees.common.Status.RUNNING


class LoiterWithHeadingClient(py_trees.behaviour.Behaviour):
    """Action client to call loiter_with_heading action server."""
    
    def __init__(self, name="LoiterWithHeadingClient"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Called once when the tree is created. Initialize action client."""
        self.node = kwargs['node']
        # TODO: Initialize ROS action client for loiter_with_heading
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Send goal to loiter_with_heading action server."""
        # TODO: Implement action client logic
        # Read robot_assignments to get target heading for each robot
        self.node.get_logger().info(f"{self.name}: Calling loiter_with_heading action...")
        return py_trees.common.Status.RUNNING


    """
    # to be put inside the class of the node that calls loiter with heading 
    # (to check at the end if each vessel is correctly positioned and oriented)


    def send_goal(self, duration, heading):
        goal_msg = BaseAction.Goal()
        goal_dict = {
            'duration': duration,
            'heading': heading
        }
        goal_msg.goal = String(data=json.dumps(goal_dict))
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback  # Handle feedback here
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    

    def feedback_callback(self, feedback_msg):
        # Called whenever feedback is received"
        feedback_str = feedback_msg.feedback.feedback.data
        feedback = json.loads(feedback_str)
        
        self.get_logger().info(
            f"Feedback: Position reached: {feedback['position_reached']}, "
            f"Heading reached: {feedback['heading_reached']}, "
            f"Distance: {feedback['distance_from_center']:.2f}m, "
            f"Heading error: {feedback['heading_error']:.1f}°"
        )
        
        # Check if both are achieved
        if feedback['both_reached']:
            self.get_logger().info("✓ Both position and heading reached!")
    
            
     def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result.data}')

    """