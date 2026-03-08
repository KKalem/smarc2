import py_trees
import numpy as np
import json
from scipy.optimize import linear_sum_assignment
from rclpy.action import ActionClient
from smarc_msgs.action import BaseAction
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String


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


# CollisionFreeCheck, PriorityCheck, CounterCheck, MoveToSide, Wait removed:
# collision avoidance is now handled by the RVO service.


class _RemovedCollisionFreeCheck:
    
    def __init__(self, name="CollisionFreeCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_radius", access=py_trees.common.Access.READ)
        self.blackboard.register_key("colliding_list", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("currently_colliding_with", access=py_trees.common.Access.WRITE)

        # Initialize the blackboard variable
        self.blackboard.currently_colliding_with = ""
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
    
        if self.blackboard.currently_colliding_with != "":
            return py_trees.common.Status.FAILURE

        robot_positions = self.blackboard.robot_positions
        self.this_position = robot_positions[self.this_robot_name]
        self.node.get_logger().info(f"{self.name}: Checking for collisions...")

        for key, position in robot_positions.items():
            if key != self.this_robot_name:
                distance = self.compute_distance(self.this_position, position)
                # Check if actually colliding (within radius)
                if distance <= self.collision_radius:
                    # Only add to list if it's not the one we're already handling
                    if self.blackboard.currently_colliding_with != key:
                        self.node.get_logger().info(f"{self.name}: Robot {self.this_robot_name} is colliding with {key}")
                        self.blackboard.colliding_list.append(key)
                    else:
                        self.node.get_logger().info(f"{self.name}: Already handling collision with {key}")
                else:
                    # Not colliding, reset collision streak for this robot
                    if key in self.blackboard.collision_streak:
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


class _RemovedPriorityCheck:
    
    def __init__(self, name="PriorityCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("colliding_list", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("formation_cluster_centre", access=py_trees.common.Access.READ)
        self.blackboard.register_key("approach_direction", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points", access=py_trees.common.Access.READ)
        self.blackboard.register_key("last_point_tolerance_move_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key("currently_colliding_with", access=py_trees.common.Access.READ)

        
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

        if self.blackboard.currently_colliding_with != "":
            return py_trees.common.Status.FAILURE

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
                   
                # Check if the other robot has already reached its goal
                # If so, force counter to 3 to trigger move_to_side
                if self._has_robot_reached_goal(robot):
                    self.node.get_logger().info(
                        f"{self.this_robot_name}: {robot} has reached its goal, "
                        f"forcing counter to 3 to avoid indefinite wait"
                    )
                    self.blackboard.collision_streak[robot].counter = np.inf

        if self.not_priority:
            return py_trees.common.Status.FAILURE
        self.node.get_logger().info(f"{self.name}: ORA RITORNO SUCCESSSSSO ")
        return py_trees.common.Status.SUCCESS
    
    def _has_robot_reached_goal(self, robot_name: str) -> bool:
        """Check if the given robot has reached its assigned goal."""
        try:
            if not hasattr(self.blackboard, 'robot_assignments'):
                return False
            if not hasattr(self.blackboard, 'robot_positions'):
                return False
            if not hasattr(self.blackboard, 'formation_points'):
                return False
            if not hasattr(self.blackboard, 'last_point_tolerance_move_path'):
                return False
                
            goal_key = self.blackboard.robot_assignments.get(robot_name)
            if goal_key is None:
                return False
                
            robot_pos = self.blackboard.robot_positions.get(robot_name)
            goal_xy = self.blackboard.formation_points.get(goal_key)
            
            if robot_pos is None or goal_xy is None:
                return False
            
            # Calculate distance to goal
            dx = robot_pos.pose.position.x - goal_xy[0]
            dy = robot_pos.pose.position.y - goal_xy[1]
            dist = np.sqrt(dx * dx + dy * dy)
            
            # Use tolerance (same as in move_path)
            tolerance = float(self.blackboard.last_point_tolerance_move_path) * 3.0
            
            return dist <= tolerance
            
        except Exception as e:
            self.node.get_logger().warn(
                f"{self.name}: Could not check if {robot_name} reached goal: {e}"
            )
            return False
            
    def calculate_proj_dist(self, this_position):
        rx, ry = this_position.pose.position.x, this_position.pose.position.y
        # signed projection distance along the approach line 
        #proj = np.abs((rx - self.fc[0]) * np.cos(self.theta) + (ry - self.fc[1]) * np.sin(self.theta))
        proj = np.abs((ry - self.fc[1]) * np.sin(self.theta))
        return proj
    
    def priority(self, this_position, other_position):
        this_proj = self.calculate_proj_dist(this_position)
        other_proj = self.calculate_proj_dist(other_position)
        if this_proj <= other_proj:
            return True
        else:
            return False


class _RemovedCounterCheck:
    
    def __init__(self, name="CounterCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("collision_streak", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("max_num_collisions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("currently_colliding_with", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):

        if self.blackboard.currently_colliding_with != "":
            return py_trees.common.Status.SUCCESS

        """Check counter value."""
        self.node.get_logger().info(f"{self.name}: Checking counter for collision avoidance...")
        for key in self.blackboard.collision_streak.keys():
            self.node.get_logger().info(f"{self.name}: Checking counter with {key} = {self.blackboard.collision_streak[key].counter}")

            if self.blackboard.collision_streak[key].counter >= self.blackboard.max_num_collisions:
                self.node.get_logger().info(f"{self.blackboard.this_robot_name} has reached the collision streak with {key}")
                self.blackboard.currently_colliding_with = key
                return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE


class _RemovedMoveToSide:

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
        self.blackboard.register_key("currently_colliding_with", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("formation_cluster_centre", access=py_trees.common.Access.READ)


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

            # # Average direction toward all colliding robots
            # dx_sum, dy_sum = 0.0, 0.0
            # for other_name in colliding_list:
            #     other_pos = self.blackboard.robot_positions.get(other_name)
            #     if other_pos is not None:
            #         dx_sum += other_pos.pose.position.x - rx
            #         dy_sum += other_pos.pose.position.y - ry

            

            # if abs(dx_sum) < 1e-6 and abs(dy_sum) < 1e-6:
            #     # Robots are on top of each other; pick an arbitrary direction
            #     dx_sum, dy_sum = 1.0, 0.0

            # Heading toward collision cluster


            other_pos = self.blackboard.robot_positions.get(self.blackboard.currently_colliding_with)

            dx_sum = other_pos.pose.position.x - rx
            dy_sum = other_pos.pose.position.y - ry

            heading_to_collision = np.arctan2(dy_sum, dx_sum)

            proj_me = np.abs((rx - self.blackboard.formation_cluster_centre[0]) * np.cos(self.theta))
            proj_other = np.abs((other_pos.pose.position.x - self.blackboard.formation_cluster_centre[0]) * np.cos(self.theta))

            if proj_me <= proj_other:
                left_heading = heading_to_collision + np.pi / 4
                evasion_point = np.array([rx + self.EVASION_DISTANCE * np.cos(left_heading),
                                   ry + self.EVASION_DISTANCE * np.sin(left_heading)])
            else:
                right_heading = heading_to_collision - np.pi / 4
                evasion_point = np.array([rx + self.EVASION_DISTANCE * np.cos(right_heading),
                                    ry + self.EVASION_DISTANCE * np.sin(right_heading)])

            # Two candidate evasion headings: ±45° from collision direction
            # left_heading = heading_to_collision + np.pi / 4
            # right_heading = heading_to_collision - np.pi / 4

            # left_point = np.array([rx + self.EVASION_DISTANCE * np.cos(left_heading),
            #                        ry + self.EVASION_DISTANCE * np.sin(left_heading)])
            # right_point = np.array([rx + self.EVASION_DISTANCE * np.cos(right_heading),
            #                         ry + self.EVASION_DISTANCE * np.sin(right_heading)])

            # # Pick the side that is furthest from all other robots (least collision risk)
            # evasion_point = self._pick_free_side(left_point, right_point)

            # Convert evasion point to lat/lon ros2 action send_goal /floatsam_usv_0/go_to_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"formation_points\": [{\"latitude\": 58.8405258584503, \"longitude\": 17.6516992496307, \"heading\": 90.0}, {\"latitude\": 58.8405428888343, \"longitude\": 17.6518663726091, \"heading\": 90.0}, {\"latitude\": 58.8405878677775, \"longitude\": 17.6517617503888, \"heading\": 90.0}]}'}}"for the move_to goal
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
                self.blackboard.currently_colliding_with = ""
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
            self.blackboard.currently_colliding_with = ""
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
        self.blackboard.currently_colliding_with = ""
        self._goal_done = True


class _RemovedWait:

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


class MoveToClient(py_trees.behaviour.Behaviour):
    """Action client that calls the move_to action server.

    Sends the assigned formation goal as a move_to goal using the standard
    waypoint dict format (latitude / longitude / tolerance + speed).
    Collision avoidance is fully delegated to the RVO service running inside
    the move_to server, so no collision-check leaves are needed in the tree.
    """

    def __init__(self, name="MoveToClient"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_assignments",           access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions",             access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points",            access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points_latlon",     access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_name",             access=py_trees.common.Access.READ)
        self.blackboard.register_key("max_velocity",                access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_arrived_flag",     access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("last_point_tolerance_move_path", access=py_trees.common.Access.READ)

        self._action_client = None
        self.blackboard.this_robot_arrived_flag = False

    def setup(self, **kwargs):
        """Called once when the tree is set up. Creates the ROS action client."""
        self.node = kwargs['node']
        self._floatsam = kwargs['node']._floatsam
        self._action_client = ActionClient(self.node, BaseAction, 'move_to')
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        """Reset state each time the BT enters this behaviour from a non-RUNNING state."""
        self._send_goal_future  = None
        self._get_result_future = None
        self._goal_handle       = None
        self._goal_accepted     = False
        self._goal_done         = False
        self._goal_succeeded    = False

    def update(self):
        self.node.get_logger().info(f"{self.name}: Update begin")

        # ── 1. Send goal (first tick) ────────────────────────────────────────────
        if self._send_goal_future is None:
            my_name     = self.blackboard.this_robot_name
            assignments = self.blackboard.robot_assignments
            if my_name not in assignments:
                self.node.get_logger().warn(f"{self.name}: No assignment for {my_name}")
                return py_trees.common.Status.FAILURE

            my_goal_key  = assignments[my_name]
            final_latlon = self.blackboard.formation_points_latlon.get(my_goal_key)
            if final_latlon is None:
                self.node.get_logger().warn(f"{self.name}: No lat/lon data for {my_goal_key}")
                return py_trees.common.Status.FAILURE

            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn(f"{self.name}: move_to action server not available")
                return py_trees.common.Status.FAILURE

            goal_dict = {
                'waypoint': {
                    'latitude':  final_latlon['latitude'],
                    'longitude': final_latlon['longitude'],
                    'tolerance': self.blackboard.last_point_tolerance_move_path
                },
                'speed': self.blackboard.max_velocity
            }
            goal_msg = BaseAction.Goal()
            goal_msg.goal.data = json.dumps(goal_dict)

            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self._feedback_cb
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
            self.node.get_logger().info(f"{self.name}: Goal sent to move_to ({my_goal_key})")
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
            self.node.get_logger().info(
                f"{self.name}: Goal finished: {'SUCCEEDED' if self._goal_succeeded else 'FAILED'}"
            )
            return (
                py_trees.common.Status.SUCCESS
                if self._goal_succeeded
                else py_trees.common.Status.FAILURE
            )

        # ── 4. Still running ─────────────────────────────────────────────────────
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancel the goal if the BT aborts this behaviour while it is running."""
        self.node.get_logger().info(f"{self.name}: Terminate called with new_status={new_status}")
        if self._goal_handle is not None and not self._goal_done:
            self._goal_handle.cancel_goal_async()

    # ── Callbacks ────────────────────────────────────────────────────────────────

    def _feedback_cb(self, feedback_msg):
        """Log feedback from move_to."""
        try:
            fb = json.loads(feedback_msg.feedback.feedback.data)
            self.node.get_logger().info(f"{self.name}: Feedback - {fb}")
        except Exception:
            pass

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name}: Goal rejected by move_to server")
            self._goal_done = True
            return
        self._goal_handle   = goal_handle
        self._goal_accepted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.node.get_logger().info(f"{self.name}: Goal result received")
        result = future.result().result
        self._goal_succeeded = result.success
        self._goal_done      = True
        if self._goal_succeeded:
            self.blackboard.this_robot_arrived_flag = True


class AllArrivalCheck(py_trees.behaviour.Behaviour):
    """Check if all agents in formation have arrived at their targets."""
    
    def __init__(self, name="AllArrivalCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)
        self.blackboard.register_key("loiter_heading_fb", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """Called once when the tree is created."""
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        """Check if all agents have arrived."""
        # TODO: Implement logic to check all agents' arrival status
        # Read robot_positions and robot_assignments
        # Check if ALL robots are at their assigned targets

        all_arrived_flag = False
        for robot_name, ready in self.blackboard.loiter_heading_fb.items():
            self.node.get_logger().info(f"{self.name}: Loiter feedback for {robot_name} = {ready}")
            if ready == 0:
                all_arrived_flag = False
                break
            elif ready == 1:
                all_arrived_flag = True


        if all_arrived_flag:
            self.node.get_logger().info(f"{self.name}: All robots have arrived at their targets!")
            return py_trees.common.Status.SUCCESS   
        else: 
            self.node.get_logger().info(f"{self.name}: NOT all robots have arrived at their targets yet!")
            return py_trees.common.Status.FAILURE  


class LoiterWithHeadingClient(py_trees.behaviour.Behaviour):
    """Action client to call loiter_with_heading action server.
    
    Loiters at the current position with a specified heading for a given duration.
    Duration: 400 seconds
    Heading: extracted from the formation goal assigned to this robot
    """
    
    def __init__(self, name="LoiterWithHeadingClient"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points_latlon", access=py_trees.common.Access.READ)
        self._action_client = None

    def setup(self, **kwargs):
        """Called once when the tree is created. Initialize action client."""
        self.node = kwargs['node']
        # Initialize ROS action client for loiter_with_heading
        self._action_client = ActionClient(self.node, BaseAction, 'loiter_heading')
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        """Reset state each time the BT enters this behaviour from a non-RUNNING state."""
        self._send_goal_future = None
        self._get_result_future = None
        self._goal_handle = None
        self._goal_accepted = False
        self._goal_done = False
        self._goal_succeeded = False

    def update(self):
        """Send goal to loiter_with_heading action server and manage action lifecycle."""
        self.node.get_logger().info(f"{self.name}: Update begin")
        
        # ── 1. Send goal (first tick) ────────────────────────────────────────────
        if self._send_goal_future is None:
            my_name = self.blackboard.this_robot_name
            assignments = self.blackboard.robot_assignments
            
            if my_name not in assignments:
                self.node.get_logger().warn(f"{self.name}: No assignment for {my_name}")
                return py_trees.common.Status.FAILURE
            
            my_goal_key = assignments[my_name]
            
            # Get the heading from the formation goal
            formation_points_latlon = self.blackboard.formation_points_latlon
            if my_goal_key not in formation_points_latlon:
                self.node.get_logger().warn(f"{self.name}: No formation point data for {my_goal_key}")
                return py_trees.common.Status.FAILURE
            
            goal_data = formation_points_latlon[my_goal_key]
            heading = goal_data.get('heading', 0.0)
            
            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn(f"{self.name}: loiter_heading action server not available")
                return py_trees.common.Status.FAILURE
            
            # Create goal message with duration=400 and heading from formation goal
            goal_dict = {
                'duration': 400,  # 400 seconds
                'heading': heading
            }
            goal_msg = BaseAction.Goal()
            goal_msg.goal.data = json.dumps(goal_dict)
            
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_cb
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
            self.node.get_logger().info(
                f"{self.name}: Goal sent to loiter_heading "
                f"(duration=400s, heading={heading}°)"
            )
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
            self.node.get_logger().info(
                f"{self.name}: Goal finished with status: "
                f"{'SUCCEEDED' if self._goal_succeeded else 'FAILED'}"
            )
            return (
                py_trees.common.Status.SUCCESS
                if self._goal_succeeded
                else py_trees.common.Status.FAILURE
            )
        
        # ── 4. Still running ─────────────────────────────────────────────────────
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancel the goal if the BT aborts this behaviour while it is running."""
        self.node.get_logger().info(f"{self.name}: Terminate called with new_status={new_status}")
        if self._goal_handle is not None and not self._goal_done:
            self._goal_handle.cancel_goal_async()

    # ── Callbacks ────────────────────────────────────────────────────────────────

    def _feedback_cb(self, feedback_msg):
        """Parse JSON feedback from the loiter action server."""
        try:
            feedback_str = feedback_msg.feedback.feedback.data
            feedback = json.loads(feedback_str)
            self.node.get_logger().info(
                f"{self.name}: Feedback - Position reached: {feedback.get('position_reached', False)}, "
                f"Heading reached: {feedback.get('heading_reached', False)}, "
                f"Distance: {feedback.get('distance_from_center', 0.0):.2f}m, "
                f"Heading error: {feedback.get('heading_error', 0.0):.1f}°"
            )
        except Exception as e:
            self.node.get_logger().warn(f"{self.name}: Failed to parse feedback: {e}")

    def _goal_response_cb(self, future):
        """Handle goal response from action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name}: Goal rejected by loiter_heading server")
            self._goal_done = True
            return
        self._goal_handle = goal_handle
        self._goal_accepted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        """Handle final result from action server."""
        self.node.get_logger().info(f"{self.name}: Goal result received")
        result = future.result().result
        self._goal_succeeded = result.success
        self._goal_done = True
