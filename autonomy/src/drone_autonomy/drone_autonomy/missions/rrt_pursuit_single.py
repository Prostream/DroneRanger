
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Iterative Exploration Pursuit with Obstacle Avoidance — stable takeoff + ENU→NED conversions

Key points
- Publishes OffboardControlMode + TrajectorySetpoint every cycle with timestamps
- Uses a conservative takeoff state machine before entering pursuit
- Converts ENU target (x_east, y_north, z_up) -> NED setpoint (xN, yE, zD=-z_up)
- Computes yaw consistently in NED: yaw = atan2(vE, vN)
- Implements iterative exploration-based avoidance:
  * When obstacle detected: rotate 90° left or right to explore
  * Unity ForwardObstacleGuard continues detecting in new direction
  * Tracks visited grid cells to avoid redundant exploration
  * Iteratively searches for clear path toward goal

Avoidance Strategy:
1. Direct mode: fly toward goal
2. Obstacle detected → BACK UP first (safety margin)
3. After backing up → rotate 90° (left or right, choose unvisited)
4. Check new direction with Unity sensor
5. If clear: advance forward in exploration direction
6. If blocked: try opposite 90° rotation
7. After moving past obstacle: return to direct mode

PX4 topics (uXRCE-DDS, ROS 2):
  pub: /fmu/in/offboard_control_mode     (px4_msgs/OffboardControlMode)
  pub: /fmu/in/trajectory_setpoint       (px4_msgs/TrajectorySetpoint)
  pub: /fmu/in/vehicle_command           (px4_msgs/VehicleCommand)
  sub: /fmu/out/vehicle_local_position   (px4_msgs/VehicleLocalPosition)
  sub: /testTarget                       (geometry_msgs/PoseStamped)  -- ENU frame target
  sub: /testObstacleSignal               (std_msgs/Bool)              -- Unity obstacle detection

Tested intent:
- SITL/Unity HITL style loops where /testTarget is published in ENU frame
- Unity ForwardObstacleGuard publishes Bool (True=blocked, False=clear)

Author: Modified for iterative exploration avoidance
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool

# ---------------------------- Utilities ----------------------------

def radians_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def enu_to_ned(x_e: float, y_n: float, z_u: float):
    """Convert ENU (x=East, y=North, z=Up) to NED (x=North, y=East, z=Down)."""
    xN = y_n
    yE = x_e
    zD = -z_u
    return xN, yE, zD

def micros(node: Node) -> int:
    """Current time in microseconds for PX4 timestamps."""
    return node.get_clock().now().nanoseconds // 1000

# ---------------------------- State Machine ----------------------------

class Stage(Enum):
    WARMUP = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER_STABLE = 3
    PURSUIT = 4

# ---------------------------- Main Node ----------------------------

class RRTPursuitFixed(Node):
    def __init__(self):
        super().__init__('rrt_pursuit_fixed')

        # ---------- Parameters ----------
        self.declare_parameter('hz', 30.0)
        self.declare_parameter('takeoff_alt_m', 3.0)       # "up" in meters; will convert to zD=-takeoff_alt_m
        self.declare_parameter('min_nav_alt_m', 2.0)       # must be above this to start pursuit
        self.declare_parameter('stable_hover_s', 1.0)      # time to hold after takeoff reaches altitude
        self.declare_parameter('target_topic', '/testTarget')
        self.declare_parameter('obstacle_topic', '/testObstacleSignal')
        self.declare_parameter('use_iterative_exploration', True)  # if True, use iterative exploration avoidance
        self.declare_parameter('step_size_m', 4.0)                # step size in meters for pursuit
        self.declare_parameter('grid_resolution_m', 2.0)          # grid cell size for visited tracking
        self.declare_parameter('exploration_timeout_s', 65.0)      # max time for exploration attempts before resetting
        self.declare_parameter('path_memory_length', 5)            # number of recent positions to remember

        hz = float(self.get_parameter('hz').value)
        self.dt = 1.0 / max(hz, 1.0)

        # ---------- Runtime Vars ----------
        self.stage = Stage.WARMUP
        self.stage_enter_ts = self.get_clock().now().nanoseconds
        self.armed = False
        self.offboard_ok = False

        # local position validity
        self.xy_valid = False
        self.z_valid  = False

        # latest local position (NED/FRD PX4 frame)
        self.px = 0.0  # north
        self.py = 0.0  # east
        self.pz = 0.0  # down (positive is downward)
        self.heading = 0.0  # yaw in NED

        # target in ENU
        self.tgt_e = None
        self.tgt_n = None
        self.tgt_u = None

        # obstacle detection (from Unity ForwardObstacleGuard)
        self.obstacle_detected = False

        # iterative exploration state
        self.visited_cells = set()              # set of (grid_x, grid_y) tuples of explored cells
        self.obstacle_map = {}                  # dict: {(grid_x, grid_y): set([blocked_yaw_angles])}
        self.exploration_path = []              # list of recent (grid_x, grid_y, yaw) tuples
        self.current_heading_mode = 'direct'    # 'direct', 'explore_left', 'explore_right'
        self.exploration_start_time = None      # timestamp when exploration started
        self.locked_heading = None              # heading to maintain during exploration
        self.direction_switch_cooldown = 0      # cycles to wait before allowing another direction switch
        self.initial_cooldown = 0               # initial cooldown value for mandatory wait calculation
        self.stuck_counter = 0                  # count how many times both directions are blocked
        self.exploration_move_count = 0         # count forward moves in exploration mode
        self.explore_start_n = None             # N coordinate when exploration started
        self.explore_start_e = None             # E coordinate when exploration started
        self.last_move_n = None                 # last position after move (for detecting actual movement)
        self.last_move_e = None
        self.clear_confirmation_count = 0       # count consecutive clear detections before proceeding
        self.cooldown_blocked_count = 0         # count consecutive blocked detections during cooldown

        # ---------- Publishers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp       = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd      = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ---------- Subscribers ----------
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_local_pos, qos)
        target_topic = str(self.get_parameter('target_topic').value)
        self.create_subscription(PoseStamped, target_topic, self.on_target, 10)

        # Subscribe to obstacle detection signal from Unity
        obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.create_subscription(Bool, obstacle_topic, self.on_obstacle, 10)

        # ---------- Timer Loop ----------
        self.timer = self.create_timer(self.dt, self.loop)

        # log banner
        self.get_logger().info(
            f"[node] up | hz={hz:.1f} | takeoff={self.get_parameter('takeoff_alt_m').value:.2f}m "
            f"| min_nav_alt={self.get_parameter('min_nav_alt_m').value:.2f}m "
            f"| step={self.get_parameter('step_size_m').value:.2f}m (safe: 2.5m) "
            f"| grid_res={self.get_parameter('grid_resolution_m').value:.2f}m "
            f"| target={target_topic} | obstacle={obstacle_topic} "
            f"| exploration={'ON (in-place turn)' if self.get_parameter('use_iterative_exploration').value else 'OFF'}"
        )

    # ---------------- Callbacks ----------------

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.xy_valid = bool(msg.xy_valid)
        self.z_valid  = bool(msg.z_valid)
        self.px = msg.x  # North (m)
        self.py = msg.y  # East  (m)
        self.pz = msg.z  # Down  (m, positive is down)
        self.heading = msg.heading  # rad, NED frame

    def on_target(self, msg: PoseStamped):
        # Target pose is assumed ENU. Extract relative or absolute? Use absolute ENU coordinates here.
        self.tgt_e = msg.pose.position.x  # East
        self.tgt_n = msg.pose.position.y  # North
        self.tgt_u = msg.pose.position.z  # Up

    def on_obstacle(self, msg: Bool):
        # Receive obstacle detection signal from Unity ForwardObstacleGuard
        # True = obstacle detected (blocked), False = clear
        self.obstacle_detected = msg.data

    # ---------------- PX4 Helpers ----------------

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = micros(self)
        # We control position in NED (x,y,z), not using velocity/accel/body-rate outputs here
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_offboard.publish(m)

    def publish_sp(self, xN=None, yE=None, zD=None, yaw=None):
        sp = TrajectorySetpoint()
        sp.timestamp = micros(self)

        # Keep last setpoint components if None
        # For simplicity, always set all components explicitly here.
        sp.position = [float(xN if xN is not None else self.px),
                       float(yE if yE is not None else self.py),
                       float(zD if zD is not None else self.pz)]
        sp.yaw = float(yaw if yaw is not None else self.heading)

        self.pub_sp.publish(sp)

    def cmd(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = micros(self)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def arm(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        self.armed = True

    def disarm(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        self.armed = False

    def set_offboard(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # PX4 custom: base=1 (custom), sub=6 (OFFBOARD)
        self.offboard_ok = True

    # ---------------- Planning Helpers ----------------

    def desired_yaw_towards(self, xN: float, yE: float, goalN: float, goalE: float) -> float:
        """Yaw in NED to face (goal - current). yaw = atan2(vE, vN)."""
        vN = goalN - xN
        vE = goalE - yE
        return radians_wrap(math.atan2(vE, vN))

    def rrt_next_step(self, curN: float, curE: float, goalN: float, goalE: float, step: float = 1.0):
        """Very small 'RRT-like' step: go straight toward the goal limited by 'step'."""
        dN = goalN - curN
        dE = goalE - curE
        dist = math.hypot(dN, dE)
        if dist < 1e-3:
            return goalN, goalE
        scale = min(1.0, step / dist)
        return curN + dN * scale, curE + dE * scale

    def position_to_grid(self, n: float, e: float) -> tuple:
        """Convert continuous NED position to discrete grid cell coordinates."""
        grid_res = float(self.get_parameter('grid_resolution_m').value)
        grid_x = int(math.floor(n / grid_res))
        grid_y = int(math.floor(e / grid_res))
        return (grid_x, grid_y)

    def mark_visited(self, n: float, e: float):
        """Mark current position as visited in the grid."""
        cell = self.position_to_grid(n, e)
        self.visited_cells.add(cell)

    def is_visited(self, n: float, e: float) -> bool:
        """Check if a position has been visited."""
        cell = self.position_to_grid(n, e)
        return cell in self.visited_cells

    def add_to_path(self, n: float, e: float, yaw: float):
        """Add current position and heading to exploration path, maintain fixed length."""
        cell = self.position_to_grid(n, e)
        max_len = int(self.get_parameter('path_memory_length').value)
        self.exploration_path.append((cell[0], cell[1], yaw))
        # Keep only recent N positions
        if len(self.exploration_path) > max_len:
            self.exploration_path.pop(0)

    def is_backtracking(self, n: float, e: float) -> bool:
        """Check if moving to (n, e) would return to a recent position in path."""
        cell = self.position_to_grid(n, e)
        # Check if this cell appears in recent path (excluding current position)
        if len(self.exploration_path) <= 1:
            return False

        # Check last N-1 positions (not including current)
        for i in range(len(self.exploration_path) - 1):
            if self.exploration_path[i][0] == cell[0] and self.exploration_path[i][1] == cell[1]:
                return True
        return False

    def mark_direction_blocked(self, n: float, e: float, yaw: float):
        """Mark a specific direction as blocked at given position."""
        cell = self.position_to_grid(n, e)
        # Quantize yaw to 45-degree increments for consistent storage
        yaw_deg = int(round(math.degrees(yaw) / 45.0) * 45) % 360

        if cell not in self.obstacle_map:
            self.obstacle_map[cell] = set()
        self.obstacle_map[cell].add(yaw_deg)

    def is_direction_blocked(self, n: float, e: float, yaw: float) -> bool:
        """Check if a specific direction is marked as blocked at given position."""
        cell = self.position_to_grid(n, e)
        if cell not in self.obstacle_map:
            return False

        yaw_deg = int(round(math.degrees(yaw) / 45.0) * 45) % 360
        return yaw_deg in self.obstacle_map[cell]

    def choose_exploration_angle(self, curN: float, curE: float, direct_yaw: float, step: float, stuck_count: int = 0):
        """
        Choose exploration angle with progressive escalation based on stuck count.
        Strategy: Start small, increase if blocked repeatedly.

        stuck_count=0: 30° (first try)
        stuck_count=1: 45° (second try)
        stuck_count=2: 60° (third try)
        stuck_count≥3: 90° (last resort)

        Returns: (chosen_yaw, angle_deg, side)
        """
        # Progressive angle based on stuck counter
        if stuck_count == 0:
            angle_deg = 30
        elif stuck_count == 1:
            angle_deg = 45
        elif stuck_count == 2:
            angle_deg = 60
        else:  # stuck_count >= 3
            angle_deg = 90

        angle_rad = math.radians(angle_deg)

        # Alternate between left and right to explore both sides
        if not hasattr(self, '_last_turn_side'):
            self._last_turn_side = 'right'

        # Try opposite side from last time
        if self._last_turn_side == 'left':
            chosen_side = 'right'
            chosen_yaw = radians_wrap(direct_yaw - angle_rad)
            test_n = curN + step * math.cos(chosen_yaw)
            test_e = curE + step * math.sin(chosen_yaw)

            # Only avoid if it's obvious backtracking
            if self.is_backtracking(test_n, test_e):
                chosen_side = 'left'
                chosen_yaw = radians_wrap(direct_yaw + angle_rad)
        else:
            chosen_side = 'left'
            chosen_yaw = radians_wrap(direct_yaw + angle_rad)
            test_n = curN + step * math.cos(chosen_yaw)
            test_e = curE + step * math.sin(chosen_yaw)

            # Only avoid if it's obvious backtracking
            if self.is_backtracking(test_n, test_e):
                chosen_side = 'right'
                chosen_yaw = radians_wrap(direct_yaw - angle_rad)

        # Remember for next time
        self._last_turn_side = chosen_side

        return chosen_yaw, angle_deg, chosen_side

    def iterative_exploration_step(self, curN: float, curE: float, goalN: float, goalE: float,
                                    step: float, obstacle_detected: bool):
        """
        Iterative exploration-based avoidance strategy with anti-stuck mechanisms:
        1. When obstacle detected: BACK UP first for safety margin
        2. After backing up: rotate 90° left or right to explore
        3. Maintain rotated heading and continue forward detection
        4. Track visited cells to avoid redundant exploration
        5. Anti-stuck mechanisms:
           - Direction switch cooldown: wait N cycles after switching for sensor update
           - Stuck counter: reset to direct mode if both directions blocked > 3 times
           - Move counter: return to direct mode after successfully moving 2+ times
        6. Return next waypoint and desired heading

        Returns: (nextN, nextE, desired_yaw)
        """
        t_now_s = self.get_clock().now().nanoseconds * 1e-9

        # Compute direction to goal for reference
        dN = goalN - curN
        dE = goalE - curE
        direct_yaw = radians_wrap(math.atan2(dE, dN))

        # Check for exploration timeout
        if self.exploration_start_time is not None:
            timeout_s = float(self.get_parameter('exploration_timeout_s').value)
            if (t_now_s - self.exploration_start_time) > timeout_s:
                # Reset exploration state after timeout
                self.get_logger().warn("[Exploration] Timeout reached, resetting to direct mode")
                self.current_heading_mode = 'direct'
                self.exploration_start_time = None
                self.locked_heading = None
                self.direction_switch_cooldown = 0
                self.stuck_counter = 0
                self.exploration_move_count = 0
                self.clear_confirmation_count = 0
                self.cooldown_blocked_count = 0
                self._first_explore_logged = False

        # State machine for exploration
        if self.current_heading_mode == 'direct':
            # Direct mode: heading toward goal
            if obstacle_detected:
                # Obstacle ahead! STOP and turn in place (NO BACKUP)
                self.get_logger().warn("[Exploration] Obstacle detected! Turning in place...")

                # Mark current position as visited (blocked in this direction)
                self.mark_visited(curN, curE)
                self.mark_direction_blocked(curN, curE, direct_yaw)
                self.add_to_path(curN, curE, direct_yaw)

                # Choose exploration angle based on stuck counter (starts at 0 = 30°)
                chosen_yaw, chosen_angle_deg, chosen_side = self.choose_exploration_angle(
                    curN, curE, direct_yaw, step, stuck_count=0
                )

                self.get_logger().info(
                    f"[Exploration] Trying {chosen_side.upper()} {chosen_angle_deg}° in place"
                )

                # Set exploration mode
                if chosen_side == 'left':
                    self.current_heading_mode = 'explore_left'
                else:
                    self.current_heading_mode = 'explore_right'

                self.locked_heading = chosen_yaw
                self.exploration_start_time = t_now_s

                # Initialize counters
                self.direction_switch_cooldown = 10
                self.initial_cooldown = 10
                self.stuck_counter = 0
                self.exploration_move_count = 0
                self.explore_start_n = curN
                self.explore_start_e = curE
                self.last_move_n = curN
                self.last_move_e = curE
                self.clear_confirmation_count = 0
                self.cooldown_blocked_count = 0

                # STAY IN PLACE while rotating (no backup)
                return curN, curE, self.locked_heading

            else:
                # No obstacle: move toward goal
                safe_step = min(step, 2.5)

                self.mark_visited(curN, curE)
                self.add_to_path(curN, curE, direct_yaw)
                nxtN, nxtE = self.rrt_next_step(curN, curE, goalN, goalE, safe_step)
                return nxtN, nxtE, direct_yaw

        elif self.current_heading_mode in ['explore_left', 'explore_right']:
            # Exploration mode: maintain locked heading and probe forward
            # Debug: log first entry into exploration mode
            if not hasattr(self, '_first_explore_logged') or not self._first_explore_logged:
                self.get_logger().info(
                    f"[Exploration] Entered {self.current_heading_mode} mode, cooldown={self.direction_switch_cooldown}"
                )
                self._first_explore_logged = True

            # Decrement cooldown timer - SIMPLIFIED for faster response
            if self.direction_switch_cooldown > 0:
                self.direction_switch_cooldown -= 1

                # Calculate minimum mandatory wait period (reduced to 3 cycles for faster response)
                min_wait_cycles = 3
                initial_cd = getattr(self, 'initial_cooldown', 10)
                in_mandatory_wait = self.direction_switch_cooldown > (initial_cd - min_wait_cycles)

                # Log cooldown status periodically
                if self.direction_switch_cooldown in [8, 5, 3, 0]:
                    self.get_logger().info(
                        f"[Exploration] Cooldown: {self.direction_switch_cooldown}, obstacle={obstacle_detected}, "
                        f"mandatory_wait={in_mandatory_wait}, clear_count={self.clear_confirmation_count}"
                    )

                # During MANDATORY wait period, ignore sensor (let drone stabilize)
                if in_mandatory_wait:
                    return curN, curE, self.locked_heading

                # After mandatory wait, check sensor with REDUCED confirmation (3 instead of 5)
                if not obstacle_detected:
                    self.clear_confirmation_count += 1
                    self.cooldown_blocked_count = 0
                    self.get_logger().info(
                        f"[Exploration] Clear during cooldown (remaining={self.direction_switch_cooldown}, count={self.clear_confirmation_count})"
                    )
                    # Require only 3 consecutive clear cycles for faster response
                    if self.clear_confirmation_count >= 3:
                        self.get_logger().info("[Exploration] Clear confirmed (3 consecutive), ending cooldown")
                        self.direction_switch_cooldown = 0
                        self.clear_confirmation_count = 0
                        # Fall through to movement logic
                    else:
                        return curN, curE, self.locked_heading
                else:
                    # Still blocked during cooldown
                    self.clear_confirmation_count = 0
                    self.cooldown_blocked_count += 1

                    # If blocked for 5+ cycles after mandatory wait, switch direction faster
                    if self.cooldown_blocked_count >= 5:
                        self.get_logger().warn(
                            f"[Exploration] Still blocked after {self.cooldown_blocked_count} cycles, ending cooldown to try new angle"
                        )
                        self.direction_switch_cooldown = 0
                        self.cooldown_blocked_count = 0
                        # Fall through to blocked logic to switch direction
                    else:
                        return curN, curE, self.locked_heading

            if obstacle_detected:
                # Still blocked in this direction
                self.mark_visited(curN, curE)
                self.mark_direction_blocked(curN, curE, self.locked_heading)
                self.add_to_path(curN, curE, self.locked_heading)

                # Increment stuck counter
                self.stuck_counter += 1

                # Check if we're truly stuck (both directions blocked repeatedly)
                if self.stuck_counter > 3:
                    self.get_logger().warn(
                        f"[Exploration] Stuck counter={self.stuck_counter}, resetting to direct mode"
                    )
                    # Reset to direct mode and clear state
                    self.current_heading_mode = 'direct'
                    self.exploration_start_time = None
                    self.locked_heading = None
                    self.stuck_counter = 0
                    self.exploration_move_count = 0
                    self.clear_confirmation_count = 0
                    self.cooldown_blocked_count = 0
                    self._first_explore_logged = False
                    return curN, curE, direct_yaw

                # Try switching direction with PROGRESSIVE angle increase
                self.get_logger().info(
                    f"[Exploration] {self.current_heading_mode} blocked (stuck={self.stuck_counter}), "
                    f"trying angle based on stuck count"
                )

                # Choose angle based on stuck counter: 30°→45°→60°→90°
                chosen_yaw, chosen_angle_deg, chosen_side = self.choose_exploration_angle(
                    curN, curE, direct_yaw, step, stuck_count=self.stuck_counter
                )

                self.get_logger().info(
                    f"[Exploration] Switching to {chosen_side.upper()} {chosen_angle_deg}° (stuck={self.stuck_counter})"
                )

                # Update mode and heading
                if chosen_side == 'left':
                    self.current_heading_mode = 'explore_left'
                else:
                    self.current_heading_mode = 'explore_right'

                self.locked_heading = chosen_yaw

                # Set REDUCED cooldown for faster switching
                self.direction_switch_cooldown = 8
                self.initial_cooldown = 8
                self.cooldown_blocked_count = 0
                self.clear_confirmation_count = 0
                self.get_logger().info(
                    f"[Exploration] Switched direction, cooldown={self.direction_switch_cooldown}"
                )
                return curN, curE, self.locked_heading

            else:
                # Clear ahead in exploration direction! Move forward
                # Use REDUCED step size for safety (better obstacle detection)
                safe_step = min(step, 2.5)  # Limit to 2.5m max to stay within sensor range

                self.mark_visited(curN, curE)
                self.add_to_path(curN, curE, self.locked_heading)
                dirN = math.cos(self.locked_heading)
                dirE = math.sin(self.locked_heading)
                nxtN = curN + dirN * safe_step
                nxtE = curE + dirE * safe_step

                # Check if this is a REAL move (position actually changed from last cycle)
                if self.last_move_n is not None and self.last_move_e is not None:
                    actual_dist = math.hypot(curN - self.last_move_n, curE - self.last_move_e)
                else:
                    actual_dist = 0.0

                # Only count as a move if we actually moved significantly (> 0.5m)
                if actual_dist > 0.5:
                    # Reset stuck counter when we successfully move
                    self.stuck_counter = 0
                    self.exploration_move_count += 1

                    # Calculate distance from exploration start
                    dist_from_start = math.hypot(curN - self.explore_start_n, curE - self.explore_start_e)

                    self.get_logger().info(
                        f"[Exploration] Real move #{self.exploration_move_count}: "
                        f"moved {actual_dist:.2f}m, dist_from_start={dist_from_start:.2f}m, "
                        f"pos=({curN:.2f}, {curE:.2f})"
                    )
                else:
                    self.get_logger().info(
                        f"[Exploration] Commanded move but actual_dist={actual_dist:.2f}m (too small, not counting)"
                    )

                # Update last move position
                self.last_move_n = curN
                self.last_move_e = curE

                # After moving forward successfully AND far enough from start, try returning to direct mode
                dist_from_explore_start = math.hypot(curN - self.explore_start_n, curE - self.explore_start_e)

                if self.exploration_move_count >= 3 and dist_from_explore_start >= 5.0:
                    self.get_logger().info(
                        f"[Exploration] Moved {self.exploration_move_count} times, "
                        f"dist_from_start={dist_from_explore_start:.2f}m >= 5.0m, returning to direct mode"
                    )
                    self.current_heading_mode = 'direct'
                    self.exploration_start_time = None
                    self.locked_heading = None
                    self.stuck_counter = 0
                    self.exploration_move_count = 0
                    self.explore_start_n = None
                    self.explore_start_e = None
                    self.last_move_n = None
                    self.last_move_e = None
                    self.clear_confirmation_count = 0
                    self.cooldown_blocked_count = 0
                    self._first_explore_logged = False

                return nxtN, nxtE, self.locked_heading

        # Fallback
        return curN, curE, direct_yaw

    # ---------------- Main Loop ----------------

    def loop(self):
        # Heartbeat messages every cycle
        self.publish_offboard_mode()

        t_now_ns = self.get_clock().now().nanoseconds

        # State machine
        if self.stage == Stage.WARMUP:
            # continuously publish a hold setpoint at current (px,py), target zD = -takeoff_alt
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)  # down is positive, so up = negative
            yaw = self.heading
            self.publish_sp(self.px, self.py, zD_goal, yaw)

            # After >= 0.5s of warmup, arm + offboard
            if (t_now_ns - self.stage_enter_ts) * 1e-9 >= 0.5:
                self.arm()
                self.set_offboard()
                self.stage = Stage.ARMING
                self.stage_enter_ts = t_now_ns
                self.get_logger().info("[stage] WARMUP -> ARMING")
                return

        elif self.stage == Stage.ARMING:
            # keep sending the same takeoff setpoint until we see z_valid and begin moving
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)

            # move to TAKEOFF once z_valid (height feedback) is available
            if self.z_valid:
                self.stage = Stage.TAKEOFF
                self.stage_enter_ts = t_now_ns
                self.get_logger().info("[stage] ARMING -> TAKEOFF")
                return

        elif self.stage == Stage.TAKEOFF:
            # keep commanding the takeoff altitude until reached (within tolerance)
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)

            alt_err = abs(self.pz - zD_goal)  # both in Down
            if alt_err <= 0.25:  # within 25cm
                self.stage = Stage.HOVER_STABLE
                self.stage_enter_ts = t_now_ns
                self.get_logger().info("[stage] TAKEOFF -> HOVER_STABLE")
                return

        elif self.stage == Stage.HOVER_STABLE:
            # hold altitude for some time to stabilize, wait for valid xy too
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)

            stable_hover_s = float(self.get_parameter('stable_hover_s').value)
            min_nav_alt_m   = float(self.get_parameter('min_nav_alt_m').value)
            if (t_now_ns - self.stage_enter_ts) * 1e-9 >= stable_hover_s and self.xy_valid and self.z_valid:
                # ensure we're above min_nav_alt_m (i.e., |zD| >= min_nav_alt_m)
                if abs(self.pz) >= min_nav_alt_m:
                    self.stage = Stage.PURSUIT
                    self.stage_enter_ts = t_now_ns
                    self.get_logger().info("[stage] HOVER_STABLE -> PURSUIT")
                    return

        elif self.stage == Stage.PURSUIT:
            # Convert target ENU to NED goal (if target exists), else hold
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)  # maintain altitude
            if self.tgt_e is None or self.tgt_n is None:
                # No target yet; hold position at current x/y and altitude
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                return

            goalN, goalE, goalD = enu_to_ned(self.tgt_e, self.tgt_n, self.tgt_u if self.tgt_u is not None else takeoff_alt_m)

            # Use configurable step toward goal
            step_xy = float(self.get_parameter('step_size_m').value)

            # Iterative exploration-based avoidance
            use_exploration = bool(self.get_parameter('use_iterative_exploration').value)
            if use_exploration:
                # Use iterative exploration strategy: rotate 90° when blocked, track visited cells
                nxtN, nxtE, yaw = self.iterative_exploration_step(
                    self.px, self.py, goalN, goalE, step_xy, self.obstacle_detected
                )
            else:
                # Simple direct pursuit (no avoidance)
                nxtN, nxtE = self.rrt_next_step(self.px, self.py, goalN, goalE, step=step_xy)
                yaw = self.desired_yaw_towards(self.px, self.py, goalN, goalE)

            # Publish the setpoint at target altitude
            self.publish_sp(nxtN, nxtE, zD_goal, yaw)

        else:
            # Failsafe: publish hold
            self.publish_sp(self.px, self.py, self.pz, self.heading)

def main():
    rclpy.init()
    node = RRTPursuitFixed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
