#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

import tf2_ros


@dataclass
class GridMeta:
    width: int
    height: int
    res: float
    origin_x: float
    origin_y: float


def idx(x: int, y: int, w: int) -> int:
    return y * w + x


def in_bounds(x: int, y: int, w: int, h: int) -> bool:
    return 0 <= x < w and 0 <= y < h


class FrontierExplorer(Node):
    """
    Active SLAM / Frontier Exploration node (simulation-first, Jazzy-ready).

    - Subscribes: /map (nav_msgs/OccupancyGrid)
    - Uses TF: map -> base_link
    - Action: /navigate_to_pose (nav2_msgs/action/NavigateToPose)
    - Publishes: zero-velocity burst to cmd_vel_topic for safety stop on shutdown.
      * If cmd_vel_topic targets ros2_control mecanum controller: TwistStamped
      * If cmd_vel_topic targets Nav2 /cmd_vel: Twist
    """

    def __init__(self):
        super().__init__("frontier_explorer")

        # -----------------------------
        # Parameters (tune in launch/yaml)
        # -----------------------------
        self.declare_parameter("map_topic", "/map")

        # IMPORTANT: this can be either:
        #  - "/mecanum_drive_controller/cmd_vel" (TwistStamped)
        #  - "/cmd_vel" (Twist)
        self.declare_parameter("cmd_vel_topic", "/mecanum_drive_controller/cmd_vel")
        self.declare_parameter("cmd_vel_frame_id", "base_link")

        # Nav2 action name: make it robust by defaulting to leading slash
        self.declare_parameter("nav_action_name", "/navigate_to_pose")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Core algorithm knobs
        self.declare_parameter("min_frontier_cluster_size", 30)
        self.declare_parameter("frontier_connectivity", 8)   # 4 or 8
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("free_value_max", 10)          # treat <= this as free
        self.declare_parameter("occupied_value_min", 65)      # treat >= this as occupied

        # Goal selection and safety
        self.declare_parameter("goal_free_search_radius_m", 0.6)     # snap centroid to free
        self.declare_parameter("min_goal_separation_m", 0.4)         # avoid re-picking basically same goal
        self.declare_parameter("obstacle_clearance_radius_m", 0.25)  # do not target too near obstacles
        self.declare_parameter("score_weight_distance", 1.0)
        self.declare_parameter("score_weight_size", 0.15)

        # Loop + failure handling
        self.declare_parameter("loop_period_s", 2.0)
        self.declare_parameter("nav_timeout_s", 120.0)
        self.declare_parameter("blacklist_radius_m", 0.6)
        self.declare_parameter("blacklist_ttl_s", 300.0)      # forget bad goals after some time
        self.declare_parameter("max_consecutive_failures", 8)

        # Stop condition
        self.declare_parameter("unknown_fraction_done", 0.02)  # stop if unknown fraction below threshold
        self.declare_parameter("min_frontier_clusters_to_continue", 1)

        # Shutdown safety behavior
        self.declare_parameter("zero_cmd_burst_duration_s", 1.0)
        self.declare_parameter("zero_cmd_burst_hz", 15.0)

        # -----------------------------
        # State
        # -----------------------------
        self.map: Optional[OccupancyGrid] = None
        self.map_meta: Optional[GridMeta] = None
        self.map_data: Optional[List[int]] = None

        self.exploring: bool = True
        self.nav_in_flight: bool = False
        self.nav_start_time: Optional[float] = None
        self.current_goal_xy: Optional[Tuple[float, float]] = None
        self.consecutive_failures: int = 0

        # blacklist: (x,y)-> expiry_time
        self.blacklist: List[Tuple[float, float, float]] = []  # (x, y, expiry_epoch)

        # -----------------------------
        # QoS: maps are often transient local in Nav2/SLAM setups
        # -----------------------------
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            map_qos,
        )

        # -----------------------------
        # cmd_vel publisher (Twist vs TwistStamped)
        # -----------------------------
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._cmd_vel_topic = cmd_vel_topic
        self._cmd_vel_frame_id = self.get_parameter("cmd_vel_frame_id").value

        # Simple rule: if topic contains mecanum_drive_controller, it's TwistStamped.
        # Otherwise assume Twist.
        if "mecanum_drive_controller" in cmd_vel_topic:
            self._cmd_is_stamped = True
            self.cmd_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
            self.get_logger().info(
                f"cmd_vel_topic='{cmd_vel_topic}' -> publishing TwistStamped for safety stop."
            )
        else:
            self._cmd_is_stamped = False
            self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.get_logger().info(
                f"cmd_vel_topic='{cmd_vel_topic}' -> publishing Twist for safety stop."
            )

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("nav_action_name").value
        )
        self.nav_goal_handle = None

        # Main loop timer
        period = float(self.get_parameter("loop_period_s").value)
        self.timer = self.create_timer(period, self._tick)

        # Ensure safe shutdown on Ctrl+C
        self._shutdown_started = False

        self.get_logger().info("FrontierExplorer node started. Waiting for /map and Nav2 action server...")

    # -----------------------------
    # Map callback
    # -----------------------------
    def _on_map(self, msg: OccupancyGrid):
        self.map = msg
        self.map_meta = GridMeta(
            width=msg.info.width,
            height=msg.info.height,
            res=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y
        )
        self.map_data = list(msg.data)

    # -----------------------------
    # Main loop
    # -----------------------------
    def _tick(self):
        if not self.exploring:
            return

        if self.map is None or self.map_meta is None or self.map_data is None:
            self.get_logger().debug("No map yet...")
            return

        # Wait for Nav2 server
        if not self.nav_client.server_is_ready():
            self.get_logger().info_throttle(5.0, "Waiting for Nav2 action server (/navigate_to_pose)...")
            return

        # If a goal is active, monitor timeout and return
        if self.nav_in_flight:
            self._check_nav_timeout()
            return

        # Clean expired blacklist entries
        self._prune_blacklist()

        # Stop condition based on unknown fraction
        if self._unknown_fraction() <= float(self.get_parameter("unknown_fraction_done").value):
            self.get_logger().info("Stop condition met: unknown fraction below threshold. Exploration complete.")
            self._finish_exploration()
            return

        # 1) Detect frontier cells
        frontiers = self._detect_frontiers()

        # 2) Cluster them
        clusters = self._cluster_frontiers(frontiers)

        min_clusters = int(self.get_parameter("min_frontier_clusters_to_continue").value)
        if len(clusters) < min_clusters:
            self.get_logger().info("Stop condition met: no valid frontier clusters remain.")
            self._finish_exploration()
            return

        # 3) Select best goal
        robot_xy = self._get_robot_xy()
        if robot_xy is None:
            self.get_logger().warn_throttle(5.0, "No TF map->base_link yet; cannot select goal.")
            return

        goal = self._select_goal(clusters, robot_xy)
        if goal is None:
            self.get_logger().warn("No valid goal found after filtering/blacklist. Marking exploration complete.")
            self._finish_exploration()
            return

        # 4) Send Nav2 goal
        self._send_nav_goal(goal)

    # -----------------------------
    # Frontier detection
    # A cell is frontier if it is unknown (-1) and touches at least one free neighbor (0-ish)
    # -----------------------------
    def _detect_frontiers(self) -> List[Tuple[int, int]]:
        assert self.map_meta is not None and self.map_data is not None
        w, h = self.map_meta.width, self.map_meta.height
        unknown_val = int(self.get_parameter("unknown_value").value)
        free_max = int(self.get_parameter("free_value_max").value)

        conn = int(self.get_parameter("frontier_connectivity").value)
        if conn == 4:
            neigh = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        else:
            neigh = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        frontiers: List[Tuple[int, int]] = []
        data = self.map_data

        # Skip border cells
        for y in range(1, h - 1):
            row_base = y * w
            for x in range(1, w - 1):
                v = data[row_base + x]
                if v != unknown_val:
                    continue

                touches_free = False
                for dx, dy in neigh:
                    nx, ny = x + dx, y + dy
                    nv = data[idx(nx, ny, w)]
                    if 0 <= nv <= free_max:
                        touches_free = True
                        break

                if touches_free:
                    frontiers.append((x, y))

        self.get_logger().debug(f"Detected {len(frontiers)} frontier cells.")
        return frontiers

    # -----------------------------
    # Frontier clustering: connected components of frontier cells
    # -----------------------------
    def _cluster_frontiers(self, frontier_cells: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        assert self.map_meta is not None
        w, h = self.map_meta.width, self.map_meta.height
        frontier_set: Set[Tuple[int, int]] = set(frontier_cells)
        visited: Set[Tuple[int, int]] = set()

        min_sz = int(self.get_parameter("min_frontier_cluster_size").value)

        conn = int(self.get_parameter("frontier_connectivity").value)
        if conn == 4:
            neigh = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        else:
            neigh = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        clusters: List[List[Tuple[int, int]]] = []

        for cell in frontier_cells:
            if cell in visited:
                continue
            if cell not in frontier_set:
                continue

            stack = [cell]
            visited.add(cell)
            comp: List[Tuple[int, int]] = []

            while stack:
                cx, cy = stack.pop()
                comp.append((cx, cy))
                for dx, dy in neigh:
                    nx, ny = cx + dx, cy + dy
                    if not in_bounds(nx, ny, w, h):
                        continue
                    ncell = (nx, ny)
                    if ncell in frontier_set and ncell not in visited:
                        visited.add(ncell)
                        stack.append(ncell)

            if len(comp) >= min_sz:
                clusters.append(comp)

        self.get_logger().info(f"Frontier clusters: {len(clusters)} (min size={min_sz})")
        return clusters

    # -----------------------------
    # Goal selection: centroid -> snap to free -> filter by clearance/blacklist -> score
    # -----------------------------
    def _select_goal(
        self,
        clusters: List[List[Tuple[int, int]]],
        robot_xy: Tuple[float, float]
    ) -> Optional[Tuple[float, float, float]]:
        assert self.map_meta is not None

        scored: List[Tuple[float, Tuple[float, float, float]]] = []

        w_dist = float(self.get_parameter("score_weight_distance").value)
        w_size = float(self.get_parameter("score_weight_size").value)

        for comp in clusters:
            cx = sum(p[0] for p in comp) / float(len(comp))
            cy = sum(p[1] for p in comp) / float(len(comp))

            wx, wy = self._grid_to_world(cx, cy)

            snapped = self._snap_to_free(wx, wy)
            if snapped is None:
                continue
            gx, gy = snapped

            if not self._has_obstacle_clearance(gx, gy):
                continue

            if self._is_blacklisted(gx, gy):
                continue

            if self.current_goal_xy is not None:
                if self._dist((gx, gy), self.current_goal_xy) < float(self.get_parameter("min_goal_separation_m").value):
                    continue

            d = self._dist((gx, gy), robot_xy)
            size = len(comp)
            score = w_dist * d - w_size * float(size)

            yaw = math.atan2(gy - robot_xy[1], gx - robot_xy[0])
            scored.append((score, (gx, gy, yaw)))

        if not scored:
            return None

        scored.sort(key=lambda t: t[0])
        best = scored[0][1]
        self.get_logger().info(f"Selected goal: x={best[0]:.2f}, y={best[1]:.2f}, yaw={best[2]:.2f}")
        return best

    # -----------------------------
    # Nav2 action handling
    # -----------------------------
    def _send_nav_goal(self, goal_xyz: Tuple[float, float, float]):
        gx, gy, yaw = goal_xyz

        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter("global_frame").value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(gx)
        pose.pose.position.y = float(gy)
        pose.pose.position.z = 0.0

        # yaw -> quaternion (z,w)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending NavigateToPose goal...")
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

        self.nav_in_flight = True
        self.nav_start_time = time.time()
        self.current_goal_xy = (gx, gy)

    def _on_goal_response(self, future):
        if self._shutdown_started:
            return

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            self.nav_in_flight = False
            self.nav_goal_handle = None
            self._on_goal_failed()
            return

        self.get_logger().info("Nav2 goal accepted.")
        self.nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        if self._shutdown_started:
            return

        res = future.result()
        self.nav_in_flight = False

        status = getattr(res, "status", None)
        self.get_logger().info(f"Nav2 goal finished with status={status}")

        # GoalStatus.STATUS_SUCCEEDED == 4
        if status == 4:
            self.consecutive_failures = 0
        else:
            self._on_goal_failed()

        self.nav_goal_handle = None
        self.nav_start_time = None

    def _check_nav_timeout(self):
        if self.nav_start_time is None:
            return
        timeout_s = float(self.get_parameter("nav_timeout_s").value)
        if (time.time() - self.nav_start_time) > timeout_s:
            self.get_logger().warn(f"Nav2 timeout after {timeout_s:.0f}s. Cancelling goal and blacklisting it.")
            self._blacklist_current_goal()
            self._cancel_nav_goal()

            self.nav_in_flight = False
            self.nav_start_time = None
            self.nav_goal_handle = None
            self._on_goal_failed()

    def _on_goal_failed(self):
        self.consecutive_failures += 1
        self.get_logger().warn(f"Goal failed. consecutive_failures={self.consecutive_failures}")

        self._blacklist_current_goal()

        if self.consecutive_failures >= int(self.get_parameter("max_consecutive_failures").value):
            self.get_logger().error("Too many consecutive failures. Stopping exploration.")
            self._finish_exploration()

    def _cancel_nav_goal(self):
        if self.nav_goal_handle is None:
            return
        try:
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Cancel request sent to Nav2."))
        except Exception as e:
            self.get_logger().warn(f"Failed to cancel goal: {e}")

    # -----------------------------
    # Stop conditions / finishing
    # -----------------------------
    def _finish_exploration(self):
        self.exploring = False
        if self.timer is not None:
            self.timer.cancel()
        self._publish_zero_cmd_burst()
        self.get_logger().info("Exploration stopped.")

    def _unknown_fraction(self) -> float:
        assert self.map_data is not None
        unknown_val = int(self.get_parameter("unknown_value").value)
        total = len(self.map_data)
        if total == 0:
            return 1.0
        unk = sum(1 for v in self.map_data if v == unknown_val)
        return float(unk) / float(total)

    # -----------------------------
    # TF helpers
    # -----------------------------
    def _get_robot_xy(self) -> Optional[Tuple[float, float]]:
        global_frame = self.get_parameter("global_frame").value
        base_frame = self.get_parameter("base_frame").value

        try:
            tf = self.tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    # -----------------------------
    # Grid/world conversion + checks
    # -----------------------------
    def _grid_to_world(self, gx: float, gy: float) -> Tuple[float, float]:
        assert self.map_meta is not None
        wx = self.map_meta.origin_x + (gx + 0.5) * self.map_meta.res
        wy = self.map_meta.origin_y + (gy + 0.5) * self.map_meta.res
        return wx, wy

    def _world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        assert self.map_meta is not None
        gx = int((wx - self.map_meta.origin_x) / self.map_meta.res)
        gy = int((wy - self.map_meta.origin_y) / self.map_meta.res)
        return gx, gy

    def _snap_to_free(self, wx: float, wy: float) -> Optional[Tuple[float, float]]:
        assert self.map_meta is not None and self.map_data is not None
        w, h = self.map_meta.width, self.map_meta.height
        free_max = int(self.get_parameter("free_value_max").value)

        r_m = float(self.get_parameter("goal_free_search_radius_m").value)
        r_cells = max(1, int(r_m / self.map_meta.res))

        cx, cy = self._world_to_grid(wx, wy)

        def is_free(x: int, y: int) -> bool:
            if not in_bounds(x, y, w, h):
                return False
            v = self.map_data[idx(x, y, w)]
            return 0 <= v <= free_max

        if is_free(cx, cy):
            return (wx, wy)

        for rr in range(1, r_cells + 1):
            for dx in range(-rr, rr + 1):
                for dy in (-rr, rr):
                    x, y = cx + dx, cy + dy
                    if is_free(x, y):
                        return self._grid_to_world(x, y)
            for dy in range(-rr + 1, rr):
                for dx in (-rr, rr):
                    x, y = cx + dx, cy + dy
                    if is_free(x, y):
                        return self._grid_to_world(x, y)

        return None

    def _has_obstacle_clearance(self, wx: float, wy: float) -> bool:
        assert self.map_meta is not None and self.map_data is not None
        w, h = self.map_meta.width, self.map_meta.height
        occ_min = int(self.get_parameter("occupied_value_min").value)

        r_m = float(self.get_parameter("obstacle_clearance_radius_m").value)
        r_cells = max(1, int(r_m / self.map_meta.res))

        cx, cy = self._world_to_grid(wx, wy)
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                x, y = cx + dx, cy + dy
                if not in_bounds(x, y, w, h):
                    continue
                v = self.map_data[idx(x, y, w)]
                if v >= occ_min:
                    return False
        return True

    # -----------------------------
    # Blacklist
    # -----------------------------
    def _blacklist_current_goal(self):
        if self.current_goal_xy is None:
            return
        gx, gy = self.current_goal_xy
        ttl = float(self.get_parameter("blacklist_ttl_s").value)
        expiry = time.time() + ttl
        self.blacklist.append((gx, gy, expiry))

    def _is_blacklisted(self, gx: float, gy: float) -> bool:
        r = float(self.get_parameter("blacklist_radius_m").value)
        now = time.time()
        for bx, by, exp in self.blacklist:
            if exp < now:
                continue
            if self._dist((gx, gy), (bx, by)) <= r:
                return True
        return False

    def _prune_blacklist(self):
        now = time.time()
        self.blacklist = [(x, y, exp) for (x, y, exp) in self.blacklist if exp >= now]

    # -----------------------------
    # Safety stop burst (zeros)
    # -----------------------------
    def _publish_zero_cmd_burst(self):
        dur = float(self.get_parameter("zero_cmd_burst_duration_s").value)
        hz = float(self.get_parameter("zero_cmd_burst_hz").value)
        if hz <= 0.0:
            hz = 10.0
        dt = 1.0 / hz
        n = max(1, int(dur * hz))

        for _ in range(n):
            if self._cmd_is_stamped:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._cmd_vel_frame_id
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.0
                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0

            self.cmd_pub.publish(msg)
            time.sleep(dt)

    # -----------------------------
    # Utility
    # -----------------------------
    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # -----------------------------
    # Clean shutdown hook
    # -----------------------------
    def safe_shutdown(self, reason: str = "shutdown"):
        if self._shutdown_started:
            return
        self._shutdown_started = True

        self.get_logger().warn(f"Safe shutdown initiated: {reason}")

        # 1) Freeze exploration logic immediately
        self.exploring = False
        if self.timer is not None:
            try:
                self.timer.cancel()
            except Exception:
                pass

        # 2) Cancel active goal
        try:
            self._cancel_nav_goal()
        except Exception:
            pass

        # 3) Publish zero cmd burst
        try:
            self._publish_zero_cmd_burst()
        except Exception:
            pass

        self.get_logger().info("Shutdown complete.")


def main():
    rclpy.init()
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.safe_shutdown("KeyboardInterrupt (Ctrl+C)")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()