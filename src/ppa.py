#!/usr/bin/env python3
"""
Starter code for Project 3: Holonomic Pure Pursuit controller.

Students must complete the missing parts:
1. Find the closest point on the square path
2. Compute the lookahead ("carrot") point
3. Transform error into the robot frame
4. Generate velocity commands

This node:
- Subscribes to robot pose
- Publishes TwistStamped commands
- Repeats the square path for a fixed number of cycles
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class HolonomicPurePursuit(Node):
    def __init__(self) -> None:
        super().__init__("holonomic_pure_pursuit")

        # Parameters
        self.declare_parameter("side_length_m", 3.0)
        self.declare_parameter("cycles", 3)
        self.declare_parameter("lookahead_m", 0.3)
        self.declare_parameter("publish_hz", 30.0)

        self.declare_parameter("kx", 0.9)
        self.declare_parameter("ky", 1.2)

        self.declare_parameter("vx_max", 0.35)
        self.declare_parameter("vy_max", 0.35)

        self.declare_parameter("cmd_topic", "/mecanum_drive_controller/cmd_vel")
        self.declare_parameter("pose_topic", "/mecanum_drive_controller/odom")

        self.side = float(self.get_parameter("side_length_m").value)
        self.cycles_total = int(self.get_parameter("cycles").value)
        self.Ld = float(self.get_parameter("lookahead_m").value)
        self.hz = float(self.get_parameter("publish_hz").value)

        self.kx = float(self.get_parameter("kx").value)
        self.ky = float(self.get_parameter("ky").value)

        self.vx_max = float(self.get_parameter("vx_max").value)
        self.vy_max = float(self.get_parameter("vy_max").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)

        # Square path
        self.waypoints: List[Tuple[float, float]] = [
            (1.0, 0.0),
            (4.0, 0.0),
            (4.0, 3.0),
            (1.0, 3.0),
            (1.0, 0.0),
        ]

        # Precompute path lengths
        self.seg_lengths: List[float] = []
        self.cum_s: List[float] = [0.0]
        for i in range(len(self.waypoints) - 1):
            x0, y0 = self.waypoints[i]
            x1, y1 = self.waypoints[i + 1]
            L = math.hypot(x1 - x0, y1 - y0)
            self.seg_lengths.append(L)
            self.cum_s.append(self.cum_s[-1] + L)
        self.path_length = self.cum_s[-1]

        # Robot state
        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.s_progress = 0.0
        self.completed_cycles = 0

        # ROS interfaces
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.pose_sub = self.create_subscription(
            Odometry, self.pose_topic, self._on_odom, 20
        )
        # Optional for world pose:
        # self.pose_sub = self.create_subscription(
        #     PoseStamped, self.pose_topic, self._on_pose, 20
        # )

        self.timer = self.create_timer(1.0 / self.hz, self._on_timer)

        self.get_logger().info("Holonomic Pure Pursuit starter node running.")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

    def _on_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.x = float(p.x)
        self.y = float(p.y)
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_pose = True

    def _on_timer(self) -> None:
        if not self.have_pose:
            return

        if self.completed_cycles >= self.cycles_total:
            self._publish_stop()
            return

        # --------------------------------------------------
        # TODO 1:
        # Find the closest point on the path to the robot.
        # This should return:
        #   s_closest -> distance along the path
        #   dist      -> Euclidean distance from robot to path
        # --------------------------------------------------
        s_closest, dist = self._closest_s_on_loop(self.x, self.y)

        # --------------------------------------------------
        # TODO 2:
        # Update path progress.
        # For a closed loop, think carefully about what happens
        # near the start/end of the square.
        # --------------------------------------------------
        self.s_progress = max(self.s_progress, s_closest)

        # --------------------------------------------------
        # TODO 3:
        # Define the lookahead point at s_goal = progress + Ld
        # Then compute the corresponding path point (xg, yg)
        # --------------------------------------------------
        s_goal = self.s_progress + self.Ld
        xg, yg = self._point_at_s(s_goal)

        # Count cycles approximately
        if s_goal >= (self.completed_cycles + 1) * self.path_length:
            self.completed_cycles += 1
            if self.completed_cycles >= self.cycles_total:
                self._publish_stop()
                return

        # Error in world frame
        dx = xg - self.x
        dy = yg - self.y

        # --------------------------------------------------
        # TODO 4:
        # Convert (dx, dy) from world frame to robot frame
        # Use robot yaw.
        # The result should be:
        #   x_r = forward error
        #   y_r = lateral error
        # --------------------------------------------------
        x_r = 0.0
        y_r = 0.0

        # --------------------------------------------------
        # TODO 5:
        # Generate velocity commands from x_r and y_r
        # using proportional control, then clamp them.
        # Do not use angular velocity.
        # --------------------------------------------------
        vx = 0.0
        vy = 0.0
        omega = 0.0

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(vx)
        cmd.twist.linear.y = float(vy)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def _closest_s_on_loop(self, x: float, y: float) -> Tuple[float, float]:
        """
        TODO:
        Find the closest point on the square path to the robot.
        Suggested approach:
        - Loop over each line segment
        - Project the robot position onto the segment
        - Clamp projection to the segment endpoints
        - Keep the projection with the minimum distance
        """
        return 0.0, 0.0

    def _point_at_s(self, s_global: float) -> Tuple[float, float]:
        """
        TODO:
        Convert distance-along-path s into a point (x, y) on the square.
        Suggested approach:
        - Reduce s to one loop of the path
        - Find which segment contains that value
        - Interpolate along that segment
        """
        return self.waypoints[0]

    def _publish_stop(self) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def destroy_node(self) -> bool:
        self._publish_stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HolonomicPurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()