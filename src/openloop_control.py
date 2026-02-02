#!/usr/bin/env python3
"""
openloop_control.py
Open-loop square path for ROSmaster X3 (omnidirectional / mecanum base) in ROS 2 Jazzy.

Key idea:
- We DO NOT use odom/ground truth to correct the motion.
- We just command constant velocities for fixed durations (time-based control).
- Because it's open-loop, drift will accumulate over cycles.

Publishes:
- /mecanum_drive_controller/cmd_vel  (geometry_msgs/TwistStamped)

Motion (no in-place rotations required for mecanum):
- +x (forward) for side_length
- +y (left)    for side_length
- -x (back)    for side_length
- -y (right)   for side_length
angular.z must stay 0.
Repeat for N cycles.

Run:
  ros2 run yahboom_rosmaster_projects openloop_control.py

Make sure:
- Your controller is running and listening to TwistStamped on:
  /mecanum_drive_controller/cmd_vel
"""
# To create the executable, run: chmod +x ~/ros2_ws/src/yahboom_rosmaster/yahboom_rosmaster_projects/src/openloop_control.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class OpenLoopSquare(Node):
    def __init__(self):
        super().__init__("openloop_control")

        # -----------------------------
        # TODO: Tune these values
        # -----------------------------
        self.side_length_m =              # meters (length of one side)
        self.linear_speed_mps =           # m/s
        self.cycles_total =               # number of full squares
        self.pause_s =                    # short stop between legs (optional)

        # TODO: Time to travel one side (distance / speed)
        self.leg_time_s = 

        # Publisher (controller expects TwistStamped)
        self.pub = self.create_publisher(
            TwistStamped,
            "/mecanum_drive_controller/cmd_vel",
            10
        )

        # -----------------------------
        # Open-loop "state machine"
        # -----------------------------
        # leg_index: counts how many legs have been completed
        # Each cycle has 4 legs → total legs = 4 * cycles_total
        self.leg_index = 0
        self.in_pause = False
        self.phase_start = self.get_clock().now()

        v = self.linear_speed_mps
        # TODO: Define the 4 square legs as (vx, vy) with NO rotation.
        # Hint: use +v forward, +v left, -v back, -v right.
        self.legs = [
               # forward
               # left
               # backward
               # right
        ]

        # Timer loop (20 Hz is enough for smooth commands)
        self.timer = self.create_timer(1.0 / 20.0, self.tick)

    def publish_cmd(self, vx, vy):
        """Publish a TwistStamped command. Rotation is NOT used (angular.z = 0)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def tick(self):
        total_legs = 4 * self.cycles_total

        # If finished: keep the robot stopped
        if self.leg_index >= total_legs:
            self.stop()
            return

        now = self.get_clock().now()
        elapsed = (now - self.phase_start).nanoseconds * 1e-9

        # Pause between legs (optional)
        if self.in_pause:
            self.stop()
            if elapsed >= self.pause_s:
                self.in_pause = False
                self.phase_start = now
            return

        # Move along the current leg
        vx, vy = self.legs[self.leg_index % 4]
        self.publish_cmd(vx, vy)
        
        # TODO: When elapsed time reaches leg_time_s, switch to the next leg and optionally start the pause.
        # Required behavior:
        # - increment self.leg_index
        # - reset self.phase_start = now
        # - if pause_s > 0, set self.in_pause = True

def main():
    rclpy.init()
    node = OpenLoopSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

