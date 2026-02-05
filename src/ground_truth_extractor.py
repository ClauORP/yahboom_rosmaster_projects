#!/usr/bin/env python3
"""
ground_truth_extractor.py

Goal:
  Gazebo (gz) can publish a PoseArray containing MANY poses (often one per link).
  Nav2 and other ROS tools usually want a SINGLE PoseStamped for "the robot pose".

What this node does:
  - Subscribes to a PoseArray topic (Gazebo's dynamic pose info)
  - Publishes that chosen pose as a PoseStamped on /ground_truth_pose
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, PoseStamped

class GroundTruthExtractor(Node):
    def __init__(self):
        super().__init__('ground_truth_extractor')

        self.declare_parameter('in_topic', '/world/p2_world/dynamic_pose/info')
        self.declare_parameter('out_topic', '/ground_truth_pose')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('pose_index', 1)

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pose_index = int(self.get_parameter('pose_index').value)

        self.sub = self.create_subscription(PoseArray, in_topic, self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(PoseStamped, out_topic, 10)

        self.get_logger().info(f"Subscribing: {in_topic} (PoseArray)")
        self.get_logger().info(f"Publishing:  {out_topic} (PoseStamped), frame_id='{self.frame_id}', pose_index={self.pose_index}")

    def cb(self, msg: PoseArray):
        if not msg.poses:
            return
        if self.pose_index < 0 or self.pose_index >= len(msg.poses):
            self.get_logger().warn(f"pose_index {self.pose_index} out of range (0..{len(msg.poses)-1})")
            return

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.pose = msg.poses[self.pose_index]
        self.pub.publish(out)

def main():
    rclpy.init()
    node = GroundTruthExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
