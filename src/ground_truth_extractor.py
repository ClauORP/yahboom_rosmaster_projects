#!/usr/bin/env python3
"""
ground_truth_extractor.py

Goal:
  Gazebo (gz) can publish a PoseArray containing MANY poses (often one per link).
  Nav2 and other ROS tools usually want a SINGLE PoseStamped for "the robot pose".

What this node does:
  - Subscribes to a PoseArray topic (Gazebo's dynamic pose info)
  - Chooses ONE pose from the array (heuristic: the pose with the largest z value)
  - Publishes that chosen pose as a PoseStamped on /ground_truth_pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped

class GroundTruthExtractor(Node):
    """
    Extract a single "ground truth" pose from Gazebo's PoseArray.

    Why do we need this?
      Gazebo may publish /world/<world_name>/dynamic_pose/info as PoseArray, which includes
      many poses (often for many links). For a robot controller or for analysis, we
      usually want one "robot pose".

    How do we decide which pose is the robot?
      Since PoseArray doesn't include link names, we pick the pose with the largest z
      (height). Typically the base/body is higher than wheels (wheels tend to have z ~ 0).
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('ground_truth_extractor')

        # Input topic that publishes PoseArray from Gazebo
        self.declare_parameter('in_topic', '/world/p2_world/dynamic_pose/info')
        
        # Output topic where we publish a single PoseStamped
        self.declare_parameter('out_topic', '/ground_truth_pose')
        
        # The frame_id we will put in the published PoseStamped header
        self.declare_parameter('frame_id', 'world') 

        # Read parameter values
        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Subscribe to PoseArray coming from Gazebo
        self.sub = self.create_subscription(PoseArray, in_topic, self.cb, 10)
        
        # Publisher for the extracted single pose
        self.pub = self.create_publisher(PoseStamped, out_topic, 10)

        # Print what we're doing
        self.get_logger().info(f"Subscribing: {in_topic} (PoseArray)")
        self.get_logger().info(f"Publishing:  {out_topic} (PoseStamped), frame_id='{self.frame_id}'")


    def cb(self, msg: PoseArray):
        """
        Callback runs every time we receive a PoseArray message.

        msg.poses is a Python list of Pose objects.
        If it's empty, we do nothing.
        """
        if not msg.poses:
            return

        # Heuristic selection
        # Pick pose with maximum z (usually base link / body)
        best = max(msg.poses, key=lambda p: p.position.z)

        # Build a PoseStamped output
        out = PoseStamped()
        
        # Timestamp with THIS node's clock time.
        out.header.stamp = self.get_clock().now().to_msg()
        # Coordinate frame label
        out.header.frame_id = self.frame_id
        # Put the chosen pose into the message
        out.pose = best
        
        # Publish the single pose
        self.pub.publish(out)



def main():
    rclpy.init()
    node = GroundTruthExtractor()
    try:
        # keeps the node alive and callbacks running
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allows clean exit when user presses Ctrl+C
        pass
    finally:
        # Always cleanup ROS resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
