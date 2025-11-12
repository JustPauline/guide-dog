#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseStamped

class PathToPoseArray(Node):
    def __init__(self):
        super().__init__('path_to_pose_array')
        self.sub = self.create_subscription(Path, '/plan', self.callback, 10)
        self.pub = self.create_publisher(PoseArray, '/plan_poses', 10)

    def callback(self, msg: Path):
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.poses = [p.pose for p in msg.poses]
        self.pub.publish(pose_array)

def main():
    rclpy.init()
    node = PathToPoseArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
