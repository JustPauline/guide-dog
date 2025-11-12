from sensor_msgs.msg import JointState
import time
import rclpy
from rclpy.node import Node
import numpy as np
import math
import sys
import struct
from builtin_interfaces.msg import Time
sys.path.insert(0,"/home/pi/dogzilla_ws/src/DOGZILLALib/DOGZILLALib")
import DOGZILLALib as dog

"""
"lf_hip_joint", 左边肩部关节
"lf_lower_leg_joint",  左边前腿底部关节
"lf_upper_leg_joint", 左边前腿上部关节


"lh_lower_leg_joint", 左边后腿底部关节
"lh_upper_leg_joint", 左边后腿上部电机
"lh_hip_joint", 左边臀部关节

"rf_hip_joint", 右边肩部电机
"rf_lower_leg_joint", 右边前腿底部关节
"rf_upper_leg_joint", 右边前腿上部关节

"rh_hip_joint", 右边臀部关节
"rh_lower_leg_joint", 右边后腿底部关节
"rh_upper_leg_joint" 右边后腿上部关节
"""


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('yahboom_dog_joint_state')
        self.dogControl = dog.DOGZILLA()
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.timer_period = 0.05 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.last_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_time = time.time()

    def timer_callback(self):
        try:
            msg = JointState()
            t = self.get_clock().now()
            msg.header.stamp = t.to_msg()
            msg.name = [
                "lf_upper_leg_joint", "lf_lower_leg_joint", "lf_hip_joint",
                "lh_upper_leg_joint", "lh_lower_leg_joint", "lh_hip_joint",
                "rf_upper_leg_joint", "rf_lower_leg_joint", "rf_hip_joint",
                "rh_upper_leg_joint", "rh_lower_leg_joint", "rh_hip_joint"
            ]

            angle = self.dogControl.read_motor()

            if len(angle) != 12:
                # self.get_logger().warn("Motor data incomplete. Skipping publish.")
                return  # Don't publish this time, data is corrupted/incomplete

            msg.position = [angle[0] * np.pi / 180, -angle[1] * np.pi / 180, angle[2] * np.pi / 180,
                            angle[3] * np.pi / 180, -angle[4] * np.pi / 180, angle[5] * np.pi / 180,
                            -angle[6] * np.pi / 180, angle[7] * np.pi / 180, angle[8] * np.pi / 180,
                            -angle[9] * np.pi / 180, angle[10] * np.pi / 180, angle[11] * np.pi / 180]

            msg.velocity = [
                (angle[i] * np.pi / 180 - self.last_state[i] * np.pi / 180) / self.timer_period
                for i in range(12)
            ]

            self.last_state = angle

            msg.effort = [float("nan")] * 12
            self.publisher_.publish(msg)
            time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
