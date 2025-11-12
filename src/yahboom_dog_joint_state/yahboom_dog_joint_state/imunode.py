import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
import board
import busio
from adafruit_bno055 import BNO055_I2C
import math
import time
import os
import json

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = BNO055_I2C(i2c)
        self.sensor.mode = 0  # CONFIG_MODE
        time.sleep(0.5)
        self.sensor.mode = 12  # NDOF
        time.sleep(1.0)

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.05, self.publish_imu_data)

        self.get_logger().info("BNO055 IMU node started")
        self.get_logger().info(f"Mode: {self.sensor.mode}")

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        accel = self.sensor.acceleration
        gyro = self.sensor.gyro
        quat = self.sensor.quaternion

        if accel is not None:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

        if gyro is not None:
            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])

        if quat is not None:
            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]

        msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
