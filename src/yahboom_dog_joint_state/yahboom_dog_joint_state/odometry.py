import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import transforms3d.euler as euler
import math
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Empty
from collections import deque


class OdometryEstimator(Node):
    def __init__(self):
        super().__init__('odometry_estimator')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_joint_positions = {}
        self.last_time = self.get_clock().now()
        self.last_stride_distance = 0.0 

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)

        self.timer = self.create_timer(0.05, self.publish_odometry)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_yaw_from_imu = 0.0
        self.angular_velocity_z = 0.0

        self.yaw_deltas = deque(maxlen=5)  # moving window of last 5 yaw changes
        self.previous_yaw = 0.0


        self.get_logger().info("Odometry estimator node started.")

    def reset_callback(self, request, response):
        self.get_logger().info('Odometry reset requested.')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        return response

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        euler_angles = euler.quat2euler([q.w, q.x, q.y, q.z])
        self.current_yaw_from_imu = euler_angles[2]
        self.angular_velocity_z = msg.angular_velocity.z


    def joint_callback(self, msg: JointState):
        leg_joints = ['lf_hip_joint', 'lh_hip_joint', 'rf_hip_joint', 'rh_hip_joint']
        positions = dict(zip(msg.name, msg.position))

        if not all(j in positions for j in leg_joints):
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        stride_sum = 0.0
        for joint in leg_joints:
            if joint in self.last_joint_positions:
                delta = abs(positions[joint] - self.last_joint_positions[joint])
                stride_sum += delta
            self.last_joint_positions[joint] = positions[joint]

        stride_distance = stride_sum * 0.17
        self.last_stride_distance = stride_distance

        # Compute yaw delta and update yaw history
        yaw_delta = abs(self.current_yaw_from_imu - self.previous_yaw)
        self.previous_yaw = self.current_yaw_from_imu
        self.yaw = self.current_yaw_from_imu

        # Update moving average buffer
        self.yaw_deltas.append(abs(yaw_delta))
        avg_yaw_delta = sum(self.yaw_deltas) / len(self.yaw_deltas)

        # Parameters
        yaw_threshold = 0.06  # rad
        stride_threshold = 0.01  # meters

        # Suppress fake translation if we're turning in place
        if avg_yaw_delta < yaw_threshold:
            self.x += stride_distance * math.cos(self.yaw)
            self.y += stride_distance * math.sin(self.yaw)

    def publish_odometry(self):
        odom = Odometry()
        now = self.get_clock().now().to_msg()

        odom.header.stamp = now
        odom.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        odom.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quat_vals = euler.euler2quat(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(
            x=quat_vals[1], y=quat_vals[2], z=quat_vals[3], w=quat_vals[0]
        )

        odom.pose.covariance = [
            0.01, 0.0,    0.0,    0.0, 0.0, 0.0,
            0.0,    0.01, 0.0,    0.0, 0.0, 0.0,
            0.0,    0.0,    999.0,  0.0, 0.0, 0.0,
            0.0,    0.0,    0.0,  999.0, 0.0, 0.0,
            0.0,    0.0,    0.0,    0.0, 999.0, 0.0,
            0.0,    0.0,    0.0,    0.0, 0.0, 0.1
        ]

        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        linear_vel = self.last_stride_distance / dt if dt > 0 else 0.0

        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_velocity_z

        odom.twist.covariance = [
            0.1, 0.0,    0.0,    0.0, 0.0, 0.0,
            0.0,   0.1,  0.0,    0.0, 0.0, 0.0,
            0.0,   0.0,  999.0,    0.0, 0.0, 0.0,
            0.0,   0.0,    0.0,  999.0, 0.0, 0.0,
            0.0,   0.0,    0.0,    0.0, 999.0, 0.0,
            0.0,   0.0,    0.0,    0.0, 0.0, 0.2
        ]

        self.odom_pub.publish(odom)

        # TF Publishing
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = odom.header.frame_id
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
