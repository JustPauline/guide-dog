#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path
from transforms3d.euler import euler2quat, quat2euler
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
import math
from yahboom_msgs.msg import PerfStamp


class VoicePathFollower(Node):
    def __init__(self):
        super().__init__('voice_path_follower')

        # Declare parameters
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('angular_tolerance', 0.1)
        self.declare_parameter('step_distance', 3.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')

        # Read parameters
        self.lookahead    = self.get_parameter('lookahead_distance').value
        self.lin_gain     = self.get_parameter('linear_gain').value
        self.ang_gain     = self.get_parameter('angular_gain').value
        self.goal_tol     = self.get_parameter('goal_tolerance').value
        self.ang_tol      = self.get_parameter('angular_tolerance').value
        self.step_dist    = self.get_parameter('step_distance').value
        self.rate         = self.get_parameter('control_rate').value
        self.robot_frame  = self.get_parameter('robot_frame').value
        self.global_frame = self.get_parameter('global_frame').value

        # TF setup
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers & Subscribers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.cmd_sub  = self.create_subscription(String, 'spokencommand', self.command_callback, 10)

        # Internal state
        self.path       = []
        self.mode       = 'IDLE'  # IDLE, ROTATE, FOLLOW, STOP
        self.target_yaw = None

        # Service client for find_chair (Trigger)
        self.cli = self.create_client(Trigger, 'find_chair')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /find_chair Trigger service...')

        # Control loop
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.get_logger().info('VoicePathFollower ready')

    def command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        cmd_start = self.get_clock().now().to_msg()

        # Rotate left/right
        if cmd in ('left', 'right'):
            yaw = self.get_current_yaw()
            if yaw is None:
                return
            delta = math.pi/2 if cmd == 'left' else -math.pi/2
            self.target_yaw = math.atan2(math.sin(yaw + delta), math.cos(yaw + delta))
            self.mode = 'ROTATE'
            self.get_logger().info(f'Rotate {cmd.upper()} to {math.degrees(self.target_yaw):.1f}°')
            return

        # Step forward/backward/go
        if cmd in ('forward', 'backward', 'go'):
            yaw = self.get_current_yaw()
            if yaw is None:
                return
            dist = self.step_dist if cmd != 'backward' else -self.step_dist
            dx = dist * math.cos(yaw)
            dy = dist * math.sin(yaw)
            goal = PoseStamped()
            goal.header.frame_id = self.global_frame
            goal.header.stamp    = self.get_clock().now().to_msg()
            goal.pose.position.x = self.get_robot_x() + dx
            goal.pose.position.y = self.get_robot_y() + dy
            w, x, y, z = euler2quat(0, 0, yaw)
            goal.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
            goal_sent = self.get_clock().now().to_msg()
            self.goal_pub.publish(goal)
            self.mode = 'FOLLOW'
            self.get_logger().info(f'{cmd.upper()} → goal at ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
            return

        # Find chair command
        if cmd == 'find chair':
            cmd_start = self.get_clock().now().to_msg()
            self.get_logger().info('Calling /find_chair Trigger service…')
            start_srv = self.get_clock().now().to_msg()
            req = Trigger.Request()
            future = self.cli.call_async(req)
            future.add_done_callback(self.on_chair_triggered)
            return

        # Stop command
        if cmd == 'stop':
            self.mode = 'STOP'
            self.path = []
            self.cmd_pub.publish(Twist())
            self.get_logger().info('STOP command received')
            return

    def on_chair_triggered(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'Service call failed: {e}')
            return
        if not res.success:
            self.get_logger().warn(f'Chair detection failed: {res.message}')
            return
        # On success, switch to follow mode; /goal_pose already published
        self.mode = 'FOLLOW'
        self.get_logger().info(f'Chair found: {res.message}')
        end_srv = self.get_clock().now().to_msg()

    def path_callback(self, msg: Path):
        self.path = msg.poses
        if self.mode == 'FOLLOW':
            self.get_logger().info(f'Received path with {len(self.path)} points')
            path_rcv = self.get_clock().now().to_msg()

    def get_current_yaw(self):
        ps = self.get_robot_pose()
        if not ps:
            return None
        q = ps.pose.orientation
        return quat2euler([q.w, q.x, q.y, q.z])[2]

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, Time())
            ps = PoseStamped()
            ps.header = trans.header
            ps.pose.position.x = trans.transform.translation.x
            ps.pose.position.y = trans.transform.translation.y
            ps.pose.orientation = trans.transform.rotation
            return ps
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF error: {e}')
            return None

    def get_robot_x(self):
        ps = self.get_robot_pose()
        return ps.pose.position.x if ps else 0.0

    def get_robot_y(self):
        ps = self.get_robot_pose()
        return ps.pose.position.y if ps else 0.0

    def control_loop(self):
        # ROTATE mode
        if self.mode == 'ROTATE':
            yaw = self.get_current_yaw()
            if yaw is None:
                return
            err = math.atan2(math.sin(self.target_yaw - yaw), math.cos(self.target_yaw - yaw))
            if abs(err) < self.ang_tol:
                self.cmd_pub.publish(Twist())
                self.mode = 'IDLE'
                self.get_logger().info('Rotation complete')
            else:
                t = Twist(); t.angular.z = self.ang_gain * err
                self.cmd_pub.publish(t)
            return

        # STOP mode
        if self.mode == 'STOP':
            self.cmd_pub.publish(Twist())
            return

        # FOLLOW mode: pure pursuit
        if self.mode == 'FOLLOW' and self.path:
            ps = self.get_robot_pose()
            if not ps:
                return
            rx, ry = ps.pose.position.x, ps.pose.position.y
            ryaw    = self.get_current_yaw()
            # Find nearest point index
            min_d, idx = float('inf'), 0
            for i, p in enumerate(self.path):
                dx = rx - p.pose.position.x
                dy = ry - p.pose.position.y
                d = math.hypot(dx, dy)
                if d < min_d:
                    min_d, idx = d, i
            # Lookahead point
            goal_p = self.path[-1].pose
            for p in self.path[idx:]:
                dx = p.pose.position.x - rx
                dy = p.pose.position.y - ry
                if math.hypot(dx, dy) > self.lookahead:
                    goal_p = p.pose
                    break
            # Compute control
            dx = goal_p.position.x - rx
            dy = goal_p.position.y - ry
            dist     = math.hypot(dx, dy)
            angle_to = math.atan2(dy, dx)
            err_ang  = math.atan2(math.sin(angle_to - ryaw), math.cos(angle_to - ryaw))
            if dist < self.goal_tol:
                self.cmd_pub.publish(Twist())
                self.mode = 'IDLE'
                self.get_logger().info('Reached goal')
                return
            t = Twist()
            t.linear.x  = self.lin_gain * dist
            t.angular.z = self.ang_gain * err_ang
            self.cmd_pub.publish(t)

    def destroy_node(self):
        super().destroy_node()


def main():
    rclpy.init()
    node = VoicePathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()