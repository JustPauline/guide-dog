import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        self._client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self._plan_pub = self.create_publisher(Path, '/plan', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.send_goal, 10)

    def send_goal(self, msg: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Planner action server not available.')
            return

        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = msg
        goal_msg.planner_id = '' 
        goal_msg.use_start = False   

        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by planner.')
            return

        # if accepted, spin off a request for the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        path: Path = result.path
        if not path.poses:
            self.get_logger().warn('Planner returned an empty path.')
        else:
            self.get_logger().info(f'Received plan with {len(path.poses)} poses.')
        self._plan_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
