#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import math
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs


def load_camera_calibration(path: str):
    with open(path, 'r') as f:
        content = f.read().replace('!!opencv-matrix', '')
    data = yaml.safe_load(content)
    cm = np.array(data['camera_matrix']['data']).reshape((3, 3))
    dc = np.array(data['distortion_coefficients']['data'])
    fx = cm[0, 0]; fy = cm[1, 1]
    cx = data['camera_matrix']['data'][2]
    cy = data['camera_matrix']['data'][5]
    width = data['image_width']
    fov = 2 * math.degrees(math.atan(width / (2 * fx)))
    return cm, dc, fx, fy, cx, cy, fov


class ChairFinderService(Node):
    def __init__(self):
        super().__init__('chair_finder_service')
        self.bridge = CvBridge()

        # Load calibration & DNN model
        calib = '/root/yahboomcar_ws/src/chairdetection/astra.yaml'
        (self.camera_matrix, self.dist_coeffs,
         self.fx, self.fy, self.cx, self.cy,
         self.fov_deg) = load_camera_calibration(calib)

        model_path  = '/root/yahboomcar_ws/src/yahboom_visual/models/mobilenet_ssd/MobileNetSSD_deploy.caffemodel'
        config_path = '/root/yahboomcar_ws/src/yahboom_visual/models/mobilenet_ssd/MobileNetSSD_deploy.prototxt'
        self.net = cv2.dnn_DetectionModel(model_path, config_path)
        self.net.setInputSize(300, 300)
        self.net.setInputScale(1.0/127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

        self.chair_id      = 9
        self.conf_thresh   = 0.5
        self.chair_width_m = 0.6  # avg chair width

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)

        #Cache incoming camera frames
        self.latest_image = None
        self.create_subscription(
            Image,
            '/image_raw',
            lambda msg: setattr(self, 'latest_image', msg),
            1
        )

        #Create the Trigger service
        self.srv = self.create_service(Trigger, 'find_chair', self.handle_find_chair)
        self.get_logger().info('Service ready: call /find_chair to detect a chair.')

    def handle_find_chair(self, req, resp):
        # 1) Grab cached frame
        img_msg = self.latest_image
        if img_msg is None:
            resp.success = False
            resp.message = 'No camera frame cached yet'
            return resp

        # 2) Undistort
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        und   = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        # 3) Run detection
        cls, confs, boxes = self.net.detect(und, confThreshold=self.conf_thresh)
        # Flatten and guard against None
        class_ids = np.array(cls).flatten() if cls is not None else np.array([])
        confs_arr = np.array(confs).flatten() if confs is not None else np.array([])

        if class_ids.size == 0:
            resp.success = False
            resp.message = 'Chair class not detected'
            return resp

        for cid, conf, box in zip(class_ids, confs_arr, boxes):
            if cid != self.chair_id:
                continue

            x, y, w, h = box
            u, v       = x + w/2, y + h/2
            depth      = (self.fx * self.chair_width_m) / w

            # 5) Back-project into camera_link
            p_cam = PointStamped()
            p_cam.header.frame_id = 'camera_link'
            p_cam.header.stamp    = self.get_clock().now().to_msg()
            p_cam.point.x = depth
            p_cam.point.y = -(u - self.cx) * depth / self.fx
            p_cam.point.z = -(v - self.cy) * depth / self.fy

            # 6) Transform to map
            try:
                t = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
                p_map = tf2_geometry_msgs.do_transform_point(p_cam, t)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                resp.success = False
                resp.message = f'TF error: {e}'
                return resp

            # 7) Publish PoseStamped goal
            goal = PoseStamped()
            goal.header       = p_map.header
            goal.pose.position = p_map.point
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)

            self.get_logger().info(f'Published goal at x={p_map.point.x:.2f}, y={p_map.point.y:.2f}')

            # 8) Reply success
            resp.success = True
            resp.message = f'Chair at x={p_map.point.x:.2f}, y={p_map.point.y:.2f}'
            return resp

        # If we finish loop without returning:
        resp.success = False
        resp.message = 'No matching chair found'
        return resp


def main():
    rclpy.init()
    node = ChairFinderService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
