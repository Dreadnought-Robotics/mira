#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from custom_msgs.msg import Object2D
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class OpticalFlowPnPNode(Node):

    def __init__(self):
        super().__init__("optical_flow_pnp")

        self.bridge = CvBridge()
        self.prev_gray = None
        self.bbox = None

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(BoundingBox2D, '/bbox', self.bbox_callback, 10)

        self.pub = self.create_publisher(Object2D, "gate_pose", 10)

        try:
            package_share_dir = get_package_share_directory('mira2_perception')
            calib_path = os.path.join(package_share_dir, 'optimized_calib.npz')
            data = np.load(calib_path)
            self.camera_matrix = data["mtx"]
            self.dist_coeffs = data["dist"]
            self.get_logger().info("Calibration loaded successfully")
        except Exception as e:
            self.get_logger().warn(f"Calibration load failed: {e}")
            self.camera_matrix = None
            self.dist_coeffs = None

    def bbox_callback(self, msg):
        self.bbox = msg

    def image_callback(self, msg):
        if self.bbox is None or self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            return

        cx = int(self.bbox.center.position.x)
        cy = int(self.bbox.center.position.y)
        bw = int(self.bbox.size_x)
        bh = int(self.bbox.size_y)

        h_img, w_img = gray.shape
        x1 = max(0, cx - bw // 2)
        x2 = min(w_img, cx + bw // 2)
        y1 = max(0, cy - bh // 2)
        y2 = min(h_img, cy + bh // 2)

        roi_prev = self.prev_gray[y1:y2, x1:x2]
        roi_curr = gray[y1:y2, x1:x2]

        if roi_prev.shape != roi_curr.shape or roi_prev.size == 0:
            self.prev_gray = gray
            return

        flow = cv2.calcOpticalFlowFarneback(
            roi_prev, roi_curr,
            None, 0.5, 3, 15, 3, 5, 1.2, 0
        )

        mag, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
        mask = (mag > 1.0).astype(np.uint8) * 255

        points = cv2.goodFeaturesToTrack(mask, 50, 0.01, 5)

        if points is None or len(points) < 4:
            self.prev_gray = gray
            return

        image_points = points[:4].reshape(-1, 2).astype(np.float32)

        half_w = (bw / w_img) * 0.5
        half_h = (bh / h_img) * 0.5

        object_points = np.array([
            [-half_w, -half_h, 0],
            [ half_w, -half_h, 0],
            [ half_w,  half_h, 0],
            [-half_w,  half_h, 0]
        ], dtype=np.float32)

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs
            )

            if success:
                msg_out = Object2D()
                msg_out.point.x = float(tvec[2])
                msg_out.point.y = float(tvec[0])
                msg_out.point.z = float(tvec[1])
                self.pub.publish(msg_out)

        except Exception as e:
            self.get_logger().warn(f"PnP solve failed: {e}")

        self.prev_gray = gray


def main(args=None):
    rclpy.init(args=args)
    node = OpticalFlowPnPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
