#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class Aruco2DPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_2d_pose_publisher')
        # Publisher for 2D pose (world coordinates)
        self.pub = self.create_publisher(Pose2D, 'aruco_pose2d', 10)

        # OpenCV video capture (adjust camera index as needed)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            rclpy.shutdown()
            return

        # ArUco dictionary and detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.params = cv2.aruco.DetectorParameters_create()

        # Known correspondences: pixel points (Aruco) and robot/world points
        self.aruco_pts = np.array([[414.8,285.8],[412.8,232.0],[356.5,207.8],[371.0,222.2],
                           [334.0,217.5],[367.0,257.2],[333.2,237.2],[312.0,222.0],
                           [305.0,271.0],[266.2,219.5],[278.0,263.0],[258.5,234.5],
                           [234.5,216.0],[252.2,265.5],[196.2,240.0],[205.2,285.2],
                           [160.5,290.8],[122.0,333.0],[203.2,312.8],[113.8,371.0],
                           [456.5,337.5],[435.5,280.0],[505.5,293.2],[483.0,373.0]], np.float32)
        self.robot_pts = np.array([[210.25,-164.9],[302.0,-176.4],[354.58,-86.32],[327.15,-107.33],
                           [342.64,-47.6],[271.25,-99.77],[298.5,-44.5],[339.13,-9.18],
                           [256.1,9.75],[358.7,45.52],[277.6,55.15],[325.9,84.83],
                           [361.5,121.66],[274.6,99.01],[332.2,189.1],[250.4,181.0],
                           [248.153,259.4],[186.5,327.35],[185.8,187.654],[122.1,344.9],
                           [120.0,-225.5],[220.8,-204.82],[185.8,-315.9],[60.28,-260.13]], np.float32)
        
        # Compute homography matrix
        H, mask = cv2.findHomography(self.aruco_pts, self.robot_pts, cv2.RANSAC)
        if H is None:
            self.get_logger().error('Failed to compute homography')
            rclpy.shutdown()
            return
        self.homography = H
        self.get_logger().info('Computed homography from correspondences')

        # Initialize storage for last frame and marker data
        self.last_frame = None
        self.last_corners = None
        self.last_ids = None

        # Timers: publish every 5s, show every 0.2s
        self.publish_timer = self.create_timer(5.0, self.publish_callback)
        self.show_timer = self.create_timer(0.2, self.show_callback)
        self.get_logger().info('Aruco2DPosePublisher initialized')

    def publish_callback(self):
        # Capture frame and detect marker, then publish Pose2D
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return
        self.last_frame = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.params)
        self.last_corners = corners
        self.last_ids = ids

        msg = Pose2D()
        if ids is None or len(ids) == 0:
            msg.x, msg.y, msg.theta = -1.0, -1.0, 0.0
        else:
            pts = corners[0].reshape(4, 2)
            center = pts.mean(axis=0)
            cx, cy = float(center[0]), float(center[1])

            # Log only pixel coordinates
            self.get_logger().info(f'Marker pixel center: x={cx:.1f}, y={cy:.1f}')

            pixel_point = np.array([[[cx, cy]]], dtype=np.float32)
            world_point = cv2.perspectiveTransform(pixel_point, self.homography)
            wx, wy = float(world_point[0,0,0]), float(world_point[0,0,1])

            msg.x, msg.y, msg.theta = wx, wy, 0.0

        self.pub.publish(msg)

    def show_callback(self):
        # Display the latest frame with image center and ArUco markers
        if self.last_frame is None:
            return
        frame = self.last_frame.copy()
        h, w = frame.shape[:2]
        img_center = (w // 2, h // 2)

        # Draw markers if detected
        if self.last_ids is not None and len(self.last_ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, self.last_corners, self.last_ids)
            # Draw first marker center and label
            pts = self.last_corners[0].reshape(4, 2)
            marker_center = (int(pts[:,0].mean()), int(pts[:,1].mean()))
            cv2.circle(frame, marker_center, 6, (0, 255, 0), -1)
            cv2.putText(frame,
                        f'Marker Center:{marker_center}',
                        (marker_center[0] + 10, marker_center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Draw image center
        cv2.circle(frame, img_center, 6, (255, 0, 0), -1)
        cv2.putText(frame,
                    f'Image Center:{img_center}',
                    (img_center[0] + 10, img_center[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow('Aruco 2D Pose', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Aruco2DPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()