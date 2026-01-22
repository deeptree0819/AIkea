#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class ArucoPose2DCalibNode(Node):
    def __init__(self):
        super().__init__('aruco_pose2d_calib_node')

        # 1) 캘리브레이션 파일 로드
        calib_file = os.path.join(
            os.path.dirname(__file__), 'camera_calib.yaml'
        )
        with open(calib_file, 'r') as f:
            calib = yaml.safe_load(f)
        self.K = np.array(calib['camera_matrix']['data']).reshape(3,3)
        self.D = np.array(calib['distortion_coefficients']['data'])

        # 2) 퍼블리셔
        self.pub = self.create_publisher(Pose2D, 'aruco_pose2d', 10)

        # 3) 카메라 열기
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 오픈 실패')
            rclpy.shutdown()
            return

        # 4) ArUco 딕셔너리 & 파라미터
        self.aruco_dict  = cv2.aruco.getPredefinedDictionary(
                              cv2.aruco.DICT_6X6_250)
        self.params      = cv2.aruco.DetectorParameters_create()

        # 5) 호모그래피 대응점 (픽셀 ↔ mm)
        self.aruco_pts = np.array([
            [513.5, 144.2],
            [506.2,  57.5],
            [593.8,174.5],
            [357.2,120.8],
            [388.0,231.8],
            [458.5,184.0],
            [432.2,88.2],
        ], dtype=np.float32)
        self.robot_pts = np.array([
            [  71.1, -321.983],
            [ 73.3270, -270.145],
            [ -113.366, -344.383 ],
            [  -19.5811,   -300.838],
            [  -33.0945,   -361.922],
            [  43.3170,   -346.71],
            [  37.4990,   -287.343],
        ], dtype=np.float32)

        # 6) H 행렬 계산 (RANSAC)
        self.H, _ = cv2.findHomography(
            self.aruco_pts, self.robot_pts,
            method=cv2.RANSAC, ransacReprojThreshold=5.0
        )
        self.get_logger().info(f'Homography:\n{self.H}')

        # 7) 타이머
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('ArucoPose2DCalibNode 초기화 완료')

    def warp_to_robot(self, x, y):
        src = np.array([[[x, y]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, self.H)
        u, v = dst[0,0]
        return float(u), float(v)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패')
            return

        # ─── 왜곡 보정 ───────────────────────────────────
        undist = cv2.undistort(frame, self.K, self.D, None, self.K)
        gray   = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        # ─── ArUco 검출 ───────────────────────────────────
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params
        )

        msg = Pose2D()
        if ids is None or len(ids)==0:
            msg.x = -1.0; msg.y = -1.0; msg.theta = 0.0
        else:
            # 픽셀 중심 계산
            pts    = corners[0].reshape(4,2)
            center = pts.mean(axis=0)
            cx, cy = float(center[0]), float(center[1])

            # 호모그래피 보정
            rx, ry = self.warp_to_robot(cx, cy)
            msg.x = rx; msg.y = ry; msg.theta = 0.0

            # 디버그 그리기
            cv2.circle(undist, (int(cx),int(cy)), 5, (0,255,0), -1)
            cv2.putText(undist, f"R:({rx:.1f},{ry:.1f})",
                        (10,30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255,0,0), 2)
            self.get_logger().info(
                f'Pixel ({cx:.1f},{cy:.1f}) → Robot ({rx:.1f},{ry:.1f})'
            )

        # 퍼블리시 & 윈도우 출력
        self.pub.publish(msg)
        cv2.imshow('Aruco Homography Calibrated', undist)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPose2DCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
