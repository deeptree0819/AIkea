import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D

class Aruco2DPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_2d_pose_publisher')
        # 물체/피키용 마커 ID 리스트
        self.object_marker_ids = [1, 2, 3]
        self.pinky_marker_ids  = [4, 5]

        # 퍼블리셔
        self.pub_obj   = self.create_publisher(Pose2D, 'aruco_object_pose', 10)
        self.pub_pinky = self.create_publisher(Pose2D, 'aruco_pinky_pose',  10)

        # 카메라
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            rclpy.shutdown()
            return

        # ArUco 세팅
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.params     = cv2.aruco.DetectorParameters_create()

        # 픽셀–월드 대응점
        self.aruco_pts = np.array([
            [353.0, 180.8],[309.0,177.8],[311.0,224.0],
            [353.0,223.2],[357.0,270.2],[398.5,266.0],
            [413.2,309.0],[471.8,327.0],[419.0,365.2],
            [434.5,271.5],
        ], dtype=np.float32)
        self.robot_pts = np.array([
            [64.874, -215.17],[66.97, -288.68],[141.575, -285.94],
            [136.391, -213.575],[218.352, -208.141],[210.326, -138.935],
            [286.175, -112.696],[317.469, -12.0004],[385.657, -98.8440],
            [219.106, -79.9620],
        ], dtype=np.float32)

        H, _ = cv2.findHomography(self.aruco_pts, self.robot_pts, cv2.RANSAC)
        if H is None:
            self.get_logger().error('Failed to compute homography')
            rclpy.shutdown()
            return
        self.homography = H

        self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Aruco2DPosePublisher initialized')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.params
        )

        # 기본 메시지 (검출 없을 때)
        msg_obj = Pose2D(x=0.0, y=0.0, theta=0.0)
        msg_pn  = Pose2D(x=0.0, y=0.0, theta=0.0)

        if ids is not None:
            for idx, corner in zip(ids.flatten(), corners):
                # 마커 중심 픽셀
                pts = corner.reshape(4, 2)
                cx, cy = pts.mean(axis=0)

                # 호모그래피로 월드 좌표 계산
                pixel = np.array([[[cx, cy]]], dtype=np.float32)
                world = cv2.perspectiveTransform(pixel, self.homography)
                wx, wy = world[0, 0]

                # ID에 따라 메시지 업데이트
                if idx in self.object_marker_ids:
                    msg_obj.x     = float(wx)
                    msg_obj.y     = float(wy)
                    msg_obj.theta = float(idx)
                    self.get_logger().info(f'[OBJ] id={idx} → x={wx:.2f}, y={wy:.2f}')
                elif idx in self.pinky_marker_ids:
                    msg_pn.x      = float(wx)
                    msg_pn.y      = float(wy)
                    msg_pn.theta  = float(idx)
                    self.get_logger().info(f'[PNKY] id={idx} → x={wx:.2f}, y={wy:.2f}')

                # 시각화
                cv2.aruco.drawDetectedMarkers(frame, [corner], np.array([idx]))
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 255, 0), -1)

        # 퍼블리시
        self.pub_obj.publish(msg_obj)
        self.pub_pinky.publish(msg_pn)

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
