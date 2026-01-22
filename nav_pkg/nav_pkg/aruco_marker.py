#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2

class ArucoCameraNode(Node):
    def __init__(self):
        super().__init__('aruco_camera_node')
        # 퍼블리셔 생성
        self.pub = self.create_publisher(Int32, 'aruco_ids', 10)

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다.')
            return

        # ArUco 딕셔너리 및 파라미터 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # 타이머: 0.1s(10Hz)마다 콜백
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters)

        msg = Int32()
        if ids is None or len(ids) == 0:
            msg.data = -1
            self.pub.publish(msg)
            self.get_logger().info(f'발행된 ArUco ID: {msg.data}  (마커 없음)')
        else:
            # 첫 번째 마커만 처리하는 예시
            first_id = int(ids.flatten()[0])
            first_corners = corners[0].reshape((4, 2))  # shape (4,2)
            
            # 중앙 좌표 계산
            cx = first_corners[:, 0].mean()
            cy = first_corners[:, 1].mean()
            
            # 로그에 출력
            self.get_logger().info(
                f'발견된 마커 ID={first_id}, 중앙 좌표=({cx:.1f}, {cy:.1f})'
            )
            
            # 메시지 발행(원래 ID만 발행하던 Int32 토픽을 확장하려면 별도 메시지 타입을 사용하세요)
            msg.data = first_id
            self.pub.publish(msg)

            # 화면에 그리기
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            # 중심점 그리기
            cv2.circle(
                frame,
                (int(cx), int(cy)),
                5,
                (0, 255, 0),
                -1
            )

        cv2.imshow('Aruco Monitor', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('종료 키(q) 입력으로 노드 종료')
            rclpy.shutdown()

    def destroy_node(self):
        # 노드 종료 시 카메라와 윈도우 정리
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
