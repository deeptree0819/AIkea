#!/usr/bin/env python3
import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time
# 장소별 좌표 매핑
PLACE_COORDS = {
    '레스토랑':   (1.0, 1.1, -90.0),
    '카페':       (0.0, 1.2, -90.0),
    '건담베이스': (0.8, -1.1,  90.0),
    '키즈카페': (-0.4, -0.55, 45.0),
}

# 파일 맨 위에 정의
ADDITIONAL_WAYPOINTS = {
    '레스토랑': [
        (0.5,  -0.5, 45.0),
        (0.6,  0.90, 60.0),
        (0.6,  1.00, 75.0),
        (0.7,  1.05, 85.0),
        (0.7,  1.10, 90.0),
    ],
    '카페': [
        (0.3,  0.90,  45.0),
        (0.4,  0.95,  30.0),
        (0.3,  1.00,  15.0),
        (0.2,  1.10,   75.0),
        (0.1,  1.20,   90.0),
    ],
    '건담베이스': [
        (0.30, -0.50,  45.0),
        (0.40, -0.60,  60.0),
        (0.40, -0.75,  75.0),
        (0.50, -0.90,  85.0),
        (0.50, -1.00,  90.0),
    ],
    '키즈카페': [
        (-0.20, -0.35, -45.0),
        (-0.30, -0.4, -50.0),
        (-0.40, -0.45, -55.0),
        (-0.40, -0.5, -42.5),
        (-0.50, -0.5, -30.0),
    ],
}


# 로봇별 Front 웨이포인트 설정
FRONT_WAYPOINTS = {
    'pinky_1': [
        (0.3, 0.0, -45.0),
        (0.1, -0.7, -90.0),
        (0.1, -0.65,   0.0),
    ],
    'pinky_2': [
        (0.2, 0.0, 0.0),
        (0.18, 0.85, 90.0),
        (0.18, 0.8,   0.0),
    ],  
}
# 원점 좌표
HOME = (0.0, 0.0, 0.0)
class PinkyNavClientNode(Node):
    def __init__(self):
        self.last_place_key = None   # 마지막 목적지 문자열 저장

        super().__init__('pinky_nav_client')
        # 파라미터 선언 및 읽기
        self.declare_parameter('server_host', '0.0.0.0')
        self.declare_parameter('server_port', 9999)
        host = self.get_parameter('server_host').value
        port = self.get_parameter('server_port').value
        self.get_logger().info(f"Connecting to server at {host}:{port}")
        # Robot ID 입력 및 등록
        robot_id = input('Enter robot ID (pinky_1 or pinky_2): ').strip()
        if robot_id not in FRONT_WAYPOINTS:
            self.get_logger().error('유효한 Robot ID(pinky_1 또는 pinky_2)를 입력해야 합니다.')
            rclpy.shutdown()
            return
        self.robot_id = robot_id
        # 소켓 연결
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.get_logger().info(f'Socket connected to {host}:{port}')
            # 등록 메시지 전송
            self.sock.sendall(f"{robot_id}\n".encode())
        except Exception as e:
            self.get_logger().error(f'Socket 연결 실패: {e}')
            rclpy.shutdown()
            return
        # Action client 초기화
        self._ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server 미가용, 종료')
            rclpy.shutdown()
            return
        self.last_place = None
        threading.Thread(target=self._socket_listener, daemon=True).start()
        self.get_logger().info('PinkyNavClientNode initialized')

    def _socket_listener(self):
        while True:
            raw = self.sock.recv(1024)
            if not raw:
                self.get_logger().info('Socket closed by server')
                break
            msg = raw.decode().strip()
            # 1) 목적지 수신
            if msg in PLACE_COORDS:
                self.last_place_key = msg
                self.get_logger().info(f"→ FRONT waypoints for {msg}")
                for wp in FRONT_WAYPOINTS[self.robot_id]:
                    self.send_goal(*wp)
                    time.sleep(1.0)

                self.get_logger().info('*** FRONT 완료 → 5초 후 ARRIVED 전송 ***')
                time.sleep(5.0)
                self.sock.sendall(b'ARRIVED\n')
                self.get_logger().info('*** ARRIVED sent ***')

            # 2) 서버에서 DONE 받으면 추가 웨이포인트+최종 목적지
            elif msg == 'DONE':
                key = self.last_place_key
                if key is None:
                    self.get_logger().warning('이전 목적지 정보 없음, 무시')
                    continue
                threading.Thread(
                    target=self._return_after_final,
                    args=(key,),
                    daemon=True
                ).start()
    def _return_after_final(self, key: str):
        """ 추가 웨이포인트 → 최종 목적지 도달 대기 → 5초 머무름 → 홈 복귀 """
        # 1) 추가 웨이포인트
        for wp in ADDITIONAL_WAYPOINTS.get(key, []):
            self.get_logger().info(f"→ additional waypoint for {key}: {wp}")
            self.send_goal(*wp)
            time.sleep(1.0)

        # 2) 최종 목적지로 이동 (동기적 대기)
        final = PLACE_COORDS[key]
        self.get_logger().info(f"→ final destination: {final}")
        # goal 생성
        rad = math.radians(final[2])
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = final[0]
        pose.pose.position.y = final[1]
        qx, qy, qz, qw = quaternion_from_euler(0, 0, rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # send + wait for accept
        send_fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()
        if not handle.accepted:
            self.get_logger().error('최종 목적지 Goal 거부됨, 홈 복귀 생략')
            return

        # wait for result
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        self.get_logger().info(f'최종 목적지 도착 (error_code={result.error_code})')

        # 3) 도착 후 5초 머무름
        self.get_logger().info('최종 목적지에 5초간 머뭅니다.')
        time.sleep(5.0)

        # 4) 홈 복귀
        self.get_logger().info('홈 복귀 명령 전송')
        self.send_goal(*HOME)
        self.get_logger().info('홈 복귀 완료')
    def _return_home(self):
        time.sleep(5.0)
        self.get_logger().info('5초 대기 후 원점으로 복귀')
        self.send_goal(*HOME)
    def send_goal(self, x: float, y: float, yaw_deg: float):
        rad = math.radians(yaw_deg)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_euler(0, 0, rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f}, {yaw_deg:.1f}°)")
        future = self._ac.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._response_cb)
    def _feedback_cb(self, feedback_msg):
        self.get_logger().info(f"Remaining distance: {feedback_msg.feedback.distance_remaining:.2f} m")
    def _response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return
        self.get_logger().info('Goal accepted, waiting for result')
        handle.get_result_async().add_done_callback(self._result_cb)
    def _result_cb(self, future):
        res = future.result().result
        self.get_logger().info(f'Navigation completed with code {res.error_code}')
        try:
            self.sock.sendall(f"RESULT {res.error_code}\n".encode())
            self.get_logger().info('Sent RESULT to server')
        except Exception:
            self.get_logger().warning('RESULT 전송 실패')
def main():
    rclpy.init()
    node = PinkyNavClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()