import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

# 장소별 좌표 매핑 (필요한 키 추가)
PLACE_COORDS = {
    'restourant': (0.1, 1.0, -90.0),
    'cafe': (1.0, 1.0, -90.0),
    'gundam': (0.8, -0.8, 90.0),
    'golf': (-0.25, -0.5, 135.0),
    # 'room1': (x, y, yaw_deg), ...
}

class PinkyNavClientNode(Node):
    def __init__(self):
        super().__init__('pinky_nav_client')
        # 런타임에 robot_id 입력
        robot_id = input('Enter robot ID: ').strip()
        if not robot_id:
            self.get_logger().error('Robot ID를 입력해야 합니다.')
            rclpy.shutdown()
            return

        # 소켓 연결
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('127.0.0.1', 9999))
            self.get_logger().info('Socket connected to kiosk server')
            self.sock.sendall(robot_id.encode())
        except Exception as e:
            self.get_logger().error(f'Socket 연결 실패: {e}')
            rclpy.shutdown()
            return

        # 수신 쓰레드 시작
        threading.Thread(target=self._socket_listener, daemon=True).start()

        # ROS2 액션 클라이언트 초기화
        self._ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server 미가용, 종료')
            rclpy.shutdown()
            return
        self.get_logger().info('NAV action server 연결 완료')

    def _socket_listener(self):
        while True:
            try:
                raw = self.sock.recv(1024)
                if not raw:
                    self.get_logger().info('Socket 종료됨 by server')
                    break
                place = raw.decode().strip()
                if place in PLACE_COORDS:
                    x, y, yaw = PLACE_COORDS[place]
                else:
                    self.get_logger().warning(f'알 수 없는 장소: {place}')
                    continue
                self.get_logger().info(f'Received place: {place} -> x={x}, y={y}, yaw={yaw}')
                self.send_goal(x, y, yaw)
            except Exception as e:
                self.get_logger().error(f'Socket listener error: {e}')
                break

    def send_goal(self, gx, gy, gyaw_deg):
        rad = math.radians(gyaw_deg)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        qx, qy, qz, qw = quaternion_from_euler(0, 0, rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info('Sending GOAL action')
        fut = self._ac.send_goal_async(goal, feedback_callback=self._feedback_cb)
        fut.add_done_callback(self._response_cb)

    def _feedback_cb(self, fb):
        dist = fb.feedback.distance_remaining
        self.get_logger().info(f'Remaining: {dist:.2f} m')

    def _response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result().result
        self.get_logger().info(f'Finished: code={res.error_code}')
        try:
            self.sock.sendall(f"RESULT {res.error_code}".encode())
        except:
            pass


def main():
    rclpy.init()
    node = PinkyNavClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
