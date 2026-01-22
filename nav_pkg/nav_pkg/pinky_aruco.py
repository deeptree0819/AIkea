#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

class PinkyNavNode(Node):
    def __init__(self):
        super().__init__('pinky_nav_node')

        # --------------------------------------------------
        # 1) NavigateToPose 액션 클라이언트 생성 및 서버 대기
        # --------------------------------------------------
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for NavigateToPose action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available, shutting down.')
            rclpy.shutdown()
            return
        self.get_logger().info('NavigateToPose action server is available!')

        # --------------------------------------------------
        # 2) /pinky_goal(Int32) 토픽 구독
        # --------------------------------------------------
        self.create_subscription(
            Int32,
            'pinky_goal',
            self.goal_callback,
            10
        )

        # --------------------------------------------------
        # 3) ID → (x, y, yaw_deg) 매핑표
        # --------------------------------------------------
        self.goal_map = {
            1: (1.5, 0.0, 0.0),
            2: (1.5, -0.3, 180.0),
            3: (1.5, -0.6, 180.0)
        }

        self.HOME_POSE = (0.0, 0.0, 0.0)         # 홈 좌표 (x, y, yaw_deg)
        self.returning = False                  # 현재 복귀 중인지 여부
        self.return_timer = None                # 복귀 지연용 타이머 핸들
        self.return_delay_sec = 5.0             # 목적지 도착 후 대기할 시간(초)

        self.get_logger().info('PinkyNavNode ready. Waiting for /pinky_goal messages.')

    def goal_callback(self, msg: Int32):
        """
        /pinky_goal(Int32) 콜백. msg.data가 1,2,3 중 하나인 경우
        해당 좌표를 액션 서버로 전송합니다. 이때 returning 상태는 False로 설정합니다.
        """
        marker_id = msg.data
        if marker_id not in self.goal_map:
            self.get_logger().warn(f'유효하지 않은 pinky_goal ID: {marker_id}. 무시합니다.')
            return

        # 새로운 목표 지점으로 갈 때는 returning=False로 초기화
        # (이전 타이머가 있다면 취소)
        self.returning = False
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.return_timer = None

        x, y, yaw_deg = self.goal_map[marker_id]
        self.get_logger().info(f'pinky_goal ID={marker_id} 수신 → (x={x}, y={y}, yaw={yaw_deg}°)로 이동합니다.')
        self.send_goal(x, y, yaw_deg)

    def send_goal(self, gx: float, gy: float, gyaw_deg: float):
        """
        실제로 NavigateToPose 액션을 호출하여 (gx, gy, gyaw_deg) 목표를 전송합니다.
        """
        gyaw = math.radians(gyaw_deg)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        # quaternion 생성 (roll=0, pitch=0, yaw=gyaw)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, gyaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # 로그에 'returning' 상태도 함께 표시
        if not self.returning:
            self.get_logger().info(f'NavigateToPose 목표 전송 → x={gx}, y={gy}, yaw={gyaw_deg}°')
        else:
            self.get_logger().info(f'RETURN HOME 목표 전송 → x={gx}, y={gy}, yaw={gyaw_deg}°')

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        액션 피드백 수신 시 호출. 남은 거리(distance_remaining)를 로그로 출력합니다.
        """
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        self.get_logger().info(f'남은 거리: {dist:.2f} m')

    def goal_response_callback(self, future):
        """
        액션 GoalResponse 반환 시 호출. goal이 수락되었으면 결과 대기.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 액션 서버에 의해 거절되었습니다.')
            return

        self.get_logger().info('Goal이 수락되었습니다. 결과를 기다립니다...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        액션 결과 수신 시 호출.
        - returning=False: 목적지에 도착한 상태 → 일정 시간 대기 후 홈으로 복귀하도록 타이머 설정
        - returning=True: 이미 복귀 중이었고, 홈에 도착했으므로 종료 로그만 출력
        """
        result_msg = future.result().result
        self.get_logger().info(f'Navigation 결과 코드: {result_msg.error_code}')

        if not self.returning:
            # 목적지에 처음 도착했으므로, delay 후 홈으로 복귀
            self.get_logger().info(f'목적지에 도착했습니다. {self.return_delay_sec:.0f}초 뒤에 홈으로 복귀합니다.')

            self.returning = True
            # 한 번만 실행되는 타이머 생성
            self.return_timer = self.create_timer(
                self.return_delay_sec,
                self.return_home_callback
            )
        else:
            # 이미 돌아오는 중이었고, 홈에 도착한 경우
            self.get_logger().info('홈에 성공적으로 도착했습니다. 네비게이션을 종료합니다.')
            self.returning = False
            # 타이머가 남아있으면 취소
            if self.return_timer is not None:
                self.return_timer.cancel()
                self.return_timer = None

    def return_home_callback(self):
        """
        delay 후 호출되어 홈 좌표로 send_goal 을 실행합니다.
        그리고 타이머를 취소해 반복 실행되지 않도록 합니다.
        """
        # 타이머는 한 번만 실행되도록 즉시 취소
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.return_timer = None

        x_home, y_home, yaw_home = self.HOME_POSE
        self.get_logger().info('이제 홈으로 복귀를 시작합니다.')
        self.send_goal(x_home, y_home, yaw_home)

    def destroy_node(self):
        """
        노드 종료 시 리소스 정리(필요 시).
        """
        # 혹시 남아있는 타이머가 있으면 취소
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.return_timer = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PinkyNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt 수신됨, 종료합니다...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
