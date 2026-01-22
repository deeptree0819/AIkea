import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time

class PinkyNavNode(Node):
    def __init__(self):
        super().__init__('pinky_nav_node')
        # ActionClient 초기화 및 서버 대기
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available, shutting down.')
            rclpy.shutdown()
            return
        self.get_logger().info('Action server available!')

        self.declare_parameter('second_goal', 1)
        second_goal = self.get_parameter('second_goal').value
        if second_goal not in (2, 3):
            self.get_logger().error(f'유효하지 않은 second_goal 값: {second_goal} (2 또는 3 이어야 합니다)')
            rclpy.shutdown()
            return
        all_goals = [
            (0.245, 0.85, 90.0),
            (0.245, 0.7, 0.0),
            (0.765, -1.0, 90.0),
            (-0.21, -0.444, 45.0),
            (0.08, 0.04, 90.0),
            (0.0, 0.0, 0.0)
        ]
        if second_goal == 2:
            self.goals = [all_goals[0], all_goals[1], all_goals[2],all_goals[4],all_goals[5]]
        elif second_goal == 3: 
            self.goals = [all_goals[0], all_goals[1], all_goals[3],all_goals[4],all_goals[5]]
        else:
            self.goals = all_goals

        self.ct = 0
        self.current_goal_index = 0
        # 첫 번째 코스 전송 시작
        self.send_next_goal()
    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('모든 코스 주행 완료. 노드 종료.')
            rclpy.shutdown()
            return
        gx, gy, gyaw_deg = self.goals[self.current_goal_index]
        self.get_logger().info(f'[{self.current_goal_index+1}/{len(self.goals)}] Sending goal: x={gx}, y={gy}, yaw={gyaw_deg}°')
        self._send_goal(gx, gy, gyaw_deg)
    def _send_goal(self, gx: float, gy: float, gyaw_deg: float):
        # Quaternion으로 변환
        gyaw = math.radians(gyaw_deg)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        qx, qy, qz, qw = quaternion_from_euler(0, 0, gyaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Remaining distance: {dist:.2f} m')
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
            result_msg = future.result().result
            code = result_msg.error_code
            if code == 0:
                if self.second_goal==2 or self.second_goal==3:
                    if self.ct ==0 or self.ct ==3:
                        self.get_logger().info(f'코스 {self.current_goal_index+1} 완료 (error_code={code}). 다음 코스로 이동하기 전에 2초 대기합니다.')
                        self.current_goal_index += 1
                        self.send_next_goal()
                    else :
                        self.get_logger().info(f'코스 {self.current_goal_index+1} 완료 (error_code={code}). 다음 코스로 이동하기 전에 2초 대기합니다.')
                        time.sleep(2)  # ← 딜레이 추가
                        self.current_goal_index += 1
                        self.send_next_goal()
                    self.ct+=1
                else:
                    if self.ct ==0 or self.ct ==4:
                        self.get_logger().info(f'코스 {self.current_goal_index+1} 완료 (error_code={code}). 다음 코스로 이동하기 전에 2초 대기합니다.')
                        self.current_goal_index += 1
                        self.send_next_goal()
                    else :
                        self.get_logger().info(f'코스 {self.current_goal_index+1} 완료 (error_code={code}). 다음 코스로 이동하기 전에 2초 대기합니다.')
                        time.sleep(2)  # ← 딜레이 추가
                        self.current_goal_index += 1
                        self.send_next_goal()
                    self.ct+=1
            else:
                self.get_logger().error(f'코스 {self.current_goal_index+1} 실패 (error_code={code}). 노드 종료.')
                rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    node = PinkyNavNode()
    rclpy.spin(node)
if __name__ == '__main__':
    main()