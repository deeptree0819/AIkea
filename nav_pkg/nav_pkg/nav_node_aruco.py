import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

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

        # 목표 좌표 파라미터 선언 및 기본값 설정
        self.declare_parameter('goal_x', 1.35)
        self.declare_parameter('goal_y', 0.75)
        self.declare_parameter('goal_yaw_deg', 90.0)
        # 목표 전송
        self.send_goal(
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_yaw_deg').value
        )
        
    def send_goal(self, gx: float, gy: float, gyaw_deg: float):
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

        self.get_logger().info(f'Sending goal: x={gx}, y={gy}, yaw={gyaw_deg}°')
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
        self.get_logger().info(f'Navigation result code: {result_msg.error_code}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PinkyNavNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()