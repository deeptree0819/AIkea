import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

class PinkyNavNode(Node):
    def __init__(self):
        super().__init__('pinky_nav_node')
        self.state_pub = self.create_publisher(String, 'pinky_state', 10)
        self.current_state = 'to_go'

        self.create_subscription(
            String, 'pinky_state', self.state_callback, 10
        )

        # ActionClient 초기화 및 서버 대기
        self.action_client = ActionClient(self, 
        NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
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

        self.publish_state()


    def publish_state(self):
        """현재 상태를 pinky_state 토픽에 발행"""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def state_callback(self, msg: String):
        state = msg.data.strip()
        if state == 'return':
            self.get_logger().info('Received "return" → sending return goal')
            self.current_state = 'returning'
            self.send_goal(0.0, 0.0, 0.0)
            self.publish_state()

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
        send_goal_future = self.action_client.send_goal_async(
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
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Navigation result code: {result}')

        if self.current_state == 'to_go':
            self.current_state = 'reached'
            self.get_logger().info(f'State set to "{self.current_state}"')
            self.publish_state()
        elif self.current_state == 'returning':
            self.current_state = 'returned'
            self.get_logger().info(f'State set to "{self.current_state}"')
            self.publish_state()



def main(args=None):
    rclpy.init(args=args)
    node = PinkyNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()