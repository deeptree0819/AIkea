#!/usr/bin/env python3
# pinky_node.py

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32, String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose2D
from tf_transformations import quaternion_from_euler

class PinkyNavNode(Node):
    def __init__(self):
        super().__init__('pinky_node')

        # 1) NavigateToPose 액션
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self.nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server unavailable')
            rclpy.shutdown()
            return

        # 2) 토픽 구독·퍼블리시
        self.create_subscription(Pose2D, '/aruco_object_pose', self.cb_obj_pose, 10)
        self.create_subscription(String, '/dobot_done',       self.cb_dobot, 10)
        self.state_pub = self.create_publisher(String, '/pinky_state', 10)

        # 3) 주요 포인트 정의 (map frame 기준, m 단위)
        self.DEST1_POSE = (1.0, 0.0,   0.0)
        self.PLACE_POSE = (0.0, 0.0, 180.0)
        self.GOAL_MAP   = {
            1: (1.5, 0.0, 0.0),
            2: (1.5, -0.3, 180.0),
            3: (1.5, -0.6, 180.0)
        }

        # 상태 관리
        self.phase      = 'to_dest1'
        self.current_id = None

        # 시작 → DEST1으로 이동
        self.send_nav(*self.DEST1_POSE)

    def send_nav(self, x, y, yaw_deg):
        """NavigateToPose 액션 호출"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, math.radians(yaw_deg))
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = q

        goal = NavigateToPose.Goal(pose=pose)
        fut = self.nav.send_goal_async(goal, feedback_callback=self.fb_cb)
        fut.add_done_callback(self.resp_cb)

    def fb_cb(self, fb):
        self.get_logger().info(f'[NAV] remain {fb.feedback.distance_remaining:.2f} m')

    def resp_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('Nav rejected')
            return
        gh.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, fut):
        code = fut.result().result.error_code
        self.get_logger().info(f'[NAV] done code={code}')

        if self.phase == 'to_dest1':
            self.phase = 'wait_object'
            self.get_logger().info('DEST1 도착, 물체 감지 대기 중...')
        elif self.phase == 'to_place':
            # Place 지점 도착 → dobot 호출
            self.phase = 'wait_dobot'
            msg = String(); msg.data = 'at_place'
            self.state_pub.publish(msg)
            self.get_logger().info('/pinky_state="at_place" published')
        elif self.phase == 'to_target':
            self.get_logger().info(f'ID={self.current_id} 목표 지점 도착 완료.')

    def cb_obj_pose(self, msg: Pose2D):
        # 검출 없을 땐 x,y에 -1.0을 퍼블리시하니 무시
    # wait_object 상태가 아닐 때
        if self.phase != 'wait_object':
            return
        # x 또는 y 가 -1.0 이면 “검출 없음”으로 간주
        if msg.x == -1.0 or msg.y == -1.0:
            return
        # Pose2D.theta 에 저장된 ID를 사용
        self.current_id = int(msg.theta)
        self.phase = 'to_place'
        self.get_logger().info(
            f'Object Pose 감지(ID={self.current_id}) → PLACE_POSE로 이동'
        )
        self.send_nav(*self.PLACE_POSE)

    def cb_dobot(self, msg: String):
        if msg.data != 'done' or self.phase != 'wait_dobot':
            return
        self.phase = 'to_target'
        goal = self.GOAL_MAP.get(self.current_id)
        if not goal:
            self.get_logger().error(f'Unknown ID={self.current_id}')
            return
        self.get_logger().info(f'dobot_done 받음 → ID={self.current_id} 위치로 이동')
        self.send_nav(*goal)

def main():
    rclpy.init()
    node = PinkyNavNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()
