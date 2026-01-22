#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from nav_pkg.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, \
	MyType, alarmAlarmJsonFile
from time import sleep
from std_msgs.msg import String
import numpy as np
import re
from std_msgs.msg import Int32


# Global variables (shared among threads)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()


def ConnectRobot():
    try:
        ip = "192.168.1.6"  # 로봇 IP (필요시 수정)
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("Establishing connection...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<Connection successful>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(Connection failed:(")
        raise e


def RunPoint(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])


def GetFeed(feed: DobotApi):
    global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex(feedInfo['test_value'][0]) == '0x123456789abcdef':
            globalLockValue.acquire()
            # 현재 값 갱신
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['isRunQueuedCmd'][0]
            enableStatus_robot = feedInfo['EnableStatus'][0]
            robotErrorState = feedInfo['ErrorStatus'][0]
            globalLockValue.release()
        sleep(0.001)


def WaitArrive(point_list):
    # 목표 좌표와 현재 좌표 간의 오차가 허용범위(여기서는 1)를 넘지 않을 때까지 대기
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if abs(current_actual[index] - point_list[index]) > 1:
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)


def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState, enableStatus_robot, algorithm_queue
    dataController, dataServo = alarmAlarmJsonFile()    
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if numbers and numbers[0] == 0:
                if len(numbers) > 1:
                    for i in numbers[1:]:
                        alarmState = False
                        if i == -2:
                            print("Robot Alarm: Collision detected", i)
                            alarmState = True
                        if alarmState:
                            continue                
                        for item in dataController:
                            if i == item["id"]:
                                print("Robot Alarm: Controller error ID", 
                                i, item["en"]["description"])
                                alarmState = True
                                break 
                        if alarmState:
                            continue
                        for item in dataServo:
                            if i == item["id"]:
                                print("Robot Alarm: Servo error ID", 
                                i, item["en"]["description"])
                                break  
                    prompt = (
                        "Enter 1 to clear errors and "
                        "continue operation: "
                    )
                    choose = input(prompt)     
                    if int(choose) == 1:                
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()
        else:  
            # 에러 상태가 없고, 로봇이 활성화 상태이면 continue 명령 전송
            if int(enableStatus_robot[0]) == 1 and int(algorithm_queue[0]) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)

def ActivateVacuumGripper(dashboard: DobotApiDashboard, activate: bool):

    index = 1  # Assuming DO_01 corresponds to index 1
    status = 1 if activate else 0
    dashboard.DO(index, status)  # Activate or deactivate DO_01 based on status
    print(f"Vacuum Gripper {'activated' if activate else 'deactivated'}")

class DobotArucoNode(Node):
    def __init__(self):
        super().__init__('dobot_aruco_node')
        self.get_logger().info("Initializing dobot_aruco_node...")
        try:
            self.dashboard, self.move, self.feed = ConnectRobot()
        except Exception as e:
            self.get_logger().error("Failed to connect to robot: {}".format(e))
            rclpy.shutdown()
            return

        # 로봇 활성화
        self.get_logger().info("Starting enable...")
        self.dashboard.EnableRobot()
        self.get_logger().info("Enable complete :)")

        # 스레드 생성: 피드백 수신
        self.feed_thread = threading.Thread(target=GetFeed, args=(self.feed,))
        self.feed_thread.setDaemon(True)
        self.feed_thread.start()

        # 스레드 생성: 에러 클리어 모니터링
        self.error_thread = threading.Thread(target=ClearRobotError, 
        args=(self.dashboard,))
        self.error_thread.setDaemon(True)
        self.error_thread.start()

        # 처리 중 플래그
        self.processing = False
        self.last_detected = -1   

        # 이동할 좌표 설정 (예시)
        self.PICK_POINT  = [282.3, -192.6, -140.0, 0.0]
        self.PLACE_POINT = [280.0, 7.75, -60.0, 0.0]
        # 안전 고도
        self.SAFE_PICK_POINT = [282.3, -192.6, 0.0, 0.0]
        self.SAFE_PLACE_POINT  = [280.0, 7.75, 0.0, 0.0]

        RunPoint(self.move, self.SAFE_PICK_POINT)
        WaitArrive(self.SAFE_PICK_POINT)
        ActivateVacuumGripper(self.dashboard, False)

        self.goal_publisher = self.create_publisher(Int32, 'pinky_goal', 10)

        # /aruco_ids 구독
        self.create_subscription(
            Int32,
            'aruco_ids',
            self.aruco_callback,
            10
        )

    def aruco_callback(self, msg: Int32):
        """
        /aruco_id 토픽(Int32)을 구독하는 콜백.
        msg.data가 -1이면 “마커 없음”으로 간주, 그 외엔 해당 ID를 처리.
        """
        # 0) 이미 픽앤플레이스 중이면 무시
        if self.processing:
            return

        # 1) payload (단일 정수) 가져오기
        marker_id = msg.data

        # 2) “-1”이면 마커가 없는 상태
        if marker_id == -1:
            if self.last_detected != -1:
                self.get_logger().info("Aruco marker lost → resetting last_detected.")
                self.last_detected = -1
            return

        # 3) marker_id가 이전과 동일하다면 무시
        if marker_id == self.last_detected:
            return

        # 4) 새로운 ID가 감지된 경우에만 픽앤플레이스 실행
        self.get_logger().info(f"Received new ArUco ID: {marker_id} → Starting pick-and-place")
        self.processing = True

        self.process_fixed_pick_place()

        pinky_goal = Int32()
        pinky_goal.data = marker_id
        self.goal_publisher.publish(pinky_goal)
        self.get_logger().info(f"Published Pinky goal: {marker_id}")

        # 완료 후 last_detected 갱신
        self.last_detected = marker_id
        self.get_logger().info("Pick-and-place completed. Waiting for marker to disappear.")
        self.processing = False


    def process_fixed_pick_place(self):

        # 1) SAFE_PICK_POINT로 이동
        RunPoint(self.move, self.SAFE_PICK_POINT)
        WaitArrive(self.SAFE_PICK_POINT)

        # 2) PICK 위치로 이동 & 그리퍼 ON
        RunPoint(self.move, self.PICK_POINT)
        WaitArrive(self.PICK_POINT)
        ActivateVacuumGripper(self.dashboard, True)

        # 3) PICK 후 SAFE_PICK_POINT 복귀
        RunPoint(self.move, self.SAFE_PICK_POINT)
        WaitArrive(self.SAFE_PICK_POINT)

        # 4) SAFE_PLACE_POINT로 이동
        RunPoint(self.move, self.SAFE_PLACE_POINT)
        WaitArrive(self.SAFE_PLACE_POINT)

        # 5) PLACE 위치로 이동 & 그리퍼 OFF
        RunPoint(self.move, self.PLACE_POINT)
        WaitArrive(self.PLACE_POINT)
        ActivateVacuumGripper(self.dashboard, False)

        # 6) PLACE 후 SAFE_PLACE_POINT 복귀
        RunPoint(self.move, self.SAFE_PLACE_POINT)
        WaitArrive(self.SAFE_PLACE_POINT)

    def destroy_node(self):
        # 종료 시 로봇 비활성화
        try:
            self.dashboard.DisableRobot()
        except Exception as e:
            self.get_logger().error(f"Error disabling robot: {e}")
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = DobotArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.get_logger().info("Disabling robot before shutdown...")
        try:
            node.dashboard.DisableRobot()
        except Exception as e:
            node.get_logger().error("Error disabling robot: {}".format(e))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
