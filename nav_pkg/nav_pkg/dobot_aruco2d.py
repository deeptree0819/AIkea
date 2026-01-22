#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import threading
from nav_pkg.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, \
    alarmAlarmJsonFile
from time import sleep
import numpy as np
import re

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
    dataController, dataServo = alarmAlarmJsonFile() # 컨트롤러 및 서보 알람 코드 읽기
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

class DobotNode(Node):
    def __init__(self):
        super().__init__('dobot_node')

        # Dobot 연결
        self.get_logger().info("Initializing Dobot Node...")
        try:
            self.dashboard, self.move, self.feed = ConnectRobot()
        except Exception as e:
            self.get_logger().error("Failed to connect to robot: {}".format(e))
            rclpy.shutdown()
            return
            
        # 스레드 생성: 피드백 수신
        self.feed_thread = threading.Thread(target=GetFeed, args=(self.feed,))
        self.feed_thread.setDaemon(True)
        self.feed_thread.start()

        # 스레드 생성: 에러 클리어 모니터링
        self.error_thread = threading.Thread(
            target=ClearRobotError, 
            args=(self.dashboard,))
        self.error_thread.setDaemon(True)
        self.error_thread.start()
        self.get_logger().info("Starting enable...")
        self.dashboard.EnableRobot()
        self.get_logger().info("Enable complete :)")

        # ArUco 위치
        self.object_pose = None  # [x(mm), y(mm), z(mm), yaw]
        self.pinky_pose  = None

        # 토픽 구독
        self.create_subscription(Pose2D, '/aruco_object_pose', self.cb_object, 10)
        self.create_subscription(Pose2D, '/aruco_pinky_pose',  self.cb_pinky,  10)
        self.create_subscription(String,  '/pinky_state',       self.cb_pinky_state, 10)

        # 완료 알림 퍼블리셔
        self.done_pub = self.create_publisher(String, '/dobot_done', 10)

        # 안전 고도/픽/플레이스 고도 (mm 단위)
        self.SAFE_Z   =  0.0
        self.PICK_Z   = -130.0

        self.get_logger().info('DobotNode ready, waiting for pinky_state=="at_place"...')
   
        self.point_Home = [282.3, -192.6, 0.0, 0.0]
        ActivateVacuumGripper(self.dashboard, False)
        RunPoint(self.move, self.point_Home)
        WaitArrive(self.point_Home)

    def cb_object(self, msg: Pose2D):
        # 아예 if 절을 없애도 되고, (0,0)인 경우만 skip
        if msg.x == 0 and msg.y == 0:
            return
        self.object_pose = [msg.x, msg.y, self.PICK_Z, msg.theta]
        self.get_logger().info(f'Object pose updated: {self.object_pose}')


    def cb_pinky(self, msg: Pose2D):
        # 카메라가 Pinky의 aruco 검출 시
        if msg.x == 0 and msg.y == 0:
            return
        self.pinky_pose = [msg.x, msg.y, self.PICK_Z, msg.theta]
        self.get_logger().info(f'Pinky pose updated: {self.pinky_pose}')

    def cb_pinky_state(self, msg: String):
        if msg.data != 'at_place':
            return
        # 물체와 Pinky 위치 모두 확보되어야 함
        if not self.object_pose or not self.pinky_pose:
            self.get_logger().warn('object_pose or pinky_pose missing, skipping pick&place')
            return

        self.get_logger().info('Picking object...')
        # 1) Safe above object → pick → safe above
        RunPoint(self.move, [self.object_pose[0], self.object_pose[1], self.SAFE_Z, 0.0])
        WaitArrive([self.object_pose[0], self.object_pose[1], self.SAFE_Z, 0.0])
        RunPoint(self.move, self.object_pose)
        WaitArrive(self.object_pose)
        ActivateVacuumGripper(self.dashboard, True)
        RunPoint(self.move, [self.object_pose[0], self.object_pose[1], self.SAFE_Z, 0.0])
        WaitArrive([self.object_pose[0], self.object_pose[1], self.SAFE_Z, 0.0])

        self.get_logger().info('Placing onto Pinky tray...')
        # 2) Safe above pinky → place → safe above
        RunPoint(self.move, [self.pinky_pose[0], self.pinky_pose[1], self.SAFE_Z, 0.0])
        WaitArrive([self.pinky_pose[0], self.pinky_pose[1], self.SAFE_Z, 0.0])
        RunPoint(self.move, self.pinky_pose)
        WaitArrive(self.pinky_pose)
        ActivateVacuumGripper(self.dashboard, False)
        RunPoint(self.move, [self.pinky_pose[0], self.pinky_pose[1], self.SAFE_Z, 0.0])
        WaitArrive([self.pinky_pose[0], self.pinky_pose[1], self.SAFE_Z, 0.0])

        # 완료 신호
        done = String()
        done.data = 'done'
        self.done_pub.publish(done)
        self.get_logger().info('Pick&Place done, published /dobot_done="done"')

def main(args=None):
    rclpy.init(args=args)
    node = DobotNode()
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

if __name__=='__main__':
    main()
