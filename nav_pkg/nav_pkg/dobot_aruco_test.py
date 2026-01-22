#!/usr/bin/env python3
import os
import threading
import re
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import numpy as np

from nav_pkg.dobot_api import (
    DobotApiDashboard,
    DobotApi,
    DobotApiMove,
    MyType,
    alarmAlarmJsonFile
)

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
        self.get_logger().info("Initializing Dobot Node...")

        try:
            self.dashboard, self.move, self.feed = ConnectRobot()
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            rclpy.shutdown()
            return

        self.dashboard.EnableRobot()
        threading.Thread(target=GetFeed, args=(self.feed,), daemon=True).start()
        threading.Thread(target=ClearRobotError, args=(self.dashboard,), daemon=True).start()

        # Subscriber for ArUco world pose (meters)
        self.pick_z = -130.0  # in mm
        self.create_subscription(
            Pose2D,
            '/aruco_pose2d',
            self.aruco_callback,
            10
        )

        # Home position
        ActivateVacuumGripper(self.dashboard, False)
        home = [282.3, -192.6, self.pick_z, 0.0]
        self.move.MovL(*home)
        WaitArrive(home)

        self.get_logger().info("DobotNode ready, tracking ArUco...")

    def aruco_callback(self, msg: Pose2D):
        if msg.x == -1.0 and msg.y == -1.0:
            self.get_logger().warn("Marker lost, stopping movement.")
            return
        # Convert meters -> mm
        rx = msg.x
        ry = msg.y
        self.get_logger().info(f"Following ArUco → x={rx:.1f}, y={ry:.1f}")
        self.move.MovL(rx, ry, self.pick_z, 0.0)
        WaitArrive([rx, ry, self.pick_z, 0.0])

    def destroy_node(self):
        try:
            self.dashboard.DisableRobot()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
