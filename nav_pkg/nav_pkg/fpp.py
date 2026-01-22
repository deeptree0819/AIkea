#!/usr/bin/env python3
import os
import cv2
import numpy as np
import time
import threading
import re
from dobot_api import (
    DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
)
from ultralytics import YOLO

# --- Dobot 연결 및 피드백/오류 관리 ---
def ConnectRobot(ip='192.168.1.6', dash_port=29999, move_port=30003, feed_port=30004):
    dash = DobotApiDashboard(ip, dash_port)
    mv   = DobotApiMove(ip, move_port)
    fd   = DobotApi(ip, feed_port)
    return dash, mv, fd

current_actual     = None
algorithm_queue    = None
enableStatus_robot = None
robotErrorState    = False
state_lock         = threading.Lock()

def GetFeed(feed):
    global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
    while True:
        data, received = bytes(), 0
        while received < 1440:
            chunk = feed.socket_dobot.recv(1440 - received)
            if not chunk: break
            data += chunk; received += len(chunk)
        if not data: continue
        info = np.frombuffer(data, dtype=MyType)
        if hex(info['test_value'][0]) == '0x123456789abcdef':
            with state_lock:
                current_actual     = info['tool_vector_actual'][0]
                algorithm_queue    = info['isRunQueuedCmd'][0]
                enableStatus_robot = info['EnableStatus'][0]
                robotErrorState    = info['ErrorStatus'][0]
        time.sleep(0.001)

def WaitArrive(target):
    """현재 좌표가 target에 도달할 때까지 대기"""
    while True:
        with state_lock:
            if current_actual is not None and all(
                abs(current_actual[i] - target[i]) <= 1 for i in range(len(target))
            ):
                return
        time.sleep(0.001)

def ClearRobotError(dashboard):
    ctrl_data, servo_data = alarmAlarmJsonFile()
    global robotErrorState, algorithm_queue, enableStatus_robot
    while True:
        with state_lock:
            err = robotErrorState
            q   = algorithm_queue
            en  = enableStatus_robot
        if err:
            codes = [int(e) for e in re.findall(r'-?\d+', dashboard.GetErrorID())]
            if codes and codes[0] == 0:
                for c in codes[1:]:
                    desc = next(
                        (item['en']['description'] for item in (ctrl_data+servo_data) if item['id']==c),
                        'Unknown'
                    )
                    print(f"Robot Alarm ID={c}: {desc}")
                if input("Enter 1 to clear and continue: ").strip() == '1':
                    dashboard.ClearError(); time.sleep(0.01); dashboard.Continue()
        else:
            if q and en and int(en[0])==1 and int(q[0])==0:
                dashboard.Continue()
        time.sleep(5)

def RunPoint(move, pt):
    move.MovL(pt[0], pt[1], pt[2], pt[3])

def ActivateVacuumGripper(dashboard, on):
    dashboard.DO(1, 1 if on else 0)
    print(f"Vacuum gripper {'on' if on else 'off'}")

def BlowOffGripper(dashboard, on):
    # DO2 를 블로우 오프 밸브 제어용으로 쓰는 예
    dashboard.DO(2, 1 if on else 0)
    print(f"Blow-off {'ON' if on else 'OFF'}")

# --- 메인: YOLO→저장 → ArUco 인식 시 pick & move 실행 ---
def main():
    dash, mv, fd = ConnectRobot()
    dash.EnableRobot()
    threading.Thread(target=GetFeed, args=(fd,), daemon=True).start()
    threading.Thread(target=ClearRobotError, args=(dash,), daemon=True).start()

    # 카메라, YOLO, ArUco, Homography 초기화
    cap        = cv2.VideoCapture(0)
    model      = YOLO(os.getenv('YOLO_MODEL_PATH','best.pt'))
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    # 2) 베이스 위치로 이동
    base = [294.0, 2.0, 0.0, 0.0]
    print("Moving to base:", base)
    RunPoint(mv, base); WaitArrive(base)

    pts_aruco  = np.array([[510.5,358.0],[485.2,365.5],[444.0,372.0],[415.0,174.0],
                           [501.5,330.0],[474.5,335.0],[450.2,344.2],[488.8,292.2],
                           [467.8,308.0],[448.0,318.2],[476.5,271.5],[459.5,283.5],
                           [441.8,295.8],[457.0,247.0],[442.5,263.2],[432.5,278.8],
                           [393.8,244.2],[371.0,218.0],[365.5,243.2],[333.0,203.0],
                           [329.5,235.5],[296.5,202.5],[297.2,233.2],[265.0,205.2],
                           [268.0,236.5],[227.2,209.8],[235.5,242.5],[193.8,231.8],
                           [205.5,256.5],[157.8,246.2],[175.5,268.8],[123.2,285.2],
                           [152.0,308.2],[107.0,318.2],[134.5,331.5],[166.5,345.5],
                           [93.2,358.8],[126.2,369.0]], np.float32)
    pts_robot  = np.array([[101.0,-342.9],[88.75,-299.16],[79.5,-228.6],[335.5,-170.8],
                           [148.4,-330.66],[140.1,-281.1],[127.5,-241.1],[203.7,-308.2],
                           [184.8,-270.7],[170.9,-239.3],[248.5,-287.7],[227.7,-260.3],
                           [208.8,-227.6],[298.7,-261.6],[265.7,-231.7],[235.8,-210.6],
                           [299.1,-147.9],[340.4,-110.4],[305.9,-106.1],[368.9,-50.8],
                           [315.2,-40.56],[370.17,14.15],[320.4,12.76],[366.4,62.27],
                           [312.6,60.36],[357.4,130.25],[304.05,120.1],[332.8,192.25],
                           [286.06,165.9],[301.0,244.8],[260.0,218.75],[236.06,303.5],
                           [199.4,255.8],[185.26,330.9],[160.0,283.9],[136.3,234.0],
                           [122.16,354.1],[95.45,300.1]], np.float32)
    H, _       = cv2.findHomography(pts_aruco, pts_robot)

    cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Detection', 1280, 720)

    # 클래스 이름 맵
    class_names    = {0:"테이블", 1:"쇼파", 2:"의자", 3:"침대"}
    class_pick_z   = {0:-129.0,  1:-128.0,  2:-115.0,  3:-149.0}
    OFF, DROP_Z    = 60, -10
    try:
        # 첫 YOLO 감지로 객체 리스트화
        ret, frame = cap.read()
        if not ret:
            print("카메라 프레임을 가져올 수 없습니다.")
            return

        dets = model(frame)[0]
        detections = []
        for b in dets.boxes:
            cls = int(b.cls[0])
            x1,y1,x2,y2 = map(int, b.xyxy[0])
            cx, cy = (x1+x2)//2, (y1+y2)//2
            uv = np.array([cx, cy, 1.0])
            w  = H.dot(uv); w /= w[2]
            wx, wy = float(w[0]), float(w[1])
            if wx <= 380.0:
                detections.append({'class':cls, 'world':(wx,wy)})

        if not detections:
            print("필터 조건을 만족하는 객체가 없습니다.")
            return

        print("초기 감지된 객체:")
        for i, det in enumerate(detections):
            name = class_names.get(det['class'], 'Unknown')
            wx, wy = det['world']
            print(f" [{i}] {name}({det['class']}) at ({wx:.3f},{wy:.3f})")

        # ArUco 대기 + 5초마다 YOLO 갱신
        print("\nArUco가 x ≤ 380 안으로 들어올 때까지 대기하며 5초마다 YOLO 갱신...")
        last_yolo = time.time() - 5
        yolo_dets = dets
        marker_pos = None

        while True:
            now = time.time()
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            # 5초마다 YOLO 갱신 & 결과 프린트
            if now - last_yolo >= 5:
                last_yolo = now
                yolo_dets = model(frame)[0]

                # 필터링된 리스트 재생성
                filtered = []
                for b in yolo_dets.boxes:
                    cls = int(b.cls[0])
                    x1,y1,x2,y2 = map(int, b.xyxy[0])
                    cx, cy = (x1+x2)//2, (y1+y2)//2
                    uv = np.array([cx, cy, 1.0])
                    w  = H.dot(uv); w /= w[2]
                    wx, wy = float(w[0]), float(w[1])
                    if wx <= 380.0:
                        filtered.append({'class':cls, 'world':(wx,wy)})

                # 상세 리스트 출력
                print("\n[YOLO] 감지된 객체:")
                for idx, det in enumerate(filtered):
                    name = class_names.get(det['class'], 'Unknown')
                    wx, wy = det['world']
                    print(f" [{idx}] {name}({det['class']}) at ({wx:.3f},{wy:.3f})")

                # 요약 출력
                counts = {}
                for det in filtered:
                    counts[det['class']] = counts.get(det['class'], 0) + 1
                summary = ', '.join(f"{class_names.get(c,'?')} {n}개" for c, n in counts.items())
                print(f"총 {len(filtered)}개 객체 감지: {summary}")

            # YOLO 박스 오버레이
            for b in yolo_dets.boxes:
                x1,y1,x2,y2 = map(int, b.xyxy[0])
                cls = int(b.cls[0])
                cx, cy = (x1+x2)//2, (y1+y2)//2
                uv = np.array([cx, cy, 1.0])
                w  = H.dot(uv); w /= w[2]
                wx = float(w[0])
                color = (0,255,0) if wx <= 380.0 else (0,0,255)
                cv2.rectangle(frame, (x1,y1),(x2,y2), color, 2)
                cv2.putText(frame, str(cls), (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # ArUco 검출
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                center = corners[0][0].mean(axis=0)
                uv2    = np.array([center[0], center[1], 1.0])
                w2     = H.dot(uv2); w2 /= w2[2]
                mx, my = float(w2[0]), float(w2[1])

                if mx <= 380.0:
                    print(f"\nArUco가 ({mx:.3f}, {my:.3f}) 위치에서 범위 내 감지되었습니다.")
                    marker_pos = (mx, my)
                    cv2.imshow('Detection', frame)
                    cv2.waitKey(500)
                    break
                else:
                    cv2.putText(frame,
                                f"ArUco x={mx:.1f} too large",
                                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0,0,255),2)

            cv2.imshow('Detection', frame)
            if cv2.waitKey(1) == 27:
                print("사용자에 의해 종료됨.")
                return

        input_name = input("\n픽할 가구명을 입력하세요 (테이블/쇼파/의자/침대): ").strip()
        # 이름→클래스 ID 역조회
        class_id = None
        for k,v in class_names.items():
            if v == input_name:
                class_id = k
                break
        if class_id is None:
            print("잘못된 가구명입니다.")
            return

        # filtered 리스트에서 해당 클래스만 추출
        matches = [d for d in filtered if d['class'] == class_id]
        if not matches:
            print(f"화면 내에 '{input_name}' 객체가 없습니다.")
            return

        # 첫 번째 매칭 객체 사용
        px, py = matches[0]['world']
        mx, my = marker_pos
        pick_z = class_pick_z[class_id]

        # Pick
        RunPoint(mv, [px, py, 0, 0.0]); WaitArrive([px, py, 0, 0.0])
        RunPoint(mv, [px, py, pick_z,     0.0]); WaitArrive([px, py, pick_z,     0.0])
        ActivateVacuumGripper(dash, True); time.sleep(1)
        RunPoint(mv, [px, py, 0, 0.0]); WaitArrive([px, py, 0, 0.0])

        # Place
        RunPoint(mv, [mx, my, 0, 0.0]); WaitArrive([mx, my, 0, 0.0])
        RunPoint(mv, [mx, my, DROP_Z,       0.0]); WaitArrive([mx, my, DROP_Z,       0.0])
        ActivateVacuumGripper(dash, False); time.sleep(1)
        BlowOffGripper(dash, True)
        time.sleep(0.3)   # 분사 시간
        BlowOffGripper(dash, False)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        dash.DisableRobot()
        print("작업 완료. 로봇 비활성화됨.")
        
if __name__=='__main__':
    main()
