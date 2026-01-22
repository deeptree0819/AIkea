#!/usr/bin/env python3
import os
import cv2
import numpy as np
import time
import threading
import socket
import queue
import re
from dobot_api import (
    DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
)
from ultralytics import YOLO

# --- 설정 ---
HOST = '0.0.0.0'
PORT = 9997
DESTINATIONS = ['카페', '레스토랑', '건담베이스', '키즈카페']

# --- 전역 큐 및 매핑 ---
command_queue = queue.Queue()   # (client_id, msg)
clients       = {}              # client_id -> socket
pending_furn  = {}              # client_id -> furniture

# --- Dobot 상태 모니터링 변수 ---
current_actual     = None
algorithm_queue    = None
enableStatus_robot = None
robotErrorState    = False
state_lock         = threading.Lock()

# --- Socket 서버 및 클라이언트 핸들러 ---
def handle_client(conn):
    client_id = None
    with conn:
        print(f"New connection: {conn.getpeername()}")
        while True:
            data = conn.recv(1024)
            if not data:
                if client_id:
                    print(f"{client_id} disconnected.")
                    clients.pop(client_id, None)
                break
            msg = data.decode().strip()
            # 1) 등록: pinky_1 또는 pinky_2
            if msg in ['pinky_1', 'pinky_2']:
                client_id = msg
                clients[client_id] = conn
                print(f"Registered client: {client_id}")
                conn.sendall(b"REGISTERED\n")
                continue
            # 2) 기타 메시지: 명령 또는 도착 신호
            print(f"Received from {client_id}: {msg}")
            command_queue.put((client_id, msg))
            conn.sendall(b"ACK\n")

def socket_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(5)
    print(f"Socket server running on {HOST}:{PORT}")
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle_client, args=(conn,), daemon=True).start()

# --- Dobot 연결 및 피드백/오류 관리 ---
def ConnectRobot(ip='192.168.1.6', dash_port=29999, move_port=30003, feed_port=30004):
    dash = DobotApiDashboard(ip, dash_port)
    mv   = DobotApiMove(ip, move_port)
    fd   = DobotApi(ip, feed_port)
    return dash, mv, fd

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
    global current_actual  # ← 최상단에 선언

    # 이전 피드백 버리고 새로 들어올 때까지 대기
    with state_lock:
        current_actual = None

    # 1) 최초 피드백 대기
    while True:
        with state_lock:
            if current_actual is not None:
                break
        time.sleep(0.005)

    # 2) 목표 도착 대기 (최대 5초)
    start_time = time.time()
    while True:
        with state_lock:
            actual = current_actual.copy()
        if all(abs(actual[i] - target[i]) <= 1 for i in range(len(target))):
            return
        if time.time() - start_time > 5.0:
            print(f"[WaitArrive] 타임아웃: target={target}, actual={actual}")
            return
        time.sleep(0.005)

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
    dashboard.DO(2, 1 if on else 0)
    print(f"Blow-off {'ON' if on else 'OFF'}")

# --- Pick & Place 헬퍼 ---
def pick_and_place(furn, marker_pos, dash, mv,
                   cap, model, H,
                   class_names, class_pick_z, DROP_Z,
                   home_pos):

    # YOLO 감지
    ret, frame = cap.read()
    if not ret:
        print("[PICK] 카메라 읽기 실패.")
        return
    dets = model(frame)[0]
    # 클래스 필터링
    cls_id = next((k for k, v in class_names.items() if v == furn), None)
    if cls_id is None:
        print(f"[PICK] 잘못된 가구명: {furn}")
        return
    found = []
    for b in dets.boxes:
        if int(b.cls[0]) != cls_id:
            continue
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        cx, cy = (x1 + x2)//2, (y1 + y2)//2
        uv = np.array([cx, cy, 1.0])
        w = H.dot(uv); w /= w[2]
        wx, wy = float(w[0]), float(w[1])
        if wx <= 380.0:
            found.append((wx, wy))
    if not found:
        print(f"[PICK] 화면에 {furn} 없음.")
        return
    px, py = found[0]
    pz = class_pick_z[cls_id]
    # Pick
    RunPoint(mv, [px, py, 0, 0]);       WaitArrive([px, py, 0, 0])
    RunPoint(mv, [px, py, pz, 0]);     WaitArrive([px, py, pz, 0])
    ActivateVacuumGripper(dash, True); time.sleep(1)
    RunPoint(mv, [px, py, 10, 0]);       WaitArrive([px, py, 10, 0])
    RunPoint(mv, home_pos);            WaitArrive(home_pos)
    # Place
    mx, my = marker_pos
    RunPoint(mv, [mx, my, 10, 0]);       WaitArrive([mx, my, 10, 0])
    RunPoint(mv, [mx, my, DROP_Z, 0]); WaitArrive([mx, my, DROP_Z, 0])
    ActivateVacuumGripper(dash, False); time.sleep(1)
    BlowOffGripper(dash, True);        time.sleep(0.3)
    BlowOffGripper(dash, False)
    print(f"[PICK] {furn} pick&place 완료.")

# --- 콘솔 입력 리스너 ---
def stdin_listener():
    while True:
        try:
            cmd = input("Enter <목적지> <가구>: ").strip()
        except EOFError:
            break
        if not cmd:
            continue
        command_queue.put((None, cmd))

# --- 메인 루프 ---
def main():
    # 1) Socket 서버 & 콘솔 리스너 시작
    threading.Thread(target=socket_server, daemon=True).start()
    threading.Thread(target=stdin_listener, daemon=True).start()
    # 2) Dobot 초기화 및 모니터링 스레드
    dash, mv, fd = ConnectRobot()
    dash.EnableRobot()
    threading.Thread(target=GetFeed, args=(fd,), daemon=True).start()
    threading.Thread(target=ClearRobotError, args=(dash,), daemon=True).start()
    # 3) 초기 상태: 기준 위치 이동 및 그리퍼 OFF
    base = [294.0, 2.0, 10.0, 0.0]
    print("Moving to base:", base)
    RunPoint(mv, base); WaitArrive(base)
    print("준비 완료")
    ActivateVacuumGripper(dash, False)
    # 4) 카메라/모델/ARUCO/Homography 초기화
    cap   = cv2.VideoCapture(0)
    model = YOLO(os.getenv('YOLO_MODEL_PATH', '/home/kiro06/nav_ws/src/nav_pkg/nav_pkg/best.pt'))
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    pts_aruco = np.array([[414.8,285.8],[412.8,232.0],[356.5,207.8],[371.0,222.2],
                           [334.0,217.5],[367.0,257.2],[333.2,237.2],[312.0,222.0],
                           [305.0,271.0],[266.2,219.5],[278.0,263.0],[258.5,234.5],
                           [234.5,216.0],[252.2,265.5],[196.2,240.0],[205.2,285.2],
                           [160.5,290.8],[122.0,333.0],[203.2,312.8],[113.8,371.0],
                           [456.5,337.5],[435.5,280.0],[505.5,293.2],[483.0,373.0]], np.float32)
    pts_robot = np.array([[210.25,-164.9],[302.0,-176.4],[354.58,-86.32],[327.15,-107.33],
                           [342.64,-47.6],[271.25,-99.77],[298.5,-44.5],[339.13,-9.18],
                           [256.1,9.75],[358.7,45.52],[277.6,55.15],[325.9,84.83],
                           [361.5,121.66],[274.6,99.01],[332.2,189.1],[250.4,181.0],
                           [248.153,259.4],[186.5,327.35],[185.8,187.654],[122.1,344.9],
                           [120.0,-225.5],[220.8,-204.82],[185.8,-315.9],[60.28,-260.13]], np.float32)
    H, _ = cv2.findHomography(pts_aruco, pts_robot)
    # 클래스 맵
    class_names  = {0:"테이블",1:"쇼파",2:"의자",3:"침대"}
    class_pick_z = {0:-129.0,1:-129.0,2:-115.0,3:-149.0}
    DROP_Z       = -10

    print("서버 준비 완료. 명령 대기 중...")
    # 5) 명령 처리 루프
    while True:
        client_id, msg = command_queue.get()
        # 1) 목적지+가구
        parts = msg.split()
        if len(parts)==2 and parts[0] in DESTINATIONS:
            dest, furn = parts
            target = 'pinky_1' if dest in ['카페','레스토랑'] else 'pinky_2'
            if target in clients:
                clients[target].sendall(f"{dest}\n".encode())
                pending_furn[target] = furn
                print(f"Sent '{dest}' to {target}, pending '{furn}'")
            else:
                print(f"{target} not connected.")

        # 2) ARRIVED (단순 문자)
        elif msg == 'ARRIVED' and client_id in pending_furn:
            # 1) 꺼낸 가구 이름
            furn = pending_furn.pop(client_id)
            print(f"{client_id} arrived → capturing frame…")

            # 2) 잠시 대기 후 프레임 캡처
            time.sleep(5.0)
            # ─── 카메라 버퍼 플러시 ───
            for _ in range(5):
                cap.grab()
            ret, frame = cap.read()
            
            if not ret:
                print("Frame capture failed at ARRIVED.")
                continue

            # 3) (선택) YOLO 박스 그리기
            dets = model(frame)[0]
            for b in dets.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0])
                cls = int(b.cls[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, str(cls), (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 4) ARUCO 검출 + homography 변환
            corners, ids, _ = detector.detectMarkers(frame)
            if ids is None or len(ids) == 0:
                print("ARRIVED but ARUCO not detected.")
                continue
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            center = corners[0][0].mean(axis=0)
            uv = np.array([[center[0], center[1]]], dtype=np.float32).reshape(-1, 1, 2)
            world = cv2.perspectiveTransform(uv, H).reshape(2)
            marker_pos = (float(world[0]), float(world[1]))
            print(f"[ARRIVED] marker_pos = {marker_pos}")

            # 5) 캡처 이미지 저장 (디버깅용)
            cv2.imwrite('arrival_annotated.png', frame)

            # 6) vision-based home_pos 계산
            #    X,Y는 marker_pos, Z와 Yaw는 초기 base 값 사용
            home_pos = base
            print(f"[ARRIVED] using home_pos = {home_pos}")

            # 7) pick & place (home_pos 인자로 전달)
            pick_and_place(
                furn, marker_pos,
                dash, mv,
                cap, model, H,
                class_names, class_pick_z, DROP_Z,
                home_pos=home_pos
            )

            # 8) 완료 신호 및 홈 복귀
            clients[client_id].sendall(b"DONE\n")
            RunPoint(mv, home_pos)
            WaitArrive(home_pos)
            print(f"Sent DONE to {client_id}")

            # 9) 임시 이미지 삭제
            try:
                os.remove('arrival_annotated.png')
                print("Deleted 'arrival_annotated.png'")
            except OSError:
                pass
            
if __name__ == '__main__':
    main()
