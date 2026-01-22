#!/usr/bin/env python3
import socket
import threading

HOST = '0.0.0.0'
PORT = 9999

# { client_id(str): conn(socket) }
clients = {}
lock = threading.Lock()

def handle_client(conn, addr):
    try:
        # 1) 최초 메시지로 클라이언트 ID 수신
        client_id = conn.recv(1024).decode('utf-8').strip()
        if not client_id:
            return
        with lock:
            clients[client_id] = conn
        print(f"[접속] {client_id} @ {addr}")

        # 2) 클라이언트로부터 받은 메시지 로그 출력
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"[수신 {client_id}] {data.decode('utf-8').strip()}")

    finally:
        # 연결 종료 시 clients 딕셔너리에서 제거
        with lock:
            clients.pop(client_id, None)
        conn.close()
        print(f"[해제] {client_id}")

def send_to_client(target_id, message):
    """특정 클라이언트에게만 메시지 전송"""
    with lock:
        conn = clients.get(target_id)
    if conn:
        conn.sendall(message.encode('utf-8'))
        print(f"[발신 {target_id}] {message}")
    else:
        print(f"[오류] '{target_id}' 클라이언트를 찾을 수 없습니다.")

def command_loop():
    """서버 콘솔에서 명령을 입력받아 특정 클라이언트로 전송"""
    while True:
        line = input("서버 명령> ").strip()
        if not line:
            continue
        parts = line.split(' ', 1)
        if len(parts) != 2:
            print("사용법: <클라이언트ID> <메시지>")
            continue
        target_id, msg = parts
        send_to_client(target_id, msg)

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"[서버 시작] {HOST}:{PORT}")

    # 콘솔 명령 처리 스레드 (데몬으로)
    threading.Thread(target=command_loop, daemon=True).start()

    # 클라이언트 연결 대기
    while True:
        conn, addr = server.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == '__main__':
    main()
