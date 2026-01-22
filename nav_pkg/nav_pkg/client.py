#!/usr/bin/env python3
import socket
import threading

HOST = '0.0.0.0'
PORT = 9999

def recv_thread(sock):
    """서버로부터 오는 메시지 수신 및 출력"""
    while True:
        data = sock.recv(1024)
        if not data:
            print("[연결 종료]")
            break
        print(f"[서버] {data.decode('utf-8').strip()}")

def main():
    client_id = input("클라이언트 ID 입력: ").strip()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    # 1) 최초에 ID 전송
    sock.sendall(client_id.encode('utf-8'))

    # 2) 수신 스레드 시작
    threading.Thread(target=recv_thread, args=(sock,), daemon=True).start()

    # 3) 표준 입력으로부터 서버에 보낼 메시지 입력
    try:
        while True:
            msg = input()
            if msg.lower() in ('exit', 'quit'):
                break
            sock.sendall(msg.encode('utf-8'))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__ == '__main__':
    main()
