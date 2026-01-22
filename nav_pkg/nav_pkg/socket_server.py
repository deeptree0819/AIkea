import socket
import threading

HOST = '0.0.0.0'
PORT = 9999
clients = {}

# AMR 연결 처리
def handle_client(conn, addr):
    print(f'[키오스크] {addr} 접속됨')
    robot_id = conn.recv(1024).decode().strip()
    clients[robot_id] = conn
    print(f'[키오스크] 등록된 AMR: {robot_id}')
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f'[키오스크] [{robot_id}]로부터: {data.decode().strip()}')
    except ConnectionResetError:
        pass
    finally:
        print(f'[키오스크] {robot_id} 연결 해제')
        clients.pop(robot_id, None)
        conn.close()

# 서버 스레드
def server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f'[키오스크] 서버 시작: {HOST}:{PORT}')
        while True:
            conn, addr = s.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

# 메인: 장소명만 전송
def main():
    threading.Thread(target=server, daemon=True).start()
    print('[키오스크] 사용법: <robot_id> <place_name>')
    while True:
        cmd = input('> ').split()
        if len(cmd) != 2:
            print('  → 입력 오류, 예: robot1 kitchen')
            continue
        robot_id, place = cmd
        conn = clients.get(robot_id)
        if not conn:
            print(f'  → {robot_id} 미연결')
        else:
            conn.sendall(place.encode())
            print(f'  → {robot_id} 에게 장소 전송: {place}')

if __name__ == '__main__':
    main()