import cv2
import socket
import threading

# 서버의 호스트와 포트 설정
HOST = ''  # 모든 네트워크 인터페이스에서 접속 허용
PORT = 8080  # 사용할 포트 번호

clients = []  # 연결된 클라이언트 소켓 리스트

def broadcast_frames():
    """웹캠 프레임을 캡처하고 모든 클라이언트에게 전송하는 함수."""
    cap = cv2.VideoCapture(0)  # 웹캠 캡처 시작 (0은 기본 웹캠 장치)
    while cap.isOpened():
        ret, frame = cap.read()  # 웹캠 프레임 캡처
        if not ret:
            break

        # 프레임을 JPEG로 인코딩
        _, jpeg = cv2.imencode('.jpg', frame)
        frame_data = jpeg.tobytes()

        # 연결된 모든 클라이언트에게 프레임 전송
        for client in clients:
            try:
                client.sendall(b"--frame\r\n" +
                               b"Content-Type: image/jpeg\r\n" +
                               b"Content-Length: " + str(len(frame_data)).encode() + b"\r\n\r\n" +
                               frame_data + b"\r\n")
            except:
                clients.remove(client)  # 전송 실패 시 클라이언트 리스트에서 제거

    cap.release()  # 웹캠 릴리스

def handle_client(client_socket):
    """클라이언트 연결을 처리하는 함수."""
    client_socket.sendall(b"HTTP/1.1 200 OK\r\n" +
                          b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n")
    
    clients.append(client_socket)  # 클라이언트를 리스트에 추가

def start_server():
    """멀티 스레드 웹서버를 시작하는 함수."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)  # 최대 5개의 연결 대기
    print(f"[INFO] Server started on {HOST}:{PORT}")

    threading.Thread(target=broadcast_frames, daemon=True).start()  # 프레임 브로드캐스트 시작

    while True:
        client_socket, addr = server_socket.accept()
        print(f"[INFO] Connection from {addr}")

        client_handler = threading.Thread(target=handle_client, args=(client_socket,))
        client_handler.start()

if __name__ == "__main__":
    start_server()
