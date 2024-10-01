# camera_receiver.py
import cv2
import socket
import struct
import pickle
import time

# UDP 설정
host_ip = "0.0.0.0"  # 수신 측 호스트 IP 주소 (모든 인터페이스에서 수신)
port = 5005  # 수신 측 포트 번호
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host_ip, port))

print("수신 대기 중...")

fps = 0
frame_count = 0
start_time = time.time()

while True:
    # 수신할 데이터 길이와 데이터 수신
    data, addr = sock.recvfrom(65507)  # 최대 UDP 패킷 크기
    size = struct.unpack("Q", data[:8])[0]  # 패킷 길이 추출
    frame_data = data[8:8 + size]  # 실제 이미지 데이터 추출

    # 직렬화된 데이터를 역직렬화하여 이미지로 변환
    frame = pickle.loads(frame_data)

    # JPEG 압축 해제
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    # FPS 계산
    frame_count += 1
    if time.time() - start_time >= 1:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

    # FPS 표시
    cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 프레임을 화면에 표시
    cv2.imshow("Camera Receiver", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

sock.close()
cv2.destroyAllWindows()
