import cv2
import socket
import struct
import pickle
import time

# UDP 설정
host_ip = "172.30.1.61"  # 서버 IP 주소
port = 5005  # 서버 포트 번호
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 카메라 설정
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 프레임을 JPEG 형식으로 인코딩
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    
    # 데이터를 전송할 수 있도록 직렬화
    data = pickle.dumps(buffer)
    
    # 데이터 크기를 전송
    sock.sendto(struct.pack("Q", len(data)) + data, (host_ip, port))

    # 서버로부터 각도 수신
    try:
        angle_data, _ = sock.recvfrom(1024)  # 최대 1024바이트 수신
        print(f"수신된 각도: {angle_data.decode('utf-8')}°")
    except socket.timeout:
        print("각도 수신 시간 초과")

    time.sleep(0.1)  # 0.1초 간격으로 전송

cap.release()
sock.close()