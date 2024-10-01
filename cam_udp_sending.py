# camera_sender.py
import cv2
import socket
import struct
import pickle
import time

# UDP 설정
host_ip = "172.30.1.31"  # 수신 측 호스트 PC IP 주소
port = 5005  # 수신 측 포트 번호
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 카메라 설정
cap = cv2.VideoCapture(0)
fps = 0
frame_count = 0
start_time = time.time()

# 해상도 설정
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 해상도를 수동으로 다시 설정 (간혹 카메라 설정이 적용되지 않는 경우에 대비)
    frame = cv2.resize(frame, (320, 240))

    # 프레임 압축 (JPEG 품질: 50)
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])

    # 직렬화하여 전송 준비
    data = pickle.dumps(buffer)

    # 데이터 송신
    sock.sendto(struct.pack("Q", len(data)) + data, (host_ip, port))

    # FPS 계산
    frame_count += 1
    if time.time() - start_time >= 1:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

    # 송신 측 화면 출력
    cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Camera Sender", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sock.close()