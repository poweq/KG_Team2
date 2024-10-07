import cv2
import socket
import struct
import pickle

# 설정
host_ip = "172.30.1.61"  # 수신 측 호스트 PC IP 주소
port = 5005  # 수신 측 포트 번호
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 카메라 캡처
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 이미지 인코딩
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    data = pickle.dumps(buffer)

    # 데이터를 여러 패킷으로 나누어 전송
    max_payload_size = 65500
    for i in range(0, len(data), max_payload_size):
        segment = data[i:i + max_payload_size]
        sock.sendto(struct.pack("Q", len(segment)) + segment, (host_ip, port))

    # 화면 출력
    cv2.imshow("Camera Sender", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sock.close()