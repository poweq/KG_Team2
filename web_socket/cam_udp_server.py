import socket
import cv2
import numpy as np
import pickle
import struct
import math
import time

# UDP 설정
host_ip = "0.0.0.0"  # 서버의 IP 주소
port = 5005  # 서버 포트 번호

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host_ip, port))

print("수신 대기 중...")

# 차선 각도 계산 함수
def calculate_angle(x1, y1, x2, y2, frame_center_x):
    dy = y1 - y2
    dx = x2 - x1
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    if angle_degrees < 0:
        angle_degrees += 180
    if x2 >= frame_center_x:
        angle_degrees = 180 - angle_degrees
    return int(angle_degrees)

# 차선 감지 함수
def detect_curve_lane(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[ (0, height), (width, height), (width, int(height * 0.6)), (0, int(height * 0.6)) ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)

    if lines is None:
        return None, None, None

    points = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            points.append([x1, y1])
            points.append([x2, y2])

    points = np.array(points)
    if len(points) > 0:
        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

        lefty = int((-x * vy / vx) + y)
        righty = int(((width - x) * vy / vx) + y)

        line_image = np.zeros_like(frame)
        cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)

        angle = calculate_angle(0, lefty, width - 1, righty, width // 2)

        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None

# 메인 루프
while True:
    data, addr = sock.recvfrom(65507)
    size = struct.unpack("Q", data[:8])[0]
    frame_data = data[8:8 + size]
    frame = pickle.loads(frame_data)
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    height, width = frame.shape[:2]
    frame_center_x = width // 2

    lane_center, lane_image, angle = detect_curve_lane(frame)

    if lane_center is None:
        print("차선을 찾을 수 없습니다.")
        angle_text = "각도: N/A"
    else:
        angle_text = f"각도: {angle}°"
        print(angle_text)  # 콘솔에 각도 출력

    # 각도를 클라이언트에 전송
    if addr:
        sock.sendto(angle_text.encode(), addr)

    combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)
    cv2.putText(combined_image, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow("Lane Detection", combined_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

sock.close()