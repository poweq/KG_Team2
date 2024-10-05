import cv2
import socket
import struct
import pickle
import numpy as np
import math
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

# 차선 각도 계산 함수
def calculate_angle(x1, y1, x2, y2, frame_center_x):
    dy = y1 - y2
    dx = x2 - x1
    angle_radians = math.atan2(dy, dx)

    # 각도를 양수로 변환 (0에서 180도 범위)
    angle_degrees = math.degrees(angle_radians)
    
    # atan2는 기본적으로 -180~180 범위를 제공하므로 이를 0~180 범위로 맞춤
    if angle_degrees < 0:
        angle_degrees += 180
    
    # 화면 기준으로 왼쪽이 0도, 오른쪽이 180도가 되도록 조정
    if x2 >= frame_center_x:  # 오른쪽에 있을 때
        angle_degrees = 180 - angle_degrees
    else:  # 왼쪽에 있을 때
        angle_degrees = angle_degrees

    return int(angle_degrees)  # 각도를 정수로 반환

# 차선 감지 함수
def detect_curve_lane(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height * 0.6)),
        (0, int(height * 0.6))
    ]], np.int32)
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
        
        try:
            lefty = int((-x * vy / vx) + y)
            righty = int(((width - x) * vy / vx) + y)
        except (ZeroDivisionError, ValueError) as e:
            print(f"Error calculating lefty or righty: {e}")
            return None, None, None

        line_image = np.zeros_like(frame)

        # 좌표값이 유효한지 확인 후 선을 그리기
        if isinstance(lefty, int) and isinstance(righty, int):
            cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)

        # 각도 계산
        angle = calculate_angle(0, lefty, width - 1, righty, width // 2)

        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None

# 메인 루프
while True:
    # 수신할 데이터 길이와 데이터 수신
    data, addr = sock.recvfrom(65507)  # 최대 UDP 패킷 크기
    size = struct.unpack("Q", data[:8])[0]  # 패킷 길이 추출
    frame_data = data[8:8 + size]  # 실제 이미지 데이터 추출

    # 직렬화된 데이터를 역직렬화하여 이미지로 변환
    frame = pickle.loads(frame_data)

    # JPEG 압축 해제
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    height, width = frame.shape[:2]
    frame_center_x = width // 2

    # 차선 감지 및 각도 계산
    lane_center, lane_image, angle = detect_curve_lane(frame)

    # 차선이 검출되지 않은 경우
    if lane_center is None:
        print("차선을 찾을 수 없습니다.")
        continue

    # 가상의 중앙 수직선 그리기
    cv2.line(frame, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)

    # 차선과 중앙선 사이의 각도 표시
    angle_text = f"Angle: {angle}°"  # 정수 각도 출력
    cv2.putText(frame, angle_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 차선이 그려진 이미지와 중앙선 겹치기
    combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

    # FPS 계산
    frame_count += 1
    if time.time() - start_time >= 1:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

    # FPS 표시
    cv2.putText(combined_image, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 결과 화면 출력
    cv2.imshow("Lane Detection with Adjusted Angle", combined_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
sock.close()
cv2.destroyAllWindows()