# pc_video_receiver_angle_sender.py
import socket
import cv2
import numpy as np
import pickle
import struct
import math

# UDP 설정
video_receive_port = 5005  # 라즈베리파이에서 송신한 영상 데이터를 수신할 포트 번호
angle_send_port = 5007  # 라즈베리파이로 각도 데이터를 송신할 포트 번호
pi_ip = "172.30.1.100"  # 라즈베리파이의 IP 주소 (각도 데이터를 송신할 목적지)

# 영상 수신을 위한 소켓 생성 및 바인딩
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", video_receive_port))

# 각도 송신을 위한 소켓 생성
angle_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("영상 수신 및 각도 송신 대기 중...")

def calculate_angle(x1, y1, x2, y2, frame_center_x):
    """
    주어진 두 점을 기반으로 차선의 각도를 계산하는 함수.
    """
    dy = y1 - y2
    dx = x2 - x1
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    
    # 좌측/우측 차선에 따라 각도 조정
    if angle_degrees < 0:
        angle_degrees += 180
    if x2 >= frame_center_x:
        angle_degrees = 180 - angle_degrees
    
    return int(angle_degrees)

def detect_lane(frame):
    """
    주어진 프레임에서 차선을 검출하고, 차선의 각도를 계산하는 함수.
    """
    # 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Gaussian 블러 적용
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny Edge Detection 적용
    edges = cv2.Canny(blur, 50, 150)

    # 차선 검출을 위한 관심 영역(ROI) 설정
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[(0, height), (width, height), (width, int(height * 0.6)), (0, int(height * 0.6))]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Hough Transform을 사용하여 직선 검출
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)
    if lines is None:
        return None, None, None

    # 검출된 직선에서 차선의 중심 계산
    points = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            points.append([x1, y1])
            points.append([x2, y2])

    points = np.array(points)
    if len(points) > 0:
        # 직선 피팅
        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)

        # 직선의 양 끝점 계산
        lefty = int((-x * vy / vx) + y)
        righty = int(((width - x) * vy / vx) + y)

        # 직선을 표시할 이미지 생성
        line_image = np.zeros_like(frame)
        cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)

        # 차선 각도 계산
        angle = calculate_angle(0, lefty, width - 1, righty, width // 2)
        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None

try:
    # 메인 수신 루프
    while True:
        # 영상 데이터 수신
        video_data, video_addr = video_sock.recvfrom(65507)  # 최대 UDP 패킷 크기
        video_size = struct.unpack("Q", video_data[:8])[0]
        video_frame_data = video_data[8:8 + video_size]
        frame = pickle.loads(video_frame_data)  # 직렬화 해제
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)  # JPEG를 OpenCV 이미지로 변환

        # 차선 검출 및 각도 계산
        lane_center, lane_image, angle = detect_lane(frame)

        if lane_center is None:
            print("차선을 찾을 수 없습니다.")
            angle_text = "각도: N/A"
        else:
            angle_text = f"각도: {angle}°"
            print(angle_text)  # 콘솔에 각도 출력

            # 차선과 각도를 포함한 이미지 생성
            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

            # 각도 데이터를 라즈베리파이로 송신
            angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))  # 각도 데이터 송신

            # 각도를 영상에 표시
            cv2.putText(combined_image, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 결과 영상 출력
            cv2.imshow("Lane Detection", combined_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("사용자에 의해 프로그램이 중단되었습니다.")

finally:
    # 리소스 정리
    cv2.destroyAllWindows()
    video_sock.close()
    angle_send_sock.close()
    print("프로그램이 정상적으로 종료되었습니다.")