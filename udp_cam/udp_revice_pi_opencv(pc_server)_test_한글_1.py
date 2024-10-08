import socket
import cv2
import numpy as np
import pickle
import struct
import math

# UDP 설정
video_receive_port = 5005  # 라즈베리파이에서 송신한 영상 데이터를 수신할 포트 번호
angle_send_port = 6000  # 라즈베리파이로 각도 데이터를 송신할 포트 번호
pi_ip = "172.30.1.100"  # 라즈베리파이의 IP 주소 (각도 데이터를 송신할 목적지)

# 영상 수신을 위한 소켓 생성 및 바인딩
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", video_receive_port))

# 각도 송신을 위한 소켓 생성
angle_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("영상 수신 및 각도 송신 대기 중...")

# 차선 각도 계산 함수
def calculate_angle(x1, y1, x2, y2):
    dy = y1 - y2
    dx = x2 - x1
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    
    if angle_degrees < 0:
        angle_degrees += 180
    
    return int(angle_degrees)

# 차선 검출 함수
def detect_curve_lane(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))  # White lane filter
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))  # Yellow lane filter
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    filtered_frame = cv2.bitwise_and(frame, frame, mask=combined_mask)

    gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[ 
        (0, height),
        (width, height),
        (int(width * 0.6), int(height * 0.6)),
        (int(width * 0.4), int(height * 0.6))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 30, minLineLength=30, maxLineGap=200)

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

        # 좌표값이 유효한지 확인한 후 선을 그립니다
        if isinstance(lefty, int) and isinstance(righty, int):
            try:
                cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)
            except Exception as e:
                print(f"Error drawing line: {e}")

        angle = calculate_angle(-width // 2, lefty, width // 2, righty)  # x축 중심을 0으로 설정

        return (0, (lefty + righty) // 2), line_image, angle  # x축 중심을 기준으로 0을 반환
    else:
        return None, None, None

try:
    while True:
        # 영상 데이터 수신
        try:
            video_data, video_addr = video_sock.recvfrom(65507)  # 최대 UDP 패킷 크기
            video_size = struct.unpack("Q", video_data[:8])[0]   # 데이터의 크기 정보 읽기
            video_frame_data = video_data[8:8 + video_size]      # 실제 영상 데이터 분리
            frame = pickle.loads(video_frame_data)               # 직렬화 해제
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)        # JPEG를 OpenCV 이미지로 변환
        except Exception as e:
            print(f"카메라 수신 오류 발생: {e}")
            continue  # 오류가 발생해도 프로그램을 계속 실행

        # 차선 검출 및 각도 계산
        lane_center, lane_image, angle = detect_curve_lane(frame)

        height, width = frame.shape[:2]
        frame_center_x = width // 2

        # 차선이 검출되었는지 확인하고 각도 설정
        if lane_center is None:
            print("차선을 찾을 수 없습니다.")
            angle_text = "각도: N/A"
            combined_image = frame  # 차선이 없을 경우, 원본 프레임만 표시
        else:
            # 각도 텍스트 설정
            angle_text = f"각도: {angle}"
            print(f"검출된 각도: {angle_text}")  # 콘솔에 각도 출력

            # 차선과 각도를 포함한 이미지 생성
            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

            # 각도 데이터를 라즈베리파이로 송신
            try:
                angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))  # 각도 데이터 송신
            except Exception as e:
                print(f"각도 전송 중 오류 발생: {e}")

        # 수직 중심선 그리기
        cv2.line(combined_image, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)

        # 수평선 그리기 (화면의 밑바닥에 위치)
        cv2.line(combined_image, (0, height - 1), (width, height - 1), (0, 255, 0), 2)

        # 각도를 영상에 표시
        cv2.putText(combined_image, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 결과 영상 출력 (차선이 없을 경우에도 영상 출력)
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
