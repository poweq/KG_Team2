import cv2
import numpy as np
import math

# 카메라 설정 (웹 카메라의 ID가 0인 경우)
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # 프레임 너비 설정 (640으로 더 크게 설정)
cap.set(4, 480)  # 프레임 높이 설정 (480으로 더 크게 설정)

# 모터 속도 계산 함수 (오프셋에 따른 속도 제어)
def calculate_motor_values(offset, base_speed=110):
    k_p = 0.5  # 비례 제어 상수 (P 제어)
    motorA = motorB = motorC = motorD = base_speed
    adjustment = int(k_p * abs(offset))

    if offset < -10:  # 왼쪽으로 많이 치우침
        motorA = base_speed - adjustment
        motorB = base_speed - adjustment
        motorC = base_speed + adjustment
        motorD = base_speed + adjustment
    elif offset > 10:  # 오른쪽으로 많이 치우침
        motorC = base_speed - adjustment
        motorD = base_speed - adjustment
        motorA = base_speed + adjustment
        motorB = base_speed + adjustment

    motorA = max(0, motorA)
    motorB = max(0, motorB)
    motorC = max(0, motorC)
    motorD = max(0, motorD)

    return motorA, motorB, motorC, motorD

# 차선 각도 계산 함수
def calculate_angle(x1, y1, x2, y2, frame_center_y):
    # 차선의 기울기 계산 (dy/dx)
    dy = y2 - y1
    dx = x2 - x1

    # 수직선(화면의 중앙선)은 x 축 기준으로 기울기가 0 (dy/dx = 무한대)
    # atan2를 사용하여 각도를 라디안으로 계산
    angle_radians = math.atan2(dy, dx)
    
    # 라디안을 각도로 변환
    angle_degrees = math.degrees(angle_radians)

    return angle_degrees

# 다항식 피팅을 통해 곡선 차선을 추정하는 함수
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
        lefty = int((-x * vy / vx) + y)
        righty = int(((width - x) * vy / vx) + y)

        line_image = np.zeros_like(frame)
        cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)

        # 각도 계산 (화면 중앙선과 차선 사이)
        angle = calculate_angle(0, lefty, width - 1, righty, height // 2)

        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None

# 메인 루프
while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 오류")
        break

    height, width = frame.shape[:2]
    frame_center = width // 2

    # 차선 감지 및 각도 계산
    lane_center, lane_image, angle = detect_curve_lane(frame)

    if lane_center is None:
        print("차선을 찾을 수 없습니다.")
        continue

    # 차선 중앙과 화면 중앙의 오프셋 계산
    offset = lane_center[0] - frame_center
    print(f"차선 중앙 오프셋: {offset}")

    # 차선과 화면 중앙선 사이의 각도 출력
    print(f"차선과 화면 중앙선 사이의 각도: {angle:.2f}도")

    # 모터 값 계산
    motorA, motorB, motorC, motorD = calculate_motor_values(offset)
    motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
    print(f"모터 명령 출력: {motor_command}")

    # 가상의 중앙선 그리기
    cv2.line(frame, (frame_center, 0), (frame_center, height), (255, 0, 0), 2)

    # 차선과 중앙선 사이의 거리와 각도 표시
    distance_text = f"Offset: {abs(offset)} pixels"
    angle_text = f"Angle: {angle:.2f} degrees"
    cv2.putText(frame, distance_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(frame, angle_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 차선이 그려진 이미지와 중앙선 겹치기
    combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

    # 결과 화면 출력
    cv2.imshow('Lane Detection with Angle', combined_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
