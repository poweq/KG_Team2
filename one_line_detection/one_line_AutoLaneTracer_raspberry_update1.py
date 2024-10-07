import cv2
import numpy as np
import math

# 카메라 설정 (웹 카메라의 ID가 0인 경우)
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # 프레임 너비 설정
cap.set(4, 480)  # 프레임 높이 설정


# 차선 각도 계산 함수 (위와 동일)
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


# 차선 감지 함수 개선
def detect_curve_lane(frame):
    # BGR에서 HLS로 변환 (흰색, 노란색 차선 잘 감지)
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 200, 0])
    upper_white = np.array([180, 255, 255])
    mask_white = cv2.inRange(hls, lower_white, upper_white)

    lower_yellow = np.array([15, 30, 115])
    upper_yellow = np.array([35, 204, 255])
    mask_yellow = cv2.inRange(hls, lower_yellow, upper_yellow)

    mask = cv2.bitwise_or(mask_white, mask_yellow)
    masked_image = cv2.bitwise_and(frame, frame, mask=mask)

    # 그레이스케일 변환 및 블러링
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Canny 에지 검출 (적응형 임계값 적용)
    median_val = np.median(blur)
    lower = int(max(0, 0.66 * median_val))
    upper = int(min(255, 1.33 * median_val))
    edges = cv2.Canny(blur, lower, upper)

    # ROI 설정
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

    # HoughLinesP를 사용하여 직선 검출
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)

    if lines is None:
        return None, None, None

    # 차선 후보군 필터링 및 평균 계산
    left_lines = []
    right_lines = []

    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < 0.5:  # 기울기가 낮으면 차선으로 간주하지 않음
                continue
            if slope < 0:  # 왼쪽 차선
                left_lines.append(line)
            else:  # 오른쪽 차선
                right_lines.append(line)

    # 왼쪽과 오른쪽 차선 각각의 중앙을 계산하여 평균 선 도출
    def average_line(lines):
        if len(lines) == 0:
            return None
        x1, y1, x2, y2 = np.mean(lines, axis=0).astype(int)
        return [x1, y1, x2, y2]

    left_avg = average_line(left_lines)
    right_avg = average_line(right_lines)

    line_image = np.zeros_like(frame)

    if left_avg is not None:
        cv2.line(line_image, (left_avg[0], left_avg[1]), (left_avg[2], left_avg[3]), (0, 255, 0), 5)
    if right_avg is not None:
        cv2.line(line_image, (right_avg[0], right_avg[1]), (right_avg[2], right_avg[3]), (0, 255, 0), 5)

    # 각도 계산
    if left_avg is not None and right_avg is not None:
        mid_x = (left_avg[2] + right_avg[2]) // 2
        angle = calculate_angle(left_avg[0], left_avg[1], right_avg[2], right_avg[3], width // 2)
        return (mid_x, height // 2), line_image, angle
    else:
        return None, None, None


# 메인 루프 (위와 동일)
while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 오류")
        break

    height, width = frame.shape[:2]
    frame_center_x = width // 2

    lane_center, lane_image, angle = detect_curve_lane(frame)

    if lane_center is None:
        print("차선을 찾을 수 없습니다.")
        continue

    offset = lane_center[0] - frame_center_x

    if 0 <= angle <= 90:
        print(f"{angle}")
    elif 90 < angle <= 180:
        print(f"{angle}")

    cv2.line(frame, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)
    cv2.line(frame, (0, height - 1), (width, height - 1), (0, 0, 255), 2)

    angle_text = f"Angle: {angle} degrees"
    cv2.putText(frame, angle_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

    cv2.imshow('Lane Detection with Adjusted Angle', combined_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
