import cv2
import numpy as np

# 카메라 설정 (웹 카메라의 ID가 0인 경우)
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # 프레임 너비 설정 (640으로 더 크게 설정)
cap.set(4, 480)  # 프레임 높이 설정 (480으로 더 크게 설정)

# 모터 속도 계산 함수 (오프셋에 따른 속도 제어)
def calculate_motor_values(offset, base_speed=110):
    # 최대 회전 비율 설정 (오프셋이 클수록 큰 속도 차이)
    k_p = 0.5  # 비례 제어 상수 (P 제어)

    # 모터 속도 초기화
    motorA = motorB = motorC = motorD = base_speed

    # 비례 제어를 사용해 모터 속도 조정
    adjustment = int(k_p * abs(offset))

    if offset < -10:  # 왼쪽으로 많이 치우침
        motorA = base_speed - adjustment  # 왼쪽 전면 모터 느리게
        motorB = base_speed - adjustment  # 왼쪽 후면 모터 느리게
        motorC = base_speed + adjustment  # 오른쪽 전면 모터 빠르게
        motorD = base_speed + adjustment  # 오른쪽 후면 모터 빠르게
    elif offset > 10:  # 오른쪽으로 많이 치우침
        motorC = base_speed - adjustment  # 오른쪽 전면 모터 느리게
        motorD = base_speed - adjustment  # 오른쪽 후면 모터 느리게
        motorA = base_speed + adjustment  # 왼쪽 전면 모터 빠르게
        motorB = base_speed + adjustment  # 왼쪽 후면 모터 빠르게

    # 속도는 최소 0 이상으로 유지
    motorA = max(0, motorA)
    motorB = max(0, motorB)
    motorC = max(0, motorC)
    motorD = max(0, motorD)

    return motorA, motorB, motorC, motorD

# 다항식 피팅을 통해 곡선 차선을 추정하는 함수
def detect_curve_lane(frame):
    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 가우시안 블러 적용하여 노이즈 제거
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny 엣지 검출기 사용
    edges = cv2.Canny(blur, 50, 150)
    
    # 관심 영역 설정 (하단 절반 부분만 사용)
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
    
    # 허프 변환을 사용해 선 검출
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)
    
    if lines is None:
        return None, None

    # 검출된 선들의 모든 좌표를 추출
    points = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            points.append([x1, y1])
            points.append([x2, y2])

    # 포인트로 다항식 피팅
    points = np.array(points)
    if len(points) > 0:
        # 2차 다항식을 사용해 곡선 맞춤
        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x * vy / vx) + y)
        righty = int(((width - x) * vy / vx) + y)
        
        # 차선을 이미지에 그리기
        line_image = np.zeros_like(frame)
        cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)
        
        return (width // 2, (lefty + righty) // 2), line_image
    else:
        return None, None

# 메인 루프
while True:
    ret, frame = cap.read()
    if not ret:
        print("카메라 오류")
        break
    
    # 화면의 크기
    height, width = frame.shape[:2]
    
    # 화면의 중앙 계산
    frame_center = width // 2
    
    # 차선 감지
    lane_center, lane_image = detect_curve_lane(frame)
    
    if lane_center is None:
        print("차선을 찾을 수 없습니다.")
        continue

    # 차선 중앙과 화면 중앙의 오프셋 계산
    offset = lane_center[0] - frame_center
    print(f"차선 중앙 오프셋: {offset}")

    # 모터 값 계산 (모터 명령 전송 대신 출력만)
    motorA, motorB, motorC, motorD = calculate_motor_values(offset)
    motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
    print(f"모터 명령 출력: {motor_command}")

    # 가상의 중앙선 그리기
    cv2.line(frame, (frame_center, 0), (frame_center, height), (255, 0, 0), 2)

    # 차선과 중앙선 사이의 거리 표시
    distance_text = f"Offset: {abs(offset)} pixels"
    cv2.putText(frame, distance_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # 차선이 그려진 이미지와 가상 중앙선 겹치기
    combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

    # 결과 화면 출력
    cv2.imshow('Lane Detection with Offset', combined_image)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
cap.release()
cv2.destroyAllWindows()
