import cv2
import numpy as np
import serial

# 카메라 설정 (웹 카메라의 ID가 0인 경우)
cap = cv2.VideoCapture(0)  # 웹 카메라의 ID가 0인 경우
cap.set(3, 640)  # 프레임 너비 설정 (640으로 더 크게 설정)
cap.set(4, 480)  # 프레임 높이 설정 (480으로 더 크게 설정)

# 이미지에서 하나의 차선만 감지하는 함수
def detect_single_lane(frame):
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
    
    # 허프 변환을 사용해 선 검출 (한 줄의 차선만 검출)
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=150)
    
    if lines is None:
        return None, None
    
    # 검출된 선 중에서 하나의 차선만 선택 (가장 왼쪽 차선을 선택)
    lane = sorted(lines, key=lambda line: line[0][0])[0]  # 가장 왼쪽 차선을 선택
    
    # 차선의 중앙 계산
    x1, y1, x2, y2 = lane[0]
    lane_center = (x1 + x2) // 2

    # 이미지를 출력할 수 있도록 차선 그리기
    line_image = np.zeros_like(frame)
    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

    return lane_center, line_image

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
    lane_center, lane_image = detect_single_lane(frame)
    
    if lane_center is None:
        print("차선을 찾을 수 없습니다.")
        # 차선을 못 찾을 때 스킵
        continue

    # 차선 중앙과 화면 중앙의 오프셋 계산
    offset = lane_center - frame_center
    print(f"차선 중앙 오프셋: {offset}")

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
