import cv2
import numpy as np
import time

# PID 제어 변수 설정
Kp = 0.6
Ki = 0.0
Kd = 0.2
previous_error = 0
total_error = 0

# 이미지 처리 함수
def process_image(frame):
    # 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 가우시안 블러 적용
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Canny 에지 검출
    edges = cv2.Canny(blur, 50, 150)
    
    # 관심 영역 설정 (화면의 하단부만 분석)
    height, width = edges.shape
    mask = np.zeros_like(edges)
    roi = np.array([[(0, height), (width, height), (width // 2, height // 2)]], np.int32)
    cv2.fillPoly(mask, roi, 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    return masked_edges

# PID 제어 함수
def pid_control(error):
    global previous_error, total_error
    total_error += error  # 적분 계산
    correction = Kp * error + Kd * (error - previous_error)  # PD 제어
    previous_error = error
    return correction

# 메인 함수
def main():
    # 카메라 초기화
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 차선 인식
        processed_frame = process_image(frame)
        
        # 차선 중심과 화면 중심 비교
        height, width = processed_frame.shape
        lane_center = width // 2  # 간단한 예로 차선 중심을 화면의 중앙으로 가정
        
        # 가장 큰 윤곽선을 찾아 차선 위치 계산
        contours, _ = cv2.findContours(processed_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                lane_center = int(M['m10'] / M['m00'])
        
        # 에러 계산
        frame_center = width // 2
        error = lane_center - frame_center
        
        # PID 제어로 보정값 계산
        correction = pid_control(error)
        
        # 모터 속도 출력 (실제로는 모터 제어 코드로 대체)
        left_motor_speed = 100 - correction
        right_motor_speed = 100 + correction
        print(f"Left Motor Speed: {left_motor_speed}, Right Motor Speed: {right_motor_speed}")
        
        # 차선 검출 결과 화면 출력
        cv2.imshow("Lane Detection", processed_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
