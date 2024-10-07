import cv2
import os
from jd_opencv_lane_detect import JdOpencvLaneDetect
from jd_car_motor_l9110 import JdCarMotorL9110

# 모터 제어 함수 예제 (구현에 따라 수정 필요)
def control_motor(direction):
    if direction == 'left':
        print("L")
        motor.motor_turn_left(speed)
    elif direction == 'straight':
        print("S")
        motor.motor_move_forward(speed)
    elif direction == 'right':
        print("R")
        motor.motor_turn_right(speed)
    else:
        print("Invalid direction")

# 객체 생성
cv_detector = JdOpencvLaneDetect()
motor = JdCarMotorL9110()
speed = 10  # 모터 속도 설정 (적절한 값으로 조정 필요)

# 카메라 객체 및 해상도 설정
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # 가로 해상도
cap.set(4, 240)  # 세로 해상도

# 데이터 폴더 생성
try:
    if not os.path.exists('./data'):
        os.makedirs('./data')
except OSError:
    print("Failed to make ./data folder")

# 비디오 녹화 객체 생성
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_orig = cv2.VideoWriter('./data/car_video.avi', fourcc, 20.0, (320, 240))

# 서보 모터 오프셋 (주석 처리된 부분은 실제로 필요할 수 있음)
#servo_offset = 1
#servo.servo[0].angle = 90 + servo_offset

# 모터 시작
motor.motor_move_forward(speed)  # 초기 모터 시작

# 실제 주행 루프
while True:
    ret, img_org = cap.read()
    if ret:
        # 카메라 이미지 기록
        video_orig.write(img_org)

        # 차선 찾기
        lanes, img_lane = cv_detector.get_lane(img_org)
        angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)

        if img_angle is None:
            print("Can't find lane...")
            control_motor('straight')  # 차선을 찾을 수 없을 때는 직진
        else:
            # 각도에 따른 방향 결정
            if 0 <= angle <83:
                control_motor('left')
            elif 83 <= angle < 100:
                control_motor('straight')
            elif 100 <= angle <= 180:
                control_motor('right')
            else:
                print("Angle out of range")

            cv2.imshow('lane', img_angle)
            print(f"Angle: {angle}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Camera error")

# 주행 종료 및 자원 해제
motor.motor_stop()
cap.release()
video_orig.release()
cv2.destroyAllWindows()
