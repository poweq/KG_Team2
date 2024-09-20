import cv2
import os
import serial  # 시리얼 통신을 위한 모듈
from jd_opencv_lane_detect import JdOpencvLaneDetect

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)

# OpenCV line detector 객체 생성
cv_detector = JdOpencvLaneDetect()

# 카메라 객체 생성
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # 가로 해상도 설정
cap.set(4, 240)  # 세로 해상도 설정

# ./data 폴더가 없으면 생성
try:
    if not os.path.exists('./data'):
        os.makedirs('./data')
except OSError:
    print("failed to make ./data folder")

# 비디오 녹화 객체 생성
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_orig = cv2.VideoWriter('./data/car_video.avi', fourcc, 20.0, (320, 240))

# 초기화 루틴
for i in range(30):
    ret, img_org = cap.read()
    if ret:
        lanes, img_lane = cv_detector.get_lane(img_org)
        angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
        if img_angle is None:
            print("can't find lane...")
            pass
        else:
            print(angle)
    else:
        print("camera error")

# 주행 루틴
while True:
    ret, img_org = cap.read()
    if ret:
        # 비디오 저장
        video_orig.write(img_org)
        
        # 차선 감지
        lanes, img_lane = cv_detector.get_lane(img_org)
        angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
        if img_angle is None:
            print("can't find lane...")
            pass
        else:
            cv2.imshow('lane', img_angle)
            print(angle)

            # 앵글 값에 따라 시리얼 데이터 전송
            if 80 <= angle <= 100:
                ser.write(b'w')  # 직진 신호
                print("Sending 'w' via serial")
            elif 40 <= angle < 80:
                ser.write(b'l')  # 좌회전 신호
                print("Sending 'l' via serial")
            elif 101 <= angle <= 120:
                ser.write(b'r')  # 우회전 신호
                print("Sending 'r' via serial")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("cap error")

# 리소스 해제
cap.release()
video_orig.release()
cv2.destroyAllWindows()
ser.close()  # 시리얼 포트 닫기

