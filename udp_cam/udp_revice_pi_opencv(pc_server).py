# pc_video_receiver_and_angle_sender_with_lane_detect.py
import socket
import cv2
import numpy as np
import pickle
import struct
import math
from jd_opencv_lane_detect import JdOpencvLaneDetect  # 사용자의 차선 인식 코드 import (your_module 부분은 파일 이름으로 변경 필요)

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

# JdOpencvLaneDetect 클래스 인스턴스 생성
lane_detector = JdOpencvLaneDetect()

try:
    # 메인 수신 루프
    while True:
        # 영상 데이터 수신
        video_data, video_addr = video_sock.recvfrom(65507)  # 최대 UDP 패킷 크기
        video_size = struct.unpack("Q", video_data[:8])[0]   # 데이터의 크기 정보 읽기
        video_frame_data = video_data[8:8 + video_size]      # 실제 영상 데이터 분리
        frame = pickle.loads(video_frame_data)               # 직렬화 해제
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)        # JPEG를 OpenCV 이미지로 변환

        # 차선 검출 및 각도 계산
        lane_lines, lane_image = lane_detector.get_lane(frame)
        angle, _ = lane_detector.get_steering_angle(lane_image, lane_lines)

        # 차선이 검출되었는지 확인하고 각도 설정
        if lane_lines is None or len(lane_lines) == 0:
            print("차선을 찾을 수 없습니다.")
            angle_text = "각도: N/A"
            combined_image = frame  # 차선이 없을 경우, 원본 프레임만 표시
        else:
            # 각도 텍스트 설정
            angle_text = f"각도: {angle}°"
            print(f"검출된 각도: {angle_text}")  # 콘솔에 각도 출력

            # 차선과 각도를 포함한 이미지 생성
            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

            # 각도 데이터를 라즈베리파이로 송신
            angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))  # 각도 데이터 송신

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
