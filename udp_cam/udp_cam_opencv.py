# raspberry_pi_camera_sender_continuous.py
import cv2
import socket
import struct
import pickle
import logging
import threading  # 각도 수신을 별도의 스레드에서 수행
import time

# 로깅 설정
logging.basicConfig(filename='camera_sender.log', level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# UDP 설정
pc_ip = "172.30.1.46"  # Windows PC의 IP 주소 (환경에 맞게 설정)
video_send_port = 5005  # Windows PC로 영상 데이터를 송신할 포트 번호
angle_receive_port = 5007  # Windows PC로부터 각도 데이터를 수신할 포트 번호

# 영상 송신을 위한 소켓 생성
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 각도 수신을 위한 소켓 생성 및 바인딩
angle_receive_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
angle_receive_sock.bind(("0.0.0.0", angle_receive_port))

# 각도 수신 소켓의 타임아웃 설정 (예: 1초)
angle_receive_sock.settimeout(1.0)  # 1초 동안 수신되지 않으면 타임아웃 발생

# 카메라 설정 (웹캠 사용)
cap = cv2.VideoCapture(0)  # 0번 장치(기본 웹캠) 사용
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # 카메라 해상도 너비 설정
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 카메라 해상도 높이 설정

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    logging.error("카메라를 열 수 없습니다. 프로그램을 종료합니다.")
    print("카메라를 열 수 없습니다. 프로그램을 종료합니다.")
    exit(1)

print(f"CLI 환경에서 카메라 데이터 송신 중... (Ctrl + C로 종료)")
print(f"각도 데이터 수신 대기 중... (포트: {angle_receive_port})")

# 데이터 수신 여부 확인용 변수
last_received_time = time.time()  # 마지막으로 각도 데이터를 수신한 시간을 기록
angle_data_received = False       # 각도 데이터가 수신되었는지 여부를 나타내는 플래그
latest_angle = "N/A"              # 수신한 각도 데이터 저장

# 각도 데이터 수신 함수 (별도 스레드에서 실행)
def receive_angle_data():
    global last_received_time, angle_data_received, latest_angle
    while True:
        try:
            angle_data, addr = angle_receive_sock.recvfrom(1024)  # 최대 1024 바이트 크기의 데이터 수신
            latest_angle = angle_data.decode()  # 바이트 데이터를 문자열로 변환
            print(f"수신된 각도 데이터: {latest_angle}")  # 수신된 각도 출력
            last_received_time = time.time()  # 마지막으로 데이터를 수신한 시간 갱신
            angle_data_received = True  # 데이터 수신 상태 업데이트
        except socket.timeout:
            # 타임아웃 발생 시: 각도 데이터를 수신하지 못한 경우
            current_time = time.time()
            if current_time - last_received_time >= 1.0:  # 1초 이상 데이터 수신이 없으면
                print("1초 동안 각도 데이터를 수신하지 못했습니다. 카메라 데이터 송신을 계속합니다.")
                angle_data_received = False  # 수신 실패 상태 업데이트
        except Exception as e:
            print(f"각도 데이터 수신 중 오류 발생: {e}")
            continue

# 각도 데이터를 수신하는 스레드 시작
angle_thread = threading.Thread(target=receive_angle_data, daemon=True)
angle_thread.start()

try:
    # 메인 송신 루프
    while True:
        # 카메라로부터 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            logging.error("프레임을 읽을 수 없습니다. 프로그램을 종료합니다.")
            print("프레임을 읽을 수 없습니다. 프로그램을 종료합니다.")
            break

        # 프레임을 JPEG로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            logging.warning("프레임 인코딩에 실패하였습니다.")
            continue

        # 인코딩된 프레임을 직렬화하여 UDP로 전송
        try:
            data = pickle.dumps(buffer)  # 직렬화
            # 데이터 크기를 먼저 보내고, 그 뒤에 실제 데이터를 전송
            video_sock.sendto(struct.pack("Q", len(data)) + data, (pc_ip, video_send_port))
            logging.info("프레임을 성공적으로 전송하였습니다.")
            print("프레임을 성공적으로 전송하였습니다.")
        except Exception as e:
            logging.error(f"영상 데이터 송신 중 오류 발생: {e}")
            print(f"영상 데이터 송신 중 오류 발생: {e}")
            continue

        # 각도 데이터 수신 상태를 출력
        if not angle_data_received:
            print("각도 데이터가 수신되지 않았습니다. 카메라 데이터만 송신 중입니다.")
        else:
            print(f"최근 수신된 각도: {latest_angle}")
        
        # 프레임 송신 주기 (필요시 조절 가능)
        time.sleep(0.033)  # 약 30fps로 송신 (주기 조절)

except KeyboardInterrupt:
    print("사용자에 의해 프로그램이 중단되었습니다.")

finally:
    # 리소스 정리
    cap.release()
    video_sock.close()
    angle_receive_sock.close()
    print("프로그램이 정상적으로 종료되었습니다.")
