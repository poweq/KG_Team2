# raspberry_pi_camera_sender_with_arduino_no_delay.py
import cv2
import socket
import serial  # 시리얼 통신 모듈
import struct
import pickle
import threading  # 스레드를 사용하여 각도 데이터 수신 병렬 처리
import time

# UDP 설정
pc_ip = "172.30.1.46"  # Windows PC의 IP 주소 (환경에 맞게 설정)
video_send_port = 5005  # Windows PC로 영상 데이터를 송신할 포트 번호
angle_receive_port = 5007  # Windows PC로부터 각도 데이터를 수신할 포트 번호

# 아두이노 시리얼 통신 설정
arduino_port = '/dev/ttyAMA1'  # 아두이노와 연결된 시리얼 포트
baud_rate = 9600  # 통신 속도
ser = serial.Serial(arduino_port, baud_rate, timeout=1)  # 시리얼 포트 열기

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
    print("카메라를 열 수 없습니다. 프로그램을 종료합니다.")
    exit(1)

print(f"카메라 데이터 송신 및 각도 수신 대기 중... (Ctrl + C로 종료)")
print(f"각도 데이터 수신 대기 중... (포트: {angle_receive_port})")

# 데이터 수신 여부 확인용 변수
last_received_time = time.time()  # 마지막으로 각도 데이터를 수신한 시간을 기록
angle_data_received = False       # 각도 데이터가 수신되었는지 여부를 나타내는 플래그
latest_angle = "N/A"              # 수신한 각도 데이터 저장
lock = threading.Lock()           # 스레드 간 데이터 보호를 위한 락 설정

# 각도 데이터 수신 함수 (별도의 스레드에서 실행)
def receive_angle_data():
    global latest_angle, angle_data_received, last_received_time
    while True:
        try:
            # 각도 데이터 수신
            angle_data, addr = angle_receive_sock.recvfrom(1024)  # 최대 1024 바이트 크기의 데이터 수신
            latest_angle = angle_data.decode()  # 바이트 데이터를 문자열로 변환
            with lock:
                angle_data_received = True  # 데이터 수신 상태 업데이트
                last_received_time = time.time()  # 마지막 수신 시간 갱신
            print(f"수신된 각도 데이터: {latest_angle}")  # 수신된 각도 출력
        except socket.timeout:
            with lock:
                # 타임아웃 발생 시: 각도 데이터를 수신하지 못한 경우
                angle_data_received = False
        except Exception as e:
            print(f"각도 데이터 수신 중 오류 발생: {e}")
            continue

# 각도 데이터를 수신하는 스레드 시작
angle_thread = threading.Thread(target=receive_angle_data, daemon=True)
angle_thread.start()

try:
    # 메인 송신 루프 (카메라 데이터 송신 및 아두이노로의 데이터 송신)
    while True:
        # 카메라로부터 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다. 프로그램을 종료합니다.")
            break

        # 프레임을 JPEG로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            print("프레임 인코딩에 실패하였습니다.")
            continue

        # 인코딩된 프레임을 직렬화하여 UDP로 전송
        try:
            data = pickle.dumps(buffer)  # 직렬화
            # 데이터 크기를 먼저 보내고, 그 뒤에 실제 데이터를 전송
            video_sock.sendto(struct.pack("Q", len(data)) + data, (pc_ip, video_send_port))
            print("프레임을 성공적으로 전송하였습니다.")
        except Exception as e:
            print(f"영상 데이터 송신 중 오류 발생: {e}")
            continue

        # 아두이노로 각도 데이터 송신
        if ser.is_open:
            try:
                with lock:
                    ser.write((latest_angle + "\n").encode())  # 아두이노로 각도 데이터 전송 (줄바꿈 추가)
                print(f"아두이노로 각도 데이터 송신: {latest_angle}")
            except Exception as e:
                print(f"아두이노로 데이터 송신 중 오류 발생: {e}")

        # 프레임 송신 주기 조절 (FPS 설정)
        time.sleep(0.033)  # 약 30fps로 송신 (필요에 따라 조절 가능)

except KeyboardInterrupt:
    print("사용자에 의해 프로그램이 중단되었습니다.")

finally:
    # 리소스 정리
    cap.release()
    video_sock.close()
    angle_receive_sock.close()
    ser.close()  # 시리얼 포트 닫기
    print("프로그램이 정상적으로 종료되었습니다.")
