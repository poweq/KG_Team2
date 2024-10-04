
# camera_sender_ffmpeg.py
import cv2
import subprocess
import time

# 송신할 호스트 IP 주소와 포트 설정
host_ip = "172.30.1.31"  # 수신 측 호스트 PC IP 주소
port = 5005  # 수신 측 포트 번호

# 카메라 설정
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 15)

# FFMPEG 명령어 설정
ffmpeg_command = [
    'ffmpeg',
    '-y',  # 기존 파일 덮어쓰기
    '-f', 'rawvideo',  # 입력 비디오 포맷 설정
    '-pix_fmt', 'bgr24',  # 픽셀 포맷 (OpenCV 기본 포맷)
    '-s', '320x240',  # 해상도 설정
    '-r', '30',  # 프레임 속도 (15 FPS)
    '-i', '-',  # 표준 입력에서 입력을 받음
    '-c:v', 'libx264',  # 비디오 코덱 설정
    '-preset', 'ultrafast',  # 인코딩 속도 설정
    '-bufsize', '32k',  # 버퍼 크기 줄이기 (지연 최소화)
    '-max_delay', '0',  # 최대 지연 시간을 0으로 설정
    '-tune', 'zerolatency',  # 지연 시간 최소화
    '-f', 'mpegts',  # 전송 포맷 (MPEG-TS)
    f'udp://{host_ip}:{port}'  # 송신 주소 (UDP 프로토콜)
]

# ffmpeg 프로세스 시작
process = subprocess.Popen(ffmpeg_command, stdin=subprocess.PIPE)

# FPS 제한 설정
fps_limit = 15  # 최대 프레임 속도 제한 (FPS)
frame_interval = 1.0 / fps_limit  # 프레임 간격
start_time = time.time()

try:
    while cap.isOpened():
        frame_start_time = time.time()

        ret, frame = cap.read()
        if not ret:
            break

        # 프레임을 ffmpeg의 stdin으로 전달하여 인코딩 및 전송
        process.stdin.write(frame.tobytes())

        # 프레임 속도 제한 적용
        elapsed_time = time.time() - frame_start_time
        if elapsed_time < frame_interval:
            time.sleep(frame_interval - elapsed_time)

        # 송신 측 화면 출력 (테스트용)
        cv2.imshow('Camera Sender H.264 (ffmpeg)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    process.stdin.close()
    process.wait()
    cv2.destroyAllWindows()
