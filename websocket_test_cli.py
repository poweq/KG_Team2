import socket
import time

# 서버 IP와 포트 설정
HOST = '172.30.1.100'  # 라즈베리 파이의 IP 주소로 수정하세요
PORT = 65432           # 라즈베리 파이 서버에서 설정한 포트 번호

# 초기 숫자 설정
value_to_send = 0

# 소켓 생성
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

try:
    while True:
        # 정수를 바이트로 변환하여 송신
        client_socket.send(value_to_send.to_bytes(4, byteorder='big'))
        print(f"송신한 값: {value_to_send}")
        value_to_send += 1
        
        if value_to_send == 180:
            value_to_send == 1
          # 송신한 값 출력
        
        time.sleep(0.1)  # 1초 대기

except KeyboardInterrupt:
    print("클라이언트 종료 중...")
finally:
    # 소켓 종료
    client_socket.close()
