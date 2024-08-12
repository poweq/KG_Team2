import serial

# 시리얼 포트 설정 (예: '/dev/ttyAMA3')
ser = serial.Serial('/dev/ttyAMA3', 115200, timeout=1)  # 포트와 보드레이트 설정

try:
    while True:
        if ser.in_waiting > 0:  # 데이터가 대기 중인지 확인
            data = ser.readline().decode('utf-8').rstrip()  # 데이터 읽기
            print("Received:", data)  # 수신한 데이터 출력
except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    ser.close()  # 프로그램 종료 시 시리얼 포트 닫기
