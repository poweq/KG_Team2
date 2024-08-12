import serial

def main():
    try:
        # 시리얼 포트 설정 (예: '/dev/ttyAMA3')
        ser = serial.Serial('/dev/ttyAMA3', 9600, timeout=1)  # 포트와 보드레이트 설정
        print("Serial port opened successfully.")

        while True:
            try:
                if ser.in_waiting > 0:  # 데이터가 대기 중인지 확인
                    data = ser.readline().decode('ascii').rstrip()  # ASCII로 데이터 디코딩
                    print("Received:", data)  # 수신한 데이터 출력
            except serial.SerialException as e:
                print(f"Serial exception: {e}")
                break
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
    finally:
        ser.close()  # 프로그램 종료 시 시리얼 포트 닫기
        print("Serial port closed.")

if __name__ == "__main__":
    main()
