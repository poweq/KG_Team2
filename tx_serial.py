import serial
import time

def main():
    try:
        # 시리얼 포트 설정 (예: '/dev/ttyAMA3')
        ser = serial.Serial('/dev/ttyAMA3', 115200, timeout=2)
        print("Serial port opened successfully.")

        while True:
            try:
                # "hello" 메시지 전송
                message = "hello\r\n"  # 데이터 끝에 개행 문자 추가
                ser.write(message.encode())  # "hello"를 시리얼 포트로 전송
                time.sleep(1)  # 1초 대기
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

