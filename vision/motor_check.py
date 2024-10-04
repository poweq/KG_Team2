import serial

serial_port = serial.Serial('/dev/ttyAMA1', 9600)  # 보드레이트는 아두이노와 동일하게 설정

def send_command(command):
    serial_port.write(f"{command}\n".encode())  # 명령어를 시리얼 포트를 통해 전송
    print(f"Command sent: {command}")

if __name__ == "__main__":
    while True:
        try:
            command = input("Enter motor command (e.g. W:100): ")  # 예시: W:100
            send_command(command)
        except KeyboardInterrupt:
            print("Exiting...")
            serial_port.close()  # 프로그램 종료 시 시리얼 포트를 닫습니다.
            break
        except Exception as e:
            print(f"Error: {e}")  # 오류 처리

