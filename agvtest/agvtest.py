from myagv import MyAgv
import time
import threading
import sys
import tty
import termios

# 키보드 입력 감지 함수
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == "__main__":
    # MyAgv 인스턴스 생성
    agv = MyAgv(port="/dev/ttyAMA2", baudrate="115200", timeout=0.1, debug=True)

    try:
        print("명령을 입력하세요 (w: 전진, s: 후진, a: 좌회전, d: 우회전, u: 시계방향 회전, j: 반시계방향 회전, q: 종료): ")
        while True:
            command = get_key().lower()
            
            if command == 'w':
                print("앞으로 이동 중...")
                agv.go_ahead(speed=50)
            elif command == 's':
                print("후진 중...")
                agv.retreat(speed=50)
            elif command == 'a':
                print("좌회전 중...")
                agv.pan_left(speed=30)
            elif command == 'd':
                print("우회전 중...")
                agv.pan_right(speed=30)
            elif command == 'u':
                print("시계 방향으로 회전 중...")
                agv.clockwise_rotation(speed=30)
            elif command == 'j':
                print("반시계 방향으로 회전 중...")
                agv.counterclockwise_rotation(speed=30)
            elif command == 'q':
                print("프로그램을 종료합니다.")
                break
            else:
                print("잘못된 명령입니다. 다시 입력해주세요.")
            
            # 키를 떼면 멈추도록 설정
            agv.stop()
    
    except KeyboardInterrupt:
        # 강제 종료 시 정지
        print("\n강제 종료, 정지합니다.")
        agv.stop()
