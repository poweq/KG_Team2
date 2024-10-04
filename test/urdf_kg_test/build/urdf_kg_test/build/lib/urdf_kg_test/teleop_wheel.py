import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import select
import termios
import tty
import threading

# 바퀴 조인트 이름 설정
LEFT_WHEEL_JOINT = 'left_wheel_joint'
RIGHT_WHEEL_JOINT = 'right_wheel_joint'

# 키보드 입력과 조인트 변경 매핑
move_bindings = {
    'w': (1.0, 1.0),  # 앞으로 이동
    's': (-1.0, -1.0),  # 뒤로 이동
    'a': (0.5, -0.5),  # 좌회전
    'd': (-0.5, 0.5),  # 우회전
    'x': (0.0, 0.0)  # 정지
}

# 기본 조인트 속도 설정
joint_speed = 0.5

class TeleopWheel(Node):
    def __init__(self):
        super().__init__('teleop_wheel')
        # 조인트 상태 퍼블리셔 생성
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.1, self.publish_joint_state)  # 0.1초마다 상태 퍼블리시

        # 터미널 초기화
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_velocity = {LEFT_WHEEL_JOINT: 0.0, RIGHT_WHEEL_JOINT: 0.0}

        # 키보드 입력 스레드 시작
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop)
        self.keyboard_thread.start()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_joint_state(self):
        # 조인트 상태 메시지 생성
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT]
        joint_state.position = [0.0, 0.0]
        joint_state.velocity = [self.current_velocity[LEFT_WHEEL_JOINT], self.current_velocity[RIGHT_WHEEL_JOINT]]

        # 퍼블리시
        self.joint_pub.publish(joint_state)

    def keyboard_input_loop(self):
        try:
            print("Use 'w' to move forward, 's' to move backward")
            print("Use 'a' to turn left, 'd' to turn right")
            print("Press 'x' to stop. Press 'Ctrl+C' to exit.")
            while True:
                key = self.get_key()

                # 키 입력에 따른 속도 업데이트
                if key in move_bindings.keys():
                    velocities = move_bindings[key]
                    self.current_velocity[LEFT_WHEEL_JOINT] = velocities[0] * joint_speed
                    self.current_velocity[RIGHT_WHEEL_JOINT] = velocities[1] * joint_speed
                    self.get_logger().info(f"Updated velocities: {self.current_velocity}")

                elif key == '\x03':  # Ctrl+C to exit
                    break

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_wheel = TeleopWheel()

    try:
        # ROS 2 노드를 별도로 스핀하며 실행
        rclpy.spin(teleop_wheel)
    except KeyboardInterrupt:
        print("Teleop interrupted.")
    finally:
        teleop_wheel.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
