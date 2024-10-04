import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial  # 시리얼 통신을 위한 라이브러리 임포트

class RandomNumberSubscriber(Node):
    def __init__(self):
        super().__init__('random_number_subscriber')
        
        # 시리얼 포트 설정: 포트와 보드레이트를 지정
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 9600
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f'Serial connection established on {self.serial_port} at {self.baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            self.serial_conn = None

        # 서브스크라이버 생성: 'kg_test_ysh' 토픽을 구독
        self.subscription = self.create_subscription(
            Int64,
            'kg_test_ysh',
            self.listener_callback,
            10)
        self.subscription  # 방지용 변수 (unused variable warning)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')  # 받은 값 출력

        # 받은 값에 따라 LED 제어
        if self.serial_conn is not None:
            if msg.data % 2 == 0:
                # 짝수일 때: LED ON
                self.serial_conn.write(b'LED ON\n')
                self.get_logger().info('Sent: LED ON to Arduino')
            else:
                # 홀수일 때: LED OFF
                self.serial_conn.write(b'LED OFF\n')
                self.get_logger().info('Sent: LED OFF to Arduino')

    def destroy_node(self):
        # 노드 종료 시 시리얼 포트 닫기
        if self.serial_conn is not None:
            self.serial_conn.close()
            self.get_logger().info(f'Serial connection on {self.serial_port} closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomNumberSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Subscriber interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
