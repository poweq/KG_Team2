import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import random

class RandomNumberPublisher(Node):
    def __init__(self):
        super().__init__('random_number_publisher')
        
        # 퍼블리셔 생성: 토픽 이름은 'kg_test_ysh'
        self.publisher_ = self.create_publisher(Int64, 'kg_test_ysh', 10)
        self.timer = self.create_timer(1.0, self.publish_random_number)  # 1초마다 실행되는 타이머 설정
        self.get_logger().info('Random number publisher has been started.')

    def publish_random_number(self):
        msg = Int64()
        msg.data = random.randint(1, 10)  # 1에서 10 사이의 랜덤한 숫자 생성
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RandomNumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
