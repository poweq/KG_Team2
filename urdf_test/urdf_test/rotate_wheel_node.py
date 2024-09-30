import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class RotateWheelNode(Node):
    def __init__(self):
        super().__init__('rotate_wheel_node')
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        self.joint_state = JointState()

        self.joint_name = ['wheel1_joint']
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.joint_state.name = self.joint_name
        self.angle = 0.0
        self.joint_state.position = [self.angle]


    def timer_callback(self):
        self.angle += 0.5
        if self.angle > 3.14:
            self.angle = -3.14

        self.joint_state.position[0] = self.angle

        self.publisher.publish(self.joint_state)
        self.get_logger().info(f'publishing : {self.angle * (180.0/3.14159)} degrees')


def main(args = None):
    rclpy.init(args=args)
    rotate_wheel_node = RotateWheelNode()
    rclpy.spin(rotate_wheel_node)
    rotate_wheel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

