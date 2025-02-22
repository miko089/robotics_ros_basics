import rclpy
from rclpy.node import Node

from math import sin, cos, pi
from interfaces.msg import RobotPosition


class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(RobotPosition, 'pos', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cur_angle = 0

    def timer_callback(self):
        msg = RobotPosition()
        msg.position_x = cos(self.cur_angle)
        msg.position_y = sin(self.cur_angle)
        msg.timestamp = float(self.get_clock().now().seconds_nanoseconds()[0]+self.get_clock().now().seconds_nanoseconds()[1]/1e9)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.position_x:.1f}, \
                               {msg.position_y:.1f} at {msg.timestamp:.1f}')
        self.cur_angle += 0.2
        if self.cur_angle > 2 * pi:
            self.cur_angle -= 2 * pi


def main(args=None):
    rclpy.init(args=args)

    pos_publisher = PositionPublisher()

    rclpy.spin(pos_publisher)

    pos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

