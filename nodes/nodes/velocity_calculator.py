import rclpy
from rclpy.node import Node

from interfaces.msg import RobotPosition, RobotVelocity

class VelocityCalculator(Node):

    def __init__(self):
        super().__init__('velocity_calculator')
        self.subscription = self.create_subscription(
            RobotPosition,
            'pos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(RobotVelocity, 'vel', 10)
        self.last_pos = None
        self.last_timestamp = None

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.position_x}, {msg.position_y} at {msg.timestamp}')
        if self.last_pos is not None:
            vel = RobotVelocity()
            vel.velocity_x = (msg.position_x - self.last_pos.position_x) / (msg.timestamp - self.last_timestamp)
            vel.velocity_y = (msg.position_y - self.last_pos.position_y) / (msg.timestamp - self.last_timestamp)
            vel.speed = (vel.velocity_x ** 2 + vel.velocity_y ** 2) ** 0.5
            vel.timestamp = msg.timestamp
            self.publisher_.publish(vel)
            self.get_logger().info(f'Publishing: {vel.velocity_x:.1f}, \
                                   {vel.velocity_y:.1f} at \
                                    {vel.timestamp:.1f} with speed {vel.speed:.1f}')
        else:
            self.get_logger().info('No previous position to calculate velocity')
        self.last_pos = msg
        self.last_timestamp = msg.timestamp



def main(args=None):
    rclpy.init(args=args)

    client = VelocityCalculator()

    rclpy.spin(client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()