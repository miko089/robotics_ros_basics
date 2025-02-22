import rclpy
from rclpy.node import Node

from interfaces.msg import RobotVelocity

class VelocityMonitor(Node):
    def __init__(self):
        super().__init__('velocity_monitor')
        self.subscription = self.create_subscription(
            RobotVelocity,
            'vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def listener_callback(self, msg):
        self.get_logger().info(
            f'I heard: {msg.velocity_x:.1f} on x, \
                {msg.velocity_y:.1f} on y with speed \
                    {msg.speed:.1f} at {msg.timestamp:.1f}')
        

def main(args=None):
    rclpy.init(args=args)

    client = VelocityMonitor()

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
