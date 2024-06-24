import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ColorPublisher(Node):
    def __init__(self):
        super().__init__('color_publisher')
        self.subscription = self.create_subscription(
            Bool,
            'color_detected',
            self.listener_callback,
            10)
        self.subscription
    
    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('Color detected')
        else:
            self.get_logger().info('Color not detected')

def main(args=None):
    rclpy.init(args=args)
    color_publisher = ColorPublisher()
    rclpy.spin(color_publisher)
    color_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
