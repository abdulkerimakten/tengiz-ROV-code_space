import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(String, 'motor_commands', self.command_callback, 10)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Motor command received: {command}')
        # Here, add code to interface with the motor controllers
        # For example, sending PWM signals or commands to an ESC

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
