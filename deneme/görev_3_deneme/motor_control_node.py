import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(String, 'motor_commands', self.command_callback, 10)
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600)

    def command_callback(self, msg):
        command = msg.data
        self.arduino.write(command.encode())
        self.get_logger().info(f'Sent command to Arduino: {command}')

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
