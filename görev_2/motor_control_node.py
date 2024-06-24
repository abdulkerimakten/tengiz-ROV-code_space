import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(String, 'line_position', self.command_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)  # Arduino'nun bağlı olduğu seri port

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'FORWARD':
            self.serial_port.write(b'F\n')
        elif command == 'TURN_LEFT':
            self.serial_port.write(b'L\n')
        elif command == 'TURN_RIGHT':
            self.serial_port.write(b'R\n')
        elif command == 'BACKWARD':
            self.serial_port.write(b'B\n')
        elif command == 'UP':
            self.serial_port.write(b'U\n')
        elif command == 'DOWN':
            self.serial_port.write(b'D\n')
        elif command == 'STOP':
            self.serial_port.write(b'S\n')
        else:
            self.get_logger().warn(f'Unknown command: {command}')

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
