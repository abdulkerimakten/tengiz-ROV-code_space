import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(String, 'motor_commands', self.command_callback, 10)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        # Burada motor kontrol kodunu ekleyin
        if command == 'MOVE_LEFT':
            self.move_left()
        elif command == 'MOVE_RIGHT':
            self.move_right()
        elif command == 'FORWARD':
            self.move_forward()
        elif command == 'TURN_LEFT':
            self.turn_left()
        elif command == 'TURN_RIGHT':
            self.turn_right()
        elif command == 'STOP':
            self.stop()

    def move_left(self):
        self.get_logger().info('Moving Left')
        # Motor hareket kodları buraya eklenecek

    def move_right(self):
        self.get_logger().info('Moving Right')
        # Motor hareket kodları buraya eklenecek

    def move_forward(self):
        self.get_logger().info('Moving Forward')
        # Motor hareket kodları buraya eklenecek

    def turn_left(self):
        self.get_logger().info('Turning Left')
        # Motor hareket kodları buraya eklenecek

    def turn_right(self):
        self.get_logger().info('Turning Right')
        # Motor hareket kodları buraya eklenecek

    def stop(self):
        self.get_logger().info('Stopping')
        # Motor durdurma kodları buraya eklenecek

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
