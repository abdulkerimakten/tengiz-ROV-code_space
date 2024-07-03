import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        height, width = binary.shape
        crop_img = binary[int(height/2):height, 0:width]
        contours, _ = cv2.findContours(crop_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        command = "STOP"

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, int(cy + height/2)), 10, (0, 255, 0), -1)
                error = cx - width // 2
                if abs(error) > 100:
                    if error > 100:
                        command = 'MOVE_LEFT'
                    elif error < -100:
                        command = 'MOVE_RIGHT'
                else:
                    command = 'FORWARD'
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                angle = rect[-1]
                if angle < -45:
                    angle += 90
                if abs(angle) > 45:
                    if angle > 0:
                        command = 'TURN_RIGHT'
                    else:
                        command = 'TURN_LEFT'
        else:
            command = 'STOP'

        self.publisher_.publish(String(data=command))
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollowerNode()
    rclpy.spin(line_follower_node)
    line_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
