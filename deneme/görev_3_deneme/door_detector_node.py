import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
import random

class DoorDetectorNode(Node):
    def __init__(self):
        super().__init__('door_detector_node')
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.distance_subscription = self.create_subscription(Range, 'distance_sensor', self.distance_callback, 10)
        self.bridge = CvBridge()
        self.exploration_step = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.forward_time = 5  # default forward time in seconds
        self.turn_time = 2  # default turn time in seconds
        self.current_command = "STOP"
        self.obstacle_detected = False
        self.door_detected = False
        self.distance_to_door = float('inf')
        self.color_detect_start = False
        self.target_color_lower = np.array([110, 50, 50])  # Example color range (blue) in HSV
        self.target_color_upper = np.array([130, 255, 255])  # Example color range (blue) in HSV

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        height, width = binary.shape
        crop_img = binary[int(height/2):height, 0:width]
        contours, _ = cv2.findContours(crop_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                        self.current_command = 'MOVE_LEFT'
                    elif error < -100:
                        self.current_command = 'MOVE_RIGHT'
                else:
                    self.current_command = 'FORWARD'
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                angle = rect[-1]
                if angle < -45:
                    angle += 90
                if abs(angle) > 45:
                    if angle > 0:
                        self.current_command = 'TURN_RIGHT'
                    else:
                        self.current_command = 'TURN_LEFT'
                self.door_detected = True  # Kapı tespit edildiğinde değişkeni güncelle
            self.exploration_step = 0  # Reset exploration step if door detected
        else:
            self.current_command = "STOP"

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def distance_callback(self, msg):
        self.obstacle_detected = msg.range < 0.5  # Assuming 0.5 meters as the threshold
        if self.door_detected:
            self.distance_to_door = msg.range

    def timer_callback(self):
        if not self.door_detected:  # Only explore if door is not detected
            if self.exploration_step == 0:  # Initial descent and exploration
                self.current_command = "FORWARD"
                self.get_logger().info('Initial descent and exploration')
                self.exploration_step += 1
            elif self.exploration_step == 1:  # Check for doors or obstacles
                if not self.obstacle_detected:
                    self.current_command = "FORWARD"
                    self.get_logger().info('No obstacle detected, moving forward')
                else:
                    self.current_command = "STOP"
                    self.get_logger().info('Obstacle detected, stopping')
                    self.exploration_step += 1
            elif self.exploration_step == 2:  # Random turn if no door is found
                self.current_command = random.choice(["TURN_RIGHT", "TURN_LEFT"])
                self.get_logger().info('Random turn, current command: ' + self.current_command)
                self.exploration_step += 1
            elif self.exploration_step == 3:  # Move forward after turn
                if not self.obstacle_detected:
                    self.current_command = "FORWARD"
                    self.get_logger().info('No obstacle detected, moving forward after turn')
                else:
                    self.current_command = "STOP"
                    self.get_logger().info('Obstacle detected, stopping after turn')
                self.exploration_step = 1  # Loop back to check for doors or obstacles
        else:
            # Door detected, proceed towards the door
            if 0.45 < self.distance_to_door < 0.55:  # 45 cm to 55 cm range
                self.color_detect_start = True
            if self.color_detect_start:
                self.color_detection_callback()
            elif self.distance_to_door > 0.5:  # 50 cm threshold
                self.current_command = "FORWARD"
                self.get_logger().info('Door detected, moving towards the door')
            else:
                self.current_command = "STOP"
                self.get_logger().info('Close to the door, stopping')

        self.publisher_.publish(String(data=self.current_command))

    def color_detection_callback(self):
        frame = self.bridge.imgmsg_to_cv2(self.last_image_msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                error = cx - frame.shape[1] // 2
                if abs(error) > 50:
                    if error > 50:
                        self.current_command = 'MOVE_LEFT'
                    elif error < -50:
                        self.current_command = 'MOVE_RIGHT'
                else:
                    self.current_command = 'FORWARD'
                self.get_logger().info('Target color detected, aligning and moving towards the door')
                self.publisher_.publish(String(data=self.current_command))
                
                # İleri doğru hareket ederken engel tespiti yap
                if self.distance_to_door < 0.15:  # 15 cm threshold
                    self.current_command = "STOP"
                    self.get_logger().info('Reached the door, stopping')
                    self.publisher_.publish(String(data=self.current_command))
                    self.rclpy.shutdown()  # Görev tamamlandıktan sonra durdur
            else:
                self.current_command = 'STOP'
                self.get_logger().info('Target color not detected, stopping')
                self.publisher_.publish(String(data=self.current_command))
        else:
            self.current_command = 'STOP'
            self.get_logger().info('Target color not detected, stopping')
            self.publisher_.publish(String(data=self.current_command))

def main(args=None):
    rclpy.init(args=args)
    door_detector_node = DoorDetectorNode()
    rclpy.spin(door_detector_node)
    door_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
