import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.publisher_ = self.create_publisher(Bool, 'color_detected', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(1)
        self.color_to_detect = [0, 255, 255]  # HSV color for detection (Yellow in this case)
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera')
            return
        
        # Process the frame to detect color
        blur = cv2.blur(frame, (3, 3))
        hsv_img = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower_limit, upper_limit = self.get_limits(self.color_to_detect)
        mask = cv2.inRange(hsv_img, lower_limit, upper_limit)
        bbox = self.get_bounding_box(mask)
        
        detected = bbox is not None
        self.publisher_.publish(Bool(data=detected))
        
        if detected:
            x1, y1, x2, y2 = bbox
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(frame, 'DETECTED', (x1+10, y2+30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
    
    def get_limits(self, color):
        # You should replace this with the actual function that computes limits
        lower_limit = np.array([color[0] - 10, 100, 100])
        upper_limit = np.array([color[0] + 10, 255, 255])
        return lower_limit, upper_limit
    
    def get_bounding_box(self, mask):
        mask_ = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_ = cv2.cvtColor(mask_, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(mask_, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x, y, w, h = cv2.boundingRect(contours[0])
            return x, y, x + w, y + h
        return None

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
