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
        self.publisher_ = self.create_publisher(String, 'line_position', 10)
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.lost_line_count = 0  # Çizginin kaybolduğu adım sayacı

    def image_callback(self, msg):
        # ROS Image mesajını OpenCV formatına çevir
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Görüntüyü gri tonlamalıya çevir
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Siyah şeridi tespit etmek için görüntüyü eşikleme
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Görüntünün alt kısmını kırpma (yolun belirli bir kısmını incelemek için)
        height, width = binary.shape
        crop_img = binary[int(height/2):height, 0:width]
        
        # Kontur bulma
        contours, _ = cv2.findContours(crop_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # En büyük konturu seç
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M['m00'] > 0:
                # Konturun merkezi
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Görüntü merkezine göre hata
                error = cx - width // 2
                
                # Motor komutlarını belirleme
                if error > 50:  # 50 pikselden fazla sağa kayma
                    command = 'TURN_RIGHT'
                elif error < -50:  # 50 pikselden fazla sola kayma
                    command = 'TURN_LEFT'
                else:
                    command = 'FORWARD'
                
                # Çizgiyi bulduysak sayaç sıfırlanır
                self.lost_line_count = 0
            else:
                command = 'STOP'
        else:
            # Eğer çizgi kaybolduysa, son komutu bir süre daha devam ettir
            if self.lost_line_count < 5:
                command = 'STOP'
                self.lost_line_count += 1
            else:
                command = 'STOP'
                self.lost_line_count = 0
        
        # Komutu yayınla
        self.publisher_.publish(String(data=command))

def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollowerNode()
    rclpy.spin(line_follower_node)
    line_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
