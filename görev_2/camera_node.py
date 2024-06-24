import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.cap = cv2.VideoCapture(0)  # 0, ilk USB kamerayı temsil eder

        if not self.cap.isOpened():
            self.get_logger().error("Kamera açılamadı!")
            rclpy.shutdown()
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV görüntüsünü ROS Image mesajına dönüştür
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            # Mesajı yayınla
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().error("Görüntü alınamadı!")

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.cap.release()  # Kamera kaynağını serbest bırak
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
