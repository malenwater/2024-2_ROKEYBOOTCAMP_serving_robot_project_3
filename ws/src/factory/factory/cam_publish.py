import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.05, self.publish_frame)  # 10Hz (0.1초마다 실행)
        self.cap = cv2.VideoCapture(0)  # 기본 카메라(웹캠) 사용

        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다! USB 카메라 또는 웹캠을 확인하세요.")
        else:
            self.get_logger().info("카메라 퍼블리셔가 시작되었습니다!")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info("카메라 프레임 전송 중...")
        else:
            self.get_logger().warning("카메라에서 프레임을 가져올 수 없습니다.")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("카메라 퍼블리셔 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

