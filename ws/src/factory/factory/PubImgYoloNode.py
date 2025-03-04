import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# 카메라 내부 파라미터 및 왜곡 계수
K = np.array([[1.39755585e+03, 0.0, 320],
              [0.0, 1.39879236e+03, 240],
              [0.0, 0.0, 1.0]])
D = np.array([2.97613204e-02, 5.68877296e-01, 1.56553998e-03, -3.11259015e-05, -1.90935153e+00])

# 카메라 중심 및 초점 거리
C_x, C_y = K[0, 2], K[1, 2]
f_x, f_y = K[0, 0], K[1, 1]

# 객체의 실제 너비 (단위: mm)
OBJECT_REAL_WIDTH = 360  # 예제: 200mm (20cm)

# YOLO 모델 로드
MODEL_PATH = '/mnt/sda1/rokey_project/8week/2024-2_ROKEYBOOTCAMP_serving_robot_project_3/ws/src/factory/factory/detect_best.pt'
model = YOLO(MODEL_PATH)

# 클래스별 색상 매핑
CLASS_COLORS = {
    'Red': (0, 0, 255),
    'Blue': (255, 0, 0),
    'Basket': (0, 255, 0),
    'Aruco': (255, 255, 0)
}
m_Pixel = 0.000164

class PubImgYoloNode(Node):
    def __init__(self):
        super().__init__('pub_img_yolo_node')
        
        self.subscription = self.create_subscription(
            CompressedImage, 'image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 10)
        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 FPS
        self.last_image = None
    
    def image_callback(self, msg):
        """이미지 수신 콜백."""
        try:
            self.last_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().info("Received an image!")
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed image: {e}")
    
    def timer_callback(self):
        """타이머 콜백: YOLO 검출 및 퍼블리시."""
        if self.last_image is None:
            self.get_logger().info("No image received yet!")
            return

        frame = cv2.undistort(self.last_image, K, D)
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # 화면 중앙 십자선 추가
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)

        results = model(frame)
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                obj_center_x, obj_center_y = (x1 + x2) // 2, (y1 + y2) // 2
                box_width = x2 - x1
                confidence = box.conf[0].item()
                class_id = int(box.cls[0].item())
                class_name = model.names[class_id]
                label = f"{class_name}: {confidence:.2f}"
                
                # 클래스별 색상 적용
                box_color = CLASS_COLORS.get(class_name, (0, 255, 0))
                
                # 거리 계산
                Z = (OBJECT_REAL_WIDTH * f_x) / box_width if box_width > 0 else -1
                offset_x_mm = ((obj_center_x - C_x) * Z) / f_x
                offset_y_mm = ((obj_center_y - C_y) * Z) / f_y
                
                # 정보 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, box_color, 2)
                cv2.putText(frame, f"Offset: ({offset_x_mm:.1f} mm, {offset_y_mm:.1f} mm)", (x1, y2 + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame, f"Distance: {Z:.1f} mm", (x1, y2 + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.circle(frame, (obj_center_x, obj_center_y), 5, box_color, -1)
                
        frame = cv2.resize(frame, (640, 360))
        # 이미지 압축 및 퍼블리시
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        ret, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        if ret:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_image.tobytes()
            self.publisher_.publish(compressed_msg)
            self.get_logger().info("Published compressed image to 'yolo/compressed'!")


def main(args=None):
    rclpy.init(args=args)
    node = PubImgYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
