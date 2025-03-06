import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import String

class PubImgYoloNode(Node):
    def __init__(self):
        super().__init__('pub_img_yolo_node')
        
        # 카메라 내부 파라미터 및 왜곡 계수
        self.K = np.array([[1.39755585e+03, 0.0, 740],
                           [0.0, 1.39879236e+03, 360],
                           [0.0, 0.0, 1.0]])
        self.D = np.array([2.97613204e-02, 5.68877296e-01, 1.56553998e-03, -3.11259015e-05, -1.90935153e+00])

        # 카메라 중심 및 초점 거리
        self.C_x, self.C_y = self.K[0, 2], self.K[1, 2]
        self.f_x, self.f_y = self.K[0, 0], self.K[1, 1]
        self.C_x = 638
        # 객체의 실제 너비 (단위: mm)
        self.OBJECT_REAL_WIDTH = 360  # 예제: 200mm (20cm)

        # YOLO 모델 로드
        self.MODEL_PATH = '/mnt/sda1/rokey_project/8week/2024-2_ROKEYBOOTCAMP_serving_robot_project_3/ws/src/factory/factory/detect_best.pt'
        self.model = YOLO(self.MODEL_PATH)

        # 클래스별 색상 매핑
        self.CLASS_COLORS = {
            'Red': (0, 0, 255),
            'Blue': (255, 0, 0),
            'Basket': (0, 255, 0),
            'Aruco': (255, 255, 0)
        }

        self.m_Pixel = 0.000164
        self.SENTER_W = 640
        self.SENTER_H = 360
        
        # ROS2 관련 초기화
        self.subscription = self.create_subscription(
            CompressedImage, 'image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 10)
        self.detected_info_pub = self.create_publisher(String, 'yolo/detected_info', 10)
        
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

        frame = cv2.undistort(self.last_image, self.K, self.D)
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # 화면 중앙 십자선 추가
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)

        results = self.model(frame)
        
        detected_info = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                obj_center_x, obj_center_y = (x1 + x2) // 2, (y1 + y2) // 2
                box_width = x2 - x1
                confidence = box.conf[0].item()
                class_id = int(box.cls[0].item())
                class_name = self.model.names[class_id]

                # 클래스별 색상 적용
                box_color = self.CLASS_COLORS.get(class_name, (0, 255, 0))
                
                # 거리 및 오프셋 계산 (mm 단위)
                Z = (self.OBJECT_REAL_WIDTH * self.f_x) / (box_width * 1.21) if box_width > 0 else -1
                offset_x_mm = (((x1 + x2) // 2 - self.C_x) * Z) / self.f_x
                offset_y_mm = (((y1 + y2) // 2 - self.C_y) * Z) / self.f_y
                
                IMG_offset_x_mm = (((x1 + x2) // 2 - self.C_x) * Z) / self.f_x
                IMG_offset_y_mm = (((y1 + y2) // 2 - self.C_y) * Z) / self.f_y
                
                # 정보 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(frame, f"{class_name}: {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, box_color, 2)
                cv2.putText(frame, f"Offset: ({offset_x_mm:.1f} mm, {offset_y_mm:.1f} mm, {IMG_offset_x_mm:.1f}, {IMG_offset_y_mm:.1f})", (x1, y2 + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                cv2.putText(frame, f"Distance: {Z:.1f} mm", (x1, y2 + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.circle(frame, (obj_center_x, obj_center_y), 5, box_color, -1)
                
                # ArUco 클래스는 제외
                if class_name == 'Aruco':
                    continue
                
                # 감지된 정보 리스트에 추가
                detected_info.append([class_id, (obj_center_x - self.SENTER_W) * self.m_Pixel, (obj_center_y - self.SENTER_H) * self.m_Pixel])

        # 감지된 정보 퍼블리시
        if detected_info:
            detected_info_str = str(detected_info)
            msg = String()
            msg.data = detected_info_str
            self.detected_info_pub.publish(msg)
            self.get_logger().info(f"Published detected info: {detected_info_str}")

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
