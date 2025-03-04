import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO

# 카메라 내부 파라미터 (K) 및 왜곡 계수 (D)
K = np.array([[1.39755585e+03, 0.00000000e+00, 7.29335091e+02],
              [0.00000000e+00, 1.39879236e+03, 4.19580562e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
D = np.array([2.97613204e-02, 5.68877296e-01, 1.56553998e-03, -3.11259015e-05, -1.90935153e+00])

# 카메라 내부 파라미터에서 중심 좌표 및 초점 거리 가져오기
C_x, C_y = K[0, 2], K[1, 2]
f_x, f_y = K[0, 0], K[1, 1]

# 객체의 실제 너비 (단위: mm) - 사전에 측정된 값
OBJECT_REAL_WIDTH = 200  # 예제: 200mm (20cm)

# YOLO 모델 로드
model_path = '/mnt/sda1/rokey_project/8week/2024-2_ROKEYBOOTCAMP_serving_robot_project_3/ws/src/factory/factory/detect_best.pt'  # 모델 파일 경로
model = YOLO(model_path)  # 모델 로드

class PubImgYoloNode(Node):
    def __init__(self):
        super().__init__('pub_img_yolo_node')

        # 서브스크라이버 선언
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10
        )

        # 퍼블리셔 선언
        self.publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 10)

        # CV_bridge 객체 생성
        self.bridge = CvBridge()

        # 타이머를 사용하여 주기적으로 이미지를 퍼블리시 (5 FPS)
        self.timer = self.create_timer(1/5.0, self.timer_callback)  # 5 FPS

        # 이미지 처리 결과를 저장하는 변수
        self.last_image = None

    def image_callback(self, msg):
        """수신한 이미지를 콜백하여 처리."""
        try:
            # 수신한 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_image = cv_image
            self.get_logger().info("Received an image!")
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed image: {e}")

    def timer_callback(self):
        """타이머 콜백에서 이미지를 퍼블리시."""
        if self.last_image is not None:
            frame = cv2.undistort(self.last_image, K, D)
            
            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2  # 화면 중앙 좌표
            
            # 화면 중앙에 십자선 추가
            cv2.line(frame, (center_x - 160, center_y), (center_x + 160, center_y), (0, 0, 255), 2)
            cv2.line(frame, (center_x, center_y - 160), (center_x, center_y + 160), (0, 0, 255), 2)
            results = model(frame)
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    obj_center_x = (x1 + x2) // 2
                    obj_center_y = (y1 + y2) // 2
                    box_width = x2 - x1  # 객체 바운딩 박스 너비 (픽셀 단위)
                    confidence = box.conf[0].item()
                    class_id = int(box.cls[0].item())
                    label = f"{model.names[class_id]}: {confidence:.2f}"

                    # 객체와의 거리(Z) 계산
                    if box_width > 0:  # 감지된 객체가 있어야 계산 가능
                        Z = (OBJECT_REAL_WIDTH * f_x) / box_width  # 거리(mm)
                    else:
                        Z = -1  # 거리 계산 불가능 (바운딩 박스 오류)

                    # 픽셀 단위를 실제 mm로 변환
                    offset_x_mm = ((obj_center_x - C_x) * Z) / f_x
                    offset_y_mm = ((obj_center_y - C_y) * Z) / f_y
                    
                    offset_text = f"Offset: ({offset_x_mm:.1f} mm, {offset_y_mm:.1f} mm)"
                    distance_text = f"Distance: {Z:.1f} mm"
                    
                    # 바운딩 박스 및 정보 출력
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, offset_text, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(frame, distance_text, (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    # 객체 중심점 표시
                    cv2.circle(frame, (obj_center_x, obj_center_y), 5, (255, 0, 0), -1)
                
            # 이미지를 640x360으로 리사이즈
            frame = cv2.resize(frame, (640, 360))
            # 이미지 압축 (JPEG 품질 30)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            ret, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if ret:
                # 인코딩된 이미지를 CompressedImage 메시지로 변환
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = self.get_clock().now().to_msg()
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_image.tobytes()

                # 이미지 퍼블리시
                self.publisher_.publish(compressed_msg)
                self.get_logger().info("Published compressed image to 'yolo/compressed'!")
        else:
            self.get_logger().info("No image received yet!")

def main(args=None):
    rclpy.init(args=args)
    node = PubImgYoloNode()
    rclpy.spin(node)  # 노드를 계속 실행시킴

    # 종료 처리
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
