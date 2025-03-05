import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.publisher = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        self.bridge = CvBridge()

        # 카메라 초기화
        self.cap = cv2.VideoCapture(2, cv2.CAP_V4L2)  # V4L2 사용
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            exit()

        # 카메라 속성 설정
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 카메라 내부 파라미터 (K) 및 왜곡 계수 (D)
        self.K = np.array([[1.39755585e+03, 0.000, 740],
                           [0.000, 1.39879236e+03, 360],
                           [0.000, 0.000, 1.000]])
        self.D = np.array([2.97613204e-02, 5.68877296e-01, 1.56553998e-03, -3.11259015e-05, -1.90935153e+00])

        self.f_x, self.f_y = self.K[0, 0], self.K[1, 1]
        self.C_x = 635  # 기존 740에서 조정
        self.C_y = self.K[1, 2]  # Y축은 그대로 사용

        self.OBJECT_REAL_WIDTH = 360  # mm

        # YOLO 모델 로드
        model_path = "/home/jsy/week8/day2/detect_best.pt"
        self.model = YOLO(model_path)

        self.timer = self.create_timer(0.2, self.process_frame)  # 5 FPS

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("카메라 프레임을 가져올 수 없습니다.")
            return

        # 왜곡 보정 적용 (원본 크기 유지)
        frame = cv2.undistort(frame, self.K, self.D)

        # YOLO 객체 감지
        results = self.model(frame)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                box_width = x2 - x1
                box_height = y2 - y1
                
                confidence = box.conf[0].item()
                class_id = int(box.cls[0].item())
                label = f"{self.model.names[class_id]}: {confidence:.2f}"
                
                # 거리 및 오프셋 계산 (mm 단위)
                Z = (self.OBJECT_REAL_WIDTH * self.f_x) / (box_width * 1.21) if box_width > 0 else -1
                offset_x_mm = (((x1 + x2) // 2 - self.C_x) * Z) / self.f_x
                offset_y_mm = (((y1 + y2) // 2 - self.C_y) * Z) / self.f_y
                
                # 바운딩 박스 및 정보 출력
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Offset: ({offset_x_mm:.1f} mm, {offset_y_mm:.1f} mm)", (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), 5, (255, 0, 0), -1)

        # **라벨링 완료 후 640x360으로 리사이즈**
        resized_frame = cv2.resize(frame, (640, 360))

        # **화면 중앙에 십자선 추가**
        center_x = resized_frame.shape[1] // 2
        center_y = resized_frame.shape[0] // 2

        # 가로선 (중앙 가로로 길게)
        cv2.line(resized_frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 1)
        # 세로선 (중앙 세로로 길게)
        cv2.line(resized_frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 1)

        # 이미지 압축 후 ROS2 메시지로 변환하여 발행
        _, encoded_frame = cv2.imencode('.jpg', resized_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded_frame.tobytes()
        self.publisher.publish(msg)
        self.get_logger().info("이미지 프레임 발행 완료.")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()