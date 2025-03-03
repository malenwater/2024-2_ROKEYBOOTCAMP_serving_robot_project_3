import rclpy
import json
import serial
import time
import threading
from rclpy.node import Node
from std_msgs.msg import String

# 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyACM0"  # 환경에 맞게 수정
BAUD_RATE = 115200

class ConveyorSerialController(Node):
    def __init__(self):
        super().__init__('conveyor_serial_controller')

        # 시리얼 포트 연결
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # 연결 안정화
            self.get_logger().info(f"Serial connected on {SERIAL_PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # 'conveyor/control' 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'conveyor/control',
            self.control_callback,
            10
        )

        # 데이터 수신 쓰레드 시작
        self.receive_thread = threading.Thread(target=self.read_serial_data)
        self.receive_thread.daemon = True  # 프로그램 종료 시 함께 종료
        self.receive_thread.start()

    def control_callback(self, msg):
        if not self.ser:
            self.get_logger().error("Serial port is not initialized!")
            return

        try:
            # JSON 파싱
            command_data = json.loads(msg.data)

            # "control" 필드 확인
            control_cmd = command_data.get("control", None)
            distance_mm = command_data.get("distance.mm", None)

            if control_cmd is None:
                self.get_logger().error("control 명령이 없습니다!")
                return

            self.get_logger().info(f"Received: control={control_cmd}, distance={distance_mm}")
            distance_mm = 10.5 * float(distance_mm)
            # JSON을 문자열로 변환 후 시리얼 전송
            serial_data = str(distance_mm) + "\n"
            self.ser.write(serial_data.encode())
            self.get_logger().info(f"Sent to Arduino: {serial_data.strip()}")

        except json.JSONDecodeError:
            self.get_logger().error("JSON 파싱 실패!")

    def read_serial_data(self):
        while rclpy.ok():  # rclpy가 활성화된 동안 계속 실행
            if self.ser and self.ser.in_waiting > 0:
                raw_data = self.ser.read_all()  # 시리얼 포트에서 데이터 읽기
                self.get_logger().info(f"Received from Arduino: {raw_data.decode().strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorSerialController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
