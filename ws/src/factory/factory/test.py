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
        self.ser = None
        self.status = None
        self.connect_serial()

        # 상태를 발행할 Publisher 설정
        self.status_publisher = self.create_publisher(String, 'conveyor/status', 10)

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
        
    def connect_serial(self):
        """시리얼 포트 연결 및 연결 상태 체크"""
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # 연결 안정화
            self.get_logger().info(f"Serial connected on {SERIAL_PORT} at {BAUD_RATE} baud")
            self.status = "READY"  # 연결되면 READY 상태
        except serial.SerialException:
            self.ser = None
            self.status = "connection lost"  # 연결이 끊어지면 상태 'connection lost'
        
    def update_status(self):
        """상태를 'conveyor/status' 토픽에 발행"""
        status_msg = String()
        status_msg.data = self.status
        if self.status_publisher:  # 상태 발행을 시도할 때 publisher가 존재하는지 확인
            self.status_publisher.publish(status_msg)

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

            # 받은 명령에 따라 동작을 구분
            if control_cmd == "go":
                if distance_mm is not None:
                    self.get_logger().info(f"Received: control={control_cmd}, distance={distance_mm}")
                    distance_mm = 10.5 * float(distance_mm)
                    # JSON을 문자열로 변환 후 시리얼 전송
                    serial_data = str(distance_mm) + "\n"
                    self.ser.write(serial_data.encode())
                    self.get_logger().info(f"Sent to Arduino: {serial_data.strip()}")
                else:
                    self.get_logger().error("distance.mm 값이 없습니다!")

            elif control_cmd == "stop":
                self.get_logger().info("Received: control=stop")
                # Arduino로 "stop" 신호 전송 (예: stop 명령을 보내는 방식)
                serial_data = "1\n"
                self.ser.write(serial_data.encode())
                self.get_logger().info(f"Sent to Arduino: {serial_data.strip()}")
                
        except json.JSONDecodeError:
            self.get_logger().error("JSON 파싱 실패!")

    def read_serial_data(self):
        previous_status = self.status  # 초기 상태 기록
        while rclpy.ok():  # rclpy가 활성화된 동안 계속 실행
            if self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        raw_data = self.ser.read_all()  # 시리얼 포트에서 데이터 읽기
                        decoded_data = raw_data.decode().strip()

                        # 받은 데이터에 따라 상태 변경
                        if 's' in decoded_data:
                            new_status = "INIT"
                        elif '.' in decoded_data:
                            new_status = "READY"
                        elif '_' in decoded_data:
                            new_status = "RUN"
                        else:
                            new_status = self.status  # 상태가 변경되지 않으면 기존 상태 유지

                        # 상태가 바뀐 경우에만 로그 출력
                        if new_status != previous_status:
                            self.status = new_status
                            self.get_logger().info(f"Received from Arduino: {decoded_data}, Current Status: {self.status}")
                            previous_status = self.status  # 상태 업데이트

                            # 상태를 'conveyor/status' 토픽에 발행
                            self.update_status()

                except serial.SerialException:
                    self.get_logger().error("Serial connection lost, reconnecting...")
                    self.ser = None
                    self.status = "connection lost"
                    self.update_status()
                    self.connect_serial()  # 다시 연결 시도
                    time.sleep(2)

            else:
                time.sleep(2)  # 시리얼 포트가 없으면 대기


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