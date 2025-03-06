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

        # 상태를 발행할 Publisher 설정
        self.ser = None
        self.status_publisher = self.create_publisher(String, 'conveyor/status', 10)

        # 'conveyor/control' 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'conveyor/control',
            self.control_callback,
            10
        )
        self.status = None
        # 데이터 수신 쓰레드 시작
        self.receive_thread = threading.Thread(target=self.check_serial)
        self.receive_thread.daemon = True  # 프로그램 종료 시 함께 종료
        self.receive_thread.start()
        
    def control_callback(self, msg):
        if not self.ser or not self.ser.is_open:  # 시리얼 포트가 열려 있지 않으면
            self.get_logger().error("Serial port is not initialized or not open!")
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
            
    # 시리얼 포트 열기
    def open_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # 연결 안정화
            self.get_logger().info(f"Serial connected on {SERIAL_PORT} at {BAUD_RATE} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None
    
    def check_serial(self):
        self.open_serial()
        if self.ser:
            # 수신 스레드 실행
            thread = threading.Thread(target=self.read_serial_data, daemon=True)
            thread.start()
        try:
            while True:
                if self.ser is None or not self.ser.is_open:  # 연결이 끊어졌으면
                    self.get_logger().error(f"⚠️ 연결이 끊어졌습니다. USB를 다시 꽂아주세요.")
                    while self.ser is None or not self.ser.is_open:
                        time.sleep(1)  # 잠시 대기 후 재시도
                        self.open_serial()  # 시리얼 포트를 다시 열기
                        if self.ser:
                            print("🚀 다시 연결되었습니다.")
                            thread = threading.Thread(target=self.read_serial_data, daemon=True)
                            thread.start()
                if thread is not None:
                    thread.join()  # 쓰레드가 종료될 때까지 기다림
                    self.get_logger().error(f"수신 스레드가 종료되었습니다.")
                time.sleep(1)  # 잠시 대기 후 재시도
        except KeyboardInterrupt:
            self.get_logger().error(f"\n프로그램 종료")
        finally:
            if self.ser:
                self.ser.close()  # 시리얼 포트 닫기
                
    def read_serial_data(self):
        previous_status = self.status  # 초기 상태 기록
        while rclpy.ok():  # rclpy가 활성화된 동안 계속 실행
            try: 
                if self.ser and self.ser.in_waiting > 0:
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
                        status_msg = String()
                        status_msg.data = self.status
                        self.status_publisher.publish(status_msg)

                        previous_status = self.status  # 상태 업데이트
            except (OSError, serial.SerialException) as e:
                self.get_logger().info(f"serial port error : {e}")
                status_msg = String()
                status_msg.data = "DISCONNECT"
                self.status_publisher.publish(status_msg)
                self.status = None
                if self.ser and self.ser.is_open:
                    self.ser.close()  # 포트 닫기
                    self.get_logger().info("시리얼 포트가 닫혔습니다.")
                self.ser = None
                return  # 에러가 나면 종료하여 다시 연결하도록 처리
                    

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
