import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QLabel
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject
import sys
import json  # JSON 변환을 위한 모듈

class ConveyorController(Node, QObject):
    status_signal = pyqtSignal(str)  # GUI 업데이트를 위한 PyQt 시그널

    def __init__(self):
        QObject.__init__(self)  # QObject 초기화
        Node.__init__(self, 'conveyor_controller')  # Node 초기화
        self.get_logger().info("컨베이어 벨트 컨트롤 노드가 시작되었습니다!")

        # 퍼블리셔 (명령 전송)
        self.publisher = self.create_publisher(String, 'conveyor/control', 10)

        # 서브스크라이버 (상태 모니터링)
        self.subscription = self.create_subscription(
            String,
            'conveyor/status',
            self.status_callback,
            10)
    
    def send_command(self, control):
        """컨베이어 벨트에 JSON 형식의 명령을 전송"""
        command_dict = {
            "control": control,
            "distance.mm": 1000  # 고정된 값 (필요하면 변경 가능)
        }
        msg = String()
        msg.data = json.dumps(command_dict)  # JSON 문자열 변환
        self.publisher.publish(msg)
        self.get_logger().info(f"컨베이어 벨트 명령 전송: {msg.data}")

    def status_callback(self, msg):
        self.get_logger().info(f"컨베이어 상태 업데이트: {msg.data}")
        self.status_signal.emit(msg.data)  # GUI 업데이트 요청

class RosThread(QThread):
    """ROS2 스레드를 실행하는 별도 QThread"""
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

class ConveyorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Conveyor Controller")
        self.setGeometry(100, 100, 400, 200)

        # 현재 상태 표시 라벨 (초기값: INIT)
        self.status_label = QLabel("현재 상태: INIT")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold;")

        # ROS2 노드 초기화 및 별도 스레드 실행
        self.node = ConveyorController()
        self.ros_thread = RosThread(self.node)
        self.node.status_signal.connect(self.update_status)  # 상태 변경 시 호출
        self.ros_thread.start()  # ROS2 실행

        # 버튼 추가
        self.button_start = QPushButton("컨베이어 벨트 시작")
        self.button_stop = QPushButton("컨베이어 벨트 종료")
        self.button_red = QPushButton("빨간 물체")
        self.button_blue = QPushButton("파란 물체")
        self.button_half = QPushButton("반반")

        # 버튼 색상 설정
        self.button_red.setStyleSheet("background-color: red; color: white;")
        self.button_blue.setStyleSheet("background-color: blue; color: white;")
        self.button_half.setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 red, stop:0.49 red, stop:0.51 blue, stop:1 blue); color: white;")

        # 버튼 이벤트 연결 (JSON 명령 전송)
        self.button_start.clicked.connect(lambda: self.node.send_command("go"))  # "go" 명령 전송
        self.button_stop.clicked.connect(lambda: self.node.send_command("stop"))  # "stop" 명령 전송

        # 레이아웃 설정
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_start)
        button_layout.addWidget(self.button_stop)
        button_layout.addWidget(self.button_red)
        button_layout.addWidget(self.button_blue)
        button_layout.addWidget(self.button_half)

        layout = QVBoxLayout()
        layout.addWidget(self.status_label)  # 상태 표시 라벨 추가
        layout.addLayout(button_layout)
        self.setLayout(layout)

    def update_status(self, status):
        """컨베이어 상태를 GUI에 반영 (메인 스레드에서 실행)"""
        QTimer.singleShot(0, lambda: self.status_label.setText(f"현재 상태: {status}"))

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = ConveyorGUI()
    window.show()
    
    app.exec_()  # Qt 이벤트 루프 실행
    rclpy.shutdown()  # ROS 종료
if __name__ == '__main__':
    main()