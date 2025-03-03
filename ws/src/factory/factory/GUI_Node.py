import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PyQt5.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QLabel, QComboBox, QTextEdit
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QPixmap, QImage
import sys
import json
from datetime import datetime
import socket
import os
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins" #적절한 경로로 수정 필요
os.environ["QT_QPA_PLATFORM"] = "xcb" #기본 플랫폼 설정(본인이 사용하려는 플랫폼으로 설정)

class ConveyorController(Node, QObject):
    status_signal = pyqtSignal(str)
    image_signal = pyqtSignal(object)

    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'conveyor_controller')
        self.get_logger().info("컨베이어 벨트 컨트롤 노드가 시작되었습니다!")

        self.publisher = self.create_publisher(String, 'conveyor/control', 10)
        self.selection_publisher = self.create_publisher(String, 'conveyor/selection', 10)
        self.command_publisher = self.create_publisher(String, 'gui/command', 10)  # 새로운 퍼블리셔 추가
        self.subscription = self.create_subscription(String, 'conveyor/status', self.status_callback, 10)

        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
    
    def send_command(self, control):
        command_dict = {
            "control": control,
            "distance.mm": 1000
        }
        msg = String()
        msg.data = json.dumps(command_dict)
        self.publisher.publish(msg)
        self.get_logger().info(f"컨베이어 벨트 명령 전송: {msg.data}")
    
    def send_selection(self, selection):
        msg = String()
        msg.data = selection
        self.selection_publisher.publish(msg)
        self.get_logger().info(f"선택한 물체 전송: {selection}")

    def send_gui_command(self, red_count, blue_count, goal_count):  # 새로운 메서드 추가
        command_dict = {"goal": goal_count}
        if red_count > 0:
            command_dict["red"] = red_count
        if blue_count > 0:
            command_dict["blue"] = blue_count
        
        msg = String()
        msg.data = json.dumps(command_dict)
        self.command_publisher.publish(msg)
        self.get_logger().info(f"GUI 명령 전송: {msg.data}")

    def status_callback(self, msg):
        self.get_logger().info(f"컨베이어 상태 업데이트: {msg.data}")
        data = "[conveyor/status] : " + msg.data
        self.status_signal.emit(data)
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_signal.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")

class ConveyorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Conveyor Controller")
        self.setGeometry(100, 100, 600, 500)

        self.status_label = QLabel("현재 상태: INIT")
        self.status_label.setStyleSheet("font-size: 16px; font-weight: bold;")

        self.status_history = []
        self.history_label = QTextEdit()
        self.history_label.setReadOnly(True)
        self.history_label.setStyleSheet("font-size: 14px;")
        self.history_label.setFixedHeight(150)
        
        self.node = ConveyorController()
        self.ros_thread = RosThread(self.node)
        self.node.status_signal.connect(self.update_status)
        self.node.image_signal.connect(self.update_camera_feed)
        self.ros_thread.start()

        self.button_start = QPushButton("컨베이어 벨트 시작")
        self.button_stop = QPushButton("컨베이어 벨트 종료")
        self.button_execute = QPushButton("실행")
        self.button_exit = QPushButton("종료")

        self.button_start.clicked.connect(lambda: self.node.send_command("go"))
        self.button_stop.clicked.connect(lambda: self.node.send_command("stop"))
        self.button_exit.setStyleSheet("background-color: gray; color: white;")
        self.button_exit.clicked.connect(self.close_application)

        self.red_label = QLabel("빨간 물체 개수:")
        self.red_combo = QComboBox()
        self.red_combo.addItems([str(i) for i in range(10)])
        
        self.blue_label = QLabel("파란 물체 개수:")
        self.blue_combo = QComboBox()
        self.blue_combo.addItems([str(i) for i in range(10)])

        self.goal_label = QLabel("목표 위치:")
        self.goal_combo = QComboBox()
        self.goal_combo.addItems([str(i) for i in range(1, 4)])

        self.button_execute.clicked.connect(self.execute_command)

        self.camera_label = QLabel("카메라 피드 없음")
        self.camera_label.setStyleSheet("border: 1px solid black; background-color: black;")
        self.camera_label.setFixedSize(400, 300)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_start)
        button_layout.addWidget(self.button_stop)

        input_layout = QHBoxLayout()
        input_layout.addWidget(self.red_label)
        input_layout.addWidget(self.red_combo)
        input_layout.addWidget(self.blue_label)
        input_layout.addWidget(self.blue_combo)
        input_layout.addWidget(self.goal_label)
        input_layout.addWidget(self.goal_combo)
        input_layout.addWidget(self.button_execute)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.status_label)
        main_layout.addWidget(self.history_label)
        main_layout.addLayout(button_layout)
        main_layout.addLayout(input_layout)
        main_layout.addWidget(self.camera_label)

        exit_layout = QHBoxLayout()
        exit_layout.addStretch()
        exit_layout.addWidget(self.button_exit)
        
        main_layout.addLayout(exit_layout)
        
        self.setLayout(main_layout)
    
    def execute_command(self):
        red_count = int(self.red_combo.currentText())
        blue_count = int(self.blue_combo.currentText())
        goal_count = int(self.goal_combo.currentText())
        
        # 기존 selection 발행 (유지)
        self.node.send_selection(f"red:{red_count}, blue:{blue_count}, goal:{goal_count}")
        
        # 새로 추가된 GUI 명령 발행
        self.node.send_gui_command(red_count, blue_count, goal_count)

        self.status_label.setText(f"빨간 물체 {red_count}개, 파란 물체 {blue_count}개, 목표 {goal_count}개 실행 중...")
    
    def update_status(self, status):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_entry = f"[{timestamp}] {status}"
        
        self.status_history.append(log_entry)
        if len(self.status_history) > 100:
            self.status_history.pop(0)
        
        self.history_label.setPlainText('\n'.join(self.status_history))
        self.history_label.verticalScrollBar().setValue(self.history_label.verticalScrollBar().maximum())
    
    def update_camera_feed(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(qt_image)
        self.camera_label.setPixmap(pixmap.scaled(self.camera_label.size()))
    
    def close_application(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.close()

class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = ConveyorGUI()
    window.show()
    
    app.exec_()  # Qt 이벤트 루프 실행
    rclpy.shutdown()  # ROS 종료
if __name__ == '__main__':
    main()