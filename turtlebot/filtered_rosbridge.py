import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

#글로벌 변수로 프로세스 저장
rosbridge_process = None

class JointStateFilterNode(Node):
    def __init__(self):
        super().__init__('joint_state_filter_node')

        # 토픽 구독 및 퍼블리시 설정
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/filtered_joint_states', 10)
        self.get_logger().info("\033[32mJointStateFilterNode is waiting...\033[0m")

    def joint_state_callback(self, msg):
        # 새로운 JointState 메시지 생성
        filtered_msg = JointState()
        filtered_msg.header = msg.header
        filtered_msg.name = msg.name
        filtered_msg.position = msg.position
        filtered_msg.velocity = msg.velocity
        
        # nan 값을 0.0으로 변경
        filtered_msg.effort = [0.0 if math.isnan(e) else e for e in msg.effort]

        # 퍼블리시
        self.publisher.publish(filtered_msg)
        self.get_logger().info("Published filtered_joint_states with efforts: nan → 0.0")

def run_rosbridge():
    """rosbridge_websocket_launch.xml 실행"""
    global rosbridge_process
    rosbridge_command = ['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml']

    try:
        rosbridge_process = subprocess.Popen(rosbridge_command, preexec_fn=os.setsid, stdout=None, stderr=None)
        time.sleep(1)  # 실행 대기
        print("\033[32mrosbridge_server 실행 성공 (백그라운드 실행 중)\033[0m", flush=True)
    except Exception as e:
        print(f"\033[31mrosbridge_server 실행 중 오류 발생: {e}\033[0m", flush=True)

def cleanup():
    """Python 종료 시 실행되는 함수 (자식 프로세스 종료)"""
    global rosbridge_process
    if rosbridge_process:
        os.killpg(os.getpgid(rosbridge_process.pid), signal.SIGINT)  # 정상 종료 시도
        time.sleep(1)  # 종료 대기
        rosbridge_process = None

def main():
    run_rosbridge()  # rosbridge 실행

    rclpy.init()
    node = JointStateFilterNode()

    try:
        rclpy.spin(node)  # ROS 2 노드 실행
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt (Ctrl+C)", flush=True)
    finally:
        cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()