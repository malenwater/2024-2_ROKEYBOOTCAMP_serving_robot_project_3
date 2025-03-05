import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from aruco_msgs.msg import MarkerArray 
from srv_call_test import TurtlebotArmClient
import ast  # 문자열을 리스트로 변환하기 위한 라이브러리
import json
import time

class ManagingNode(Node):
    def __init__(self):
        super().__init__("managing_node")
        self.get_logger().info(f"managing_node 시작")
        self.DEBUG = True
        self.WORKING = False
        # 사용자 입력 처리 (gui/command 토픽)
        self.subscription_command = self.create_subscription(
            String,
            "gui/command",
            self.sub_user_command_callback,
            10
        )
        self.GUI_COMMAND = None
        # 컨베이어 상태 확인 (conveyor/status 토픽)
        self.subscription_conveyor = self.create_subscription(
            String,
            "conveyor/status",
            self.sub_conveyor_status_callback,
            10
        )
        self.CONVEYOR_STATUS = None
        # 물체 감지 정보 수신 (yolo/detected_info 토픽)
        self.subscription_yolo = self.create_subscription(
            String,
            "yolo/detected_info",
            self.sub_object_detection_callback,
            10
        )
        self.YOLO_DETECTED_INFO = None
        # 감지된 MarkerArray 수신 (detected_markers 토픽)
        self.subscription_marker = self.create_subscription(
            MarkerArray,
            "detected_markers",
            self.sub_detected_markers_callback,
            10
        )
        self.SUBSCRIPTION_MARKER = []
        self.publisher_conveyor = self.create_publisher(
            String,
            'conveyor/control',
            10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        self.twist = Twist()
        self.target_marker_id = None
        self.marker = []
        self.arm_client = TurtlebotArmClient()
        
        self.get_logger().info(f"managing_node 설정 끝")
        
    def publish_control_conveyor(self,control : String, distance : float):
        # 퍼블리시할 메시지 생성 (예: 'go' 명령)
        control_message = {
                            "control" : control,
                            "distance.mm" : distance,
                           }
        msg = String()
        msg.data = json.dumps(control_message)
        self.publisher_conveyor.publish(msg)
        if self.DEBUG:
            self.get_logger().info(f"퍼블리시된 메시지: {msg.data}")
        
    def sub_user_command_callback(self, msg):
        """사용자의 조작 명령을 처리하는 콜백 함수"""
        if self.WORKING:
            return
        self.WORKING = True
        try:
            self.GUI_COMMAND = ast.literal_eval(msg.data)  # JSON 형식 문자열을 딕셔너리로 변환
            if self.DEBUG :
                self.get_logger().info(f"[사용자 입력] 수신된 명령: {self.GUI_COMMAND}")
                self.get_logger().info(f"[사용자 입력] 수신된 red 명령: {self.GUI_COMMAND['red'] }")
                self.get_logger().info(f"[사용자 입력] 수신된 blue 명령: {self.GUI_COMMAND['blue'] }")
                self.get_logger().info(f"[사용자 입력] 수신된 goal 명령: {self.GUI_COMMAND['goal'] }")
            block_ids = ["0"] * int(self.GUI_COMMAND['red']) + ["1"] * int(self.GUI_COMMAND['blue'])
            
            # 1번째 MARKER의 0번을 읽고 일정 위치까지 이동하기 (Aroco + cmd_vel)
                # for 문으로 레드, 블루의 개수에 따라 아래 3,4,5번을 반복한다
                # 3번째 특정 위치에 가서 사진을 찍어서 사용할 블럭의 위치를 특정한다. (팔작업 + YOLO)
                # 4번째 특정 위치에 가서 블럭을 집고, 컨테이너 벨트에 올리고 원위치로 온다. (팔작업)
                # 5번째 컨테이너 벨트를 움직인다. (컨테이너 벨트)
            # 컨테이너 벨트를 움직여서 모든 블럭을 basket에 옮긴다. (컨테이너 벨트)
            # basket을 찾기 위해 움직인다. (Aroco + cmd_vel)
            # basket을 찾으면 잡는다. (팔작업 + YOLO)
            # basket을 반대로 다시 두고 MARKER을 찾기위해 다시 움직인다.(해당하는)
            # MARKER을 중앙에 두고 다시 basket을 놓는다. 
            
            self.move_Block_status()
            for block_id in block_ids:
                if self.DEBUG :
                    self.get_logger().info(f"집을 박스 ID: {block_id}")
                pick_place = self.check_Block_Place(block_id)
                self.pick_Block_Place(pick_place)
                self.move_Conveyor_Next()
            self.move_Conveyor_ALL()
            self.check_move_Basket()
            self.pick_Basket()
            self.check_move_aroco()
            self.put_aroco()
            
        except Exception as e:
            self.get_logger().error(f"[사용자 입력] 명령 해석 실패: {e}")
        finally:
            self.WORKING = False
    
    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)   
        
    def move_Block_status(self):
        self.target_marker_id = 0
        running = True
        while running:
            if not self.SUBSCRIPTION_MARKER:
                return
            
            for self.marker in self.SUBSCRIPTION_MARKER:
                if self.marker.id == self.target_marker_id:
                    # self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                    print(f'z:[{self.marker.pose.pose.position.z}] x:[{self.marker.pose.pose.position.x}] ')
                    # if marker.pose.pose.position.z > 0.30:
                    if self.marker.pose.pose.position.z > 0.467:
                        self.publish_cmd_vel(0.10)
                    elif self.marker.pose.pose.position.z > 0.30:
                        self.publish_cmd_vel(0.06)
                    elif self.marker.pose.pose.position.z > 0.20 :
                        self.publish_cmd_vel(0.04)
                    else:
                        self.publish_cmd_vel(0.0)
                        self.finish_move = True
                    break
            time.sleep(0.2)
            
    def check_Block_Place(self,block_id):
        pass
        return "1"
    def pick_Block_Place(self,pick_place):
        pass
    def move_Conveyor_Next(self):
        pass
    def move_Conveyor_ALL(self):
        pass
    def check_move_Basket(self):
        pass
    def pick_Basket(self):
        pass
    def check_move_aroco(self):
        pass
    def put_aroco(self):
        pass
    def sub_conveyor_status_callback(self, msg):
        """컨베이어 상태를 확인하는 콜백 함수"""
        self.CONVEYOR_STATUS = msg.data.strip()
        if self.DEBUG:
            self.get_logger().info(f"[컨베이어 상태] 현재 상태: {self.CONVEYOR_STATUS}")

    def sub_object_detection_callback(self, msg):
        """객체 감지 정보를 배열로 변환하고 출력하는 콜백 함수"""
        try:
            self.YOLO_DETECTED_INFO = ast.literal_eval(msg.data)  # 문자열을 리스트로 변환
            if self.YOLO_DETECTED_INFO and self.DEBUG:
                self.get_logger().info(f"[물체 감지] 감지된 객체 목록: {self.YOLO_DETECTED_INFO}")
                for data in self.YOLO_DETECTED_INFO:
                    self.get_logger().info(f"[물체 감지] 감지된 객체: {data}")
                
        except Exception as e:
            self.get_logger().error(f"[물체 감지] 데이터 해석 실패: {e}")

    def sub_detected_markers_callback(self, msg: MarkerArray):
        """감지된 ArUco MarkerArray 데이터를 처리하는 콜백 함수"""
        self.SUBSCRIPTION_MARKER = msg.markers  # msg의 markers 리스트 저장

        if self.DEBUG:
            self.get_logger().info(f"[마커 감지] 총 {len(self.SUBSCRIPTION_MARKER)}개의 마커 감지됨.")
            for marker in self.SUBSCRIPTION_MARKER:
                self.get_logger().info(
                    f" - 마커 ID: {marker.id}, 위치: ({marker.pose.pose.position.x}, {marker.pose.pose.position.y}, {marker.pose.pose.position.z})"
                )
def main(args=None):
    rclpy.init(args=args)
    node = ManagingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# 위에서 보는 값 [0.0, -0.05522330836388308, -0.4049709280018093, 1.9987769666149904] 

# arm_client.send_request(2, "open")
# arm_client.send_request(2, "close")
# arm_client.send_request(0, "", pose_array)
# arm_client.send_request(3, "", pose_array)

# arm_client.send_request(1, "home1")
# arm_client.send_request(1, "home2")

# arm_client.send_request(1, "conveyor_up")
# arm_client.send_request(1, "conveyor_down")
# arm_client.send_request(1, "camera_home")
# arm_client.send_request(1, "test_conveyor")
# arm_client.send_request(1, "box_home_01")

# arm_client.send_request(1, "box_up_01")
# arm_client.send_request(1, "box_up_02")
# arm_client.send_request(1, "box_up_03")
# arm_client.send_request(1, "box_front")
# arm_client.send_request(1, "box_back_01")
# arm_client.send_request(1, "box_back_put")

if __name__ == "__main__":
    main()
