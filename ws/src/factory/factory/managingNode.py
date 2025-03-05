import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseArray
from aruco_msgs.msg import MarkerArray 
from srv_call_test import TurtlebotArmClient
import ast  # 문자열을 리스트로 변환하기 위한 라이브러리
import json
import time
import os

# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화

class ManagingNode(Node):
    def __init__(self):
        super().__init__("managing_node")
        self.get_logger().info(f"managing_node 시작")
        self.DEBUG = True
        self.WORKING = False
        self.ID_CLASS = [0, 1, 2]
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
        self.armrun = False
        self.count = 0
        self.yolofind = False
        self.block_ids = []
        self.len_block_ids = 0
        # 파일 경로 설정
        file_path = './offset_values.txt'

        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                for line in file:
                    # 각 줄을 "변수명 : 값" 형식으로 분리하여 변수에 값을 넣음
                    parts = line.strip().split(":")
                    if len(parts) == 2:
                        var_name, value = parts
                        try:
                            # 값을 float로 변환하여 변수에 할당
                            value = float(value.strip())
                            if var_name == "right_low_x_offset ":
                                self.right_low_x_offset = value
                            elif var_name == "right_low_y_offset ":
                                self.right_low_y_offset = value
                            elif var_name == "right_high_x_offset ":
                                self.right_high_x_offset = value
                            elif var_name == "right_high_y_offset ":
                                self.right_high_y_offset = value
                            elif var_name == "left_low_x_offset ":
                                self.left_low_x_offset = value
                            elif var_name == "left_low_y_offset ":
                                self.left_low_y_offset = value
                            elif var_name == "left_high_x_offset ":
                                self.left_high_x_offset = value
                            elif var_name == "left_high_y_offset ":
                                self.left_high_y_offset = value
                        except ValueError:
                            pass
        self.state = "START"
        response = self.arm_client.send_request(1, "camera_home")
        self.get_logger().info(f'Response: {response.response}')
        
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
            self.block_ids = [0] * int(self.GUI_COMMAND['red']) + [1] * int(self.GUI_COMMAND['blue'])
            self.len_block_ids = len(self.block_ids)

            self.state = 'ARUCO'
            self.aroco_Block_status(self.state)
            # self.state = 'YOLO'
            self.yolo_status(self.state)
            # self.state = 'BACKWARD'
            self.aroco_Block_status(self.state)
            # self.state = 'PURPLE'
            self.yolo_status(self.state)
            # self.state = 'CHECK'
            self.aroco_Block_status(self.state)
            self.get_logger().info(f'Response: {self.state}')

            
        except Exception as e:
            self.get_logger().error(f"[사용자 입력] 명령 해석 실패: {e}")
        finally:
            self.WORKING = False
    
    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)  
        
    def check_pose(self,data_list):
        pose = {
            "1" : [],
            "2" : [],
            "3" : [],
            "4" : [],
        }
        for data in data_list:
            if data[1] > 0 and data[2] > 0:
                pose["1"] =  data
            elif data[1] > 0 and data[2] < 0:
                pose["2"] =  data
            elif data[1] < 0 and data[2] > 0:
                pose["3"] =  data
            elif data[1] < 0 and data[2] < 0:
                pose["4"] =  data
        return pose
    
    def choose_block(self,data_list):
        data_pose = self.check_pose(data_list)
        pick_data = self.block_ids[self.count][0]
        if data_pose["1"][0] == pick_data:
            return data_pose["1"]
        elif data_pose["2"][0] == pick_data:
            return data_pose["2"]
        elif data_pose["3"][0] == pick_data:
            return data_pose["3"]
        elif data_pose["4"][0] == pick_data:
            return data_pose["4"]
        return data_list
    
    def yolo_status(self,state):
        while True:
            if self.YOLO_DETECTED_INFO is None:
                continue
            if state != self.state:
                break
            
            if not self.armrun:  # 로봇 암이 동작 중이 아니면
                try:
                    data_list = self.YOLO_DETECTED_INFO
                    if len(data_list) > 0:
                        data_list = self.choose_block(data_list)
                        self.yolo_x = data_list[1]
                        self.yolo_y = data_list[2]
                        
                        print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                        print("done")

                        if self.state == 'YOLO':
                            if not self.yolofind:
                                self.yolofind = True
                                self.yolo_arm_controll()
                                
                                if self.count == self.len_block_ids:
                                    self.home2_arm_controll()
                                    self.state = 'BACKWARD'
                                    self.count = 0
                        
                        elif self.state == 'PURPLE':
                            if not self.yolofind and abs(self.yolo_x) < 0.01:
                                self.publish_cmd_vel(0.0)
                                self.yolofind = True
                                self.purple_arm_control()
                            elif not self.yolofind and self.yolo_x > 0.01:
                                self.publish_cmd_vel(-0.01)
                            elif not self.yolofind and self.yolo_x < -0.01:
                                self.publish_cmd_vel(0.01)                      
                                
                except Exception as e:
                    self.get_logger().error(f"Error processing the data: {e}")
            else:
                time.sleep(0.2)
                    
    def purple_arm_control(self):
        if self.state == 'PURPLE':
            arm_client = TurtlebotArmClient()


            print ("task start!")
            
            print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

            if self.yolofind:
                self.armrun = True

                response = arm_client.send_request(2, "open")
                arm_client.get_logger().info(f'Response: {response.response}')
                time.sleep(1)

                pose_array = self.append_pose_init(0.0103589 ,-0.2700000  ,0.205779  - self.yolo_y + 0.06 )

                response = arm_client.send_request(3, "", pose_array)
                arm_client.get_logger().info(f'Response: {response.response}')

                response = arm_client.send_request(9, "")
                arm_client.get_logger().info(f'Response: {response.response}')

                pose_array = self.append_pose_init(0.0103589,-0.3000000   ,0.205779  - self.yolo_y + 0.06 )

                response = arm_client.send_request(3, "", pose_array)
                arm_client.get_logger().info(f'Response: {response.response}')     

                response = arm_client.send_request(9, "")
                arm_client.get_logger().info(f'Response: {response.response}')

                response = arm_client.send_request(2, "close")
                arm_client.get_logger().info(f'Response: {response.response}')
                time.sleep(1)

                response = arm_client.send_request(1, "box_up_01")
                arm_client.get_logger().info(f'Response: {response.response}')    
                time.sleep(1)

                response = arm_client.send_request(1, "box_up_02")
                arm_client.get_logger().info(f'Response: {response.response}')    
                time.sleep(1)

                response = arm_client.send_request(1, "box_up_03")
                arm_client.get_logger().info(f'Response: {response.response}')    
                time.sleep(1)

                response = arm_client.send_request(1, "box_back_01")
                arm_client.get_logger().info(f'Response: {response.response}')   

                time.sleep(1)


                print("jobs_done")

                self.armrun = False
                self.yolofind = False  # 작업 완료 후 초기화
                
                self.state = 'CHECK'
                
    def home2_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "home2")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)      
    
    def append_pose_init(self, x,y,z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y =  y
        pose.position.z =  z

        pose_array.poses.append(pose)
        
        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array
    
    def yolo_arm_controll(self):
        arm_client = TurtlebotArmClient()

        print ("task start!")
        print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

        if self.yolofind:
            self.armrun = True

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.05,0.00 - self.yolo_x ,0.122354 )

            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.05,0.00 - self.yolo_x ,0.087354  )

            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
   
            response = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            print ("conveyor task start")

            response = arm_client.send_request(1, "conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "test_conveyor")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("throw ")
            
            response = arm_client.send_request(1, "conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')    

            time.sleep(3)

            print("jobs_done")

            self.armrun = False
            self.yolofind = False  # 작업 완료 후 초기화
            
            self.count += 1
            
    def aroco_Block_status(self,state):
        self.target_marker_id = 0
        while True:
            if not self.SUBSCRIPTION_MARKER:
                continue
            if state != self.state:
                break
        
            for marker in self.SUBSCRIPTION_MARKER:
                if marker.id == self.target_marker_id:
                    self.marker_id = marker.id
                    self.aruco_pose = marker.pose.pose  # Aruco의 위치 저장
                    self.get_logger().info(f'Marker ID: {marker.id}, PositionZ: {self.aruco_pose.position.z}')
                    self.aruco_marker_found = True
                    if self.state ==  'ARUCO':
                        self.execute_forward_task(self.aruco_pose.position.z)  # 전진 작업 실행
                    elif self.state == 'BACKWARD':
                        self.execute_backward_task(self.aruco_pose.position.z)
                        
                if self.state == 'CHECK':
                    marker_id = marker.id
                    x_position = marker.pose.pose.position.x
                    if marker_id == self.GUI_COMMAND['goal'] and abs(x_position) <= 0.05:  # marker3을 가까이서 감지하면 정지
                        print("find")
                        self.publish_cmd_vel(0.0)
                        self.final_task()
                    else:  # 어떤 marker든 감지되면 전진
                        print("keep run")
                        self.publish_cmd_vel(0.03)
                        
    def final_task(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "box_back_put")
        arm_client.get_logger().info(f'Response: {response.response}') 
        time.sleep(1)
        response = arm_client.send_request(2, "open")
        arm_client.get_logger().info(f'Response: {response.response}')       
        self.state = "FINISH"
    
    def sub_conveyor_status_callback(self, msg):
        """컨베이어 상태를 확인하는 콜백 함수"""
        self.CONVEYOR_STATUS = msg.data.strip()
        if self.DEBUG:
            self.get_logger().info(f"[컨베이어 상태] 현재 상태: {self.CONVEYOR_STATUS}")
            
    def execute_forward_task(self, current_z_position):
        # 전진 작업: 30cm까지 전진 후 멈추고, 작업을 진행
        if self.aruco_marker_found and self.aruco_pose:
            self.get_logger().info("Executing forward task...")
            # 목표 z축 위치를 30cm로 설정
            if current_z_position > 0.3:
                self.publish_cmd_vel(0.05)
            elif current_z_position > 0.25:
                self.publish_cmd_vel(0.025)
            else:
                self.publish_cmd_vel(0.0)
                self.get_logger().info("Target reached")
                self.camera_arm_controll()
                self.state = 'YOLO'

                
    def execute_backward_task(self, current_z_position):
        # 후진 작업: 1m만큼 후진하고 다시 Aruco marker를 확인
        if self.aruco_marker_found and self.aruco_pose:
            self.get_logger().info("Executing backward task...")
            # 목표 z축 위치를 30cm로 설정
            if current_z_position < 0.98:
                self.publish_cmd_vel(-0.05)
            else:
                self.publish_cmd_vel(0.0)
                self.get_logger().info("Target reached")
                self.box_home_arm_controll()
                self.state = 'PURPLE'
                
    def box_home_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "box_home_01")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)      
                   
    def camera_arm_controll(self):
        arm_client = TurtlebotArmClient()
        response = arm_client.send_request(1, "camera_home")
        arm_client.get_logger().info(f'Response: {response.response}')
        time.sleep(3)        
        
      
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
