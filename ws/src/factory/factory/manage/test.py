import rclpy
from .srv_call_test import TurtlebotArmClient
from geometry_msgs.msg import Twist, Pose, PoseArray
import time
def append_pose_init( x,y,z):
    pose_array = PoseArray()
    pose = Pose()

    pose.position.x = x
    pose.position.y =  y
    pose.position.z =  z

    pose_array.poses.append(pose)
    
    return pose_array
    
def main():
    rclpy.init()  # rclpy 초기화
# data: '[[1, 0.06974, -0.03807], [0, -0.05426, -0.03556], [1, 0.06702, 0.02...'

# data: '[ [1, 0.067014, -0.037944], [0, -0.054621, -0.035955],[1, 0.066555, 0.023409]]'
# # data: '[[1, 0.071504, 0.025092], [0, -0.058548, -0.03854], [1, 0.07134, -0.040672]]'
# # data: '[ [1, 0.070192, -0.041], [0, -0.0574, -0.038704],[1, 0.07134, 0.02378]]'
    arm_client = TurtlebotArmClient()
    response = arm_client.send_request(2, "open")
    arm_client.get_logger().info(f'Response: {response.response}')  
    response = arm_client.send_request(1, "camera_home")
    arm_client.get_logger().info(f'Response: {response.response}')   
    # response = arm_client.send_request(1, "box_back_put_2")
    # arm_client.get_logger().info(f'Response: {response.response}')   
    
    # response = arm_client.send_request(1, "box_up_03")
    # arm_client.get_logger().info(f'Response: {response.response}')    
    # response = arm_client.send_request(1, "box_back_01")
    # arm_client.get_logger().info(f'Response: {response.response}')    
    
    # pose_array = append_pose_init(0.00248397, -0.28187, 0.374011)
    # response = arm_client.send_request(3, "", pose_array)
    
    # pose_array = self.append_pose_init(0.0103589 ,-0.2700000  ,0.205779)
    # pose_array = append_pose_init(0.00248397, -0.28187, 0.354011)
    # response = arm_client.send_request(3, "", pose_array)
    print("1")
# # pose_array = self.append_pose_init(0.0103589,-0.3000000   ,0.205779  - self.yolo_y + 0.06 )
#     pose_array = append_pose_init(0.0016946549744881157, -0.05521983269987424, 0.17193338228588664)
#     response = arm_client.send_request(3, "", pose_array)
#     print("2")
#     pose_array = append_pose_init(0.0017793591098962654, -0.0552171681639206, 0.17193338228588664)
#     response = arm_client.send_request(3, "", pose_array)
#     print("3")
    
#     pose_array = append_pose_init(0.0030045548608435774, 0.12239199607668318, 0.19676864700270505)
#     response = arm_client.send_request(3, "", pose_array)
#     print("4")
    
    
#     # pose_array = append_pose_init(-0.0016946549744881157, -0.05521983269987424, 0.17193338228588664)
    print("1")
    time.sleep(1)
    pose_array = append_pose_init(0.2559, 0.07075, 0.122354)
    response = arm_client.send_request(0, "", pose_array)
    #  x: 0.2554, y: 0.05915, z: 0.100354
    print("1")
    time.sleep(1)
    pose_array = append_pose_init(0.25591600000000003, 0.07075200000000001, 0.089354)
    response = arm_client.send_request(0, "", pose_array)
    time.sleep(1)
    print("1")
#     print("5")
    response = arm_client.send_request(9, "")
    # response = arm_client.send_request(3, "", pose_array)
    # response = arm_client.send_request(2, "close")
   
    
    
    # 여러 번 출력하려면 아래와 같이 반복문을 사용할 수 있습니다.
    for _ in range(4):  # 추가로 4번 출력
        print(response)
    
    rclpy.shutdown()  # 종료

if __name__ == '__main__':
    main()