import rclpy
from .srv_call_test import TurtlebotArmClient
from geometry_msgs.msg import Twist, Pose, PoseArray

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

    arm_client = TurtlebotArmClient()
    
    # response = arm_client.send_request(1, "box_back_put_1")
    # arm_client.get_logger().info(f'Response: {response.response}')   
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
#     pose_array = append_pose_init(0.17246755169677735, -0.06311327871093751, 0.122354)
#     response = arm_client.send_request(0, "", pose_array)
#     print("5")
    response = arm_client.send_request(9, "")
    # response = arm_client.send_request(3, "", pose_array)
    # response = arm_client.send_request(2, "close")
    response = arm_client.send_request(2, "open")
    arm_client.get_logger().info(f'Response: {response.response}')     
    
    
    # 여러 번 출력하려면 아래와 같이 반복문을 사용할 수 있습니다.
    for _ in range(4):  # 추가로 4번 출력
        print(response)
    
    rclpy.shutdown()  # 종료

if __name__ == '__main__':
    main()