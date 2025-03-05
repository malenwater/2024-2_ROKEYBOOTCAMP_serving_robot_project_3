import rclpy
from srv_call_test import TurtlebotArmClient

def main():
    rclpy.init()  # rclpy 초기화

    arm_client = TurtlebotArmClient()
    
    print("hi")
    # 요청을 여러 번 보낼 수 있게 하려면 이 부분을 반복문으로 처리할 수 있습니다.
    response = arm_client.send_request(9, "")
    
    print(response)  # 한 번 응답을 받고 출력합니다.
    
    # 여러 번 출력하려면 아래와 같이 반복문을 사용할 수 있습니다.
    for _ in range(4):  # 추가로 4번 출력
        print(response)
    
    rclpy.shutdown()  # 종료

if __name__ == '__main__':
    main()

# ros2 service call /moveit_control turtlebot_cosmo_interface/srv/MoveitControl "{
#   cmd: 9, 
#   posename: '', 
#   waypoints: {
#     header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
#     poses: []
#   }
# }"
