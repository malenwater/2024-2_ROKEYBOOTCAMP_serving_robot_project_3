## 수정 사항
    - 블럭 집는 코드, 전진, check, 바구니 집고 놓기 등을 수정하였다.
## 사용법 실행순서
    저희는 아래와 같은 파일 ?개를 실행해야합니다.
        - 터틀봇3 내부에, 4개의 파이썬 파일을 실행
        - PC에서, 5개의 필요 파일을 실행
        - PC에서, 유니티를 위한 파일 2개를 실행

    터틀봇3 내부에 실행해야하는 파일 내용은
        - 이미지 pub 하는 코드 (강사님 코드)
        - Aroco pub 하는 코드 (강사님 코드)
        - Yolo pub 하는 코드 (A-3 코드)
        - 터틀봇3 bringup lauch (강사님 코드)
    이에 따른 코드는
        - python3 robot_compressed_image_pub_test.py
        - python3 robot_aruco_marker_detect.py
        - python3 PubImgYoloNode.py (내부에 꼭 best.pt에 대한 경로를 변경해 주십시오.)
        - ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

    PC에서 파이썬 파일을 실행하는 내용은
        - 터틀봇3 매뉴플레이터 moveit2 lauch 코드 (강사님코드) 
        - moveit2를 관리하는 서버 실행 코드 (강사님코드)
        - managing Node 코드 
        - 컨테이너 벨트 Node 노드
        - GUI 실행 코드
    이에 따른 코드는
        - ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
        - ros2 run turtlebot_moveit turtlebot_arm_controller
        - python3 -m factory.factory.manage.managingNode
        - python3 ConveyorSerialController.py
        - python3 GUI_Node.py

    유니티를 실행하기 위한 파일 내용은
        - 유니티 실행 
        - 유니티 ros2를 위한 실행
    이에 따른 코드는
        - filtered_rosbridge.py 실행 (경우에 따라 ros2 launch rosbridge_server rosbridge_websocket_launch.xml 실행)
## 파일 구조도
A-3-제출자료
    - README.md                                         # 설명 md
    - A3-발표자료 -  자동물류 분류 시스템.pptx                # 발표자료
    - turtlebot3_ws                                     # 터틀봇3 패키지 폴더 ** 터틀봇3 패키지 **
    - UnityProject                                      # 유니티 프로젝트 폴더 ** 유니티 **
    - PC_ws                                             # 로봇 내 실행 파일 3개 + PC 실행 파일 3개
      - factory
        ├── conveyor                            # 컨테이너 관련 폴더
        │   ├── ConveyorSerialController.py              # 컨테이너 제어 코드
        │   └── __init__.py
        ├── gui                                 # GUI 관련 폴더
        │   ├── GUI_Node.py                              # GUI 실행 코드
        │   └── __init__.py
        ├── __init__.py
        ├── manage                              # Mange 관련 폴더
        │   ├── __init__.py
        │   ├── managingNode.py                          # Managing 하는 파일 코드
        │   ├── offset_values.txt
        │   ├── srv_call_test.py
        └── robot                               # 터틀봇3 내부 실행할 파일 관련 폴더
            ├── calibration_params.yaml
            ├── __init__.py
            ├── detect_best.pt
            ├── PubImgYoloNode.py                        # Yolo 파일
            ├── robot_aruco_marker_detect.py             # Aroco 파일 (강사)
            └── robot_compressed_image_pub_test.py       # Img 파일 (강사)
