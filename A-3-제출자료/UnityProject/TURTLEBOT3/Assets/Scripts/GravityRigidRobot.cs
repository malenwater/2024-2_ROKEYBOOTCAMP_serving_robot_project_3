
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections;
using Newtonsoft.Json;
using System.Collections.Generic;

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;

using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std; // String 메세지
using sens_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor; // JointState 메시지
using nav_msgs = RosSharp.RosBridgeClient.MessageTypes.Nav;     //odom 메세지

using System;
using System.IO;

public class GravityRigidRobot : MonoBehaviour
{

    private RosSocket rosSocket;   
    private GameObject robot, base_scan;

    public Rigidbody joint0, joint1, joint2, joint3, joint4;  // 피벗 오브젝트
    public Rigidbody gripper_right, gripper_left;


    public Rigidbody base_link, wheel_left_link, wheel_right_link, caster_back_right_link, caster_back_left_link;

    private GameObject end_effector, basketbase;

    private GameObject conveyor;

    private GameObject goal1, goal2, goal3;
    private Boolean boxhold = false;
    private Button colorbutton;

    private float meterScale = 1000f;
    public Vector3 cameraOffset = new Vector3(-500f, 500f, -1000f); // 카메라를 더 멀리 위치시키기 위한 오프셋
    public Camera mainCamera; // 메인 카메라
    public float cameraMoveSpeed = 200;

    private float positionX, positionY, positionZ;
    public Vector3 robotOffset;
    public float conveyorSpeed = 5.0f;
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 5.0f;
    public float jointVelocity = 5.0f;
     
    private float orientationX, orientationY, orientationZ, orientationW;
    private string[] jointName;  // jointName 배열로 변경

    private float[] Position, Velocity, Effort;   // float 배열로 정의   

    private System.Random random = new System.Random();

    private Rigidbody[] cubes = new Rigidbody[5];
    private Rigidbody closestBox;

    private WebSocketNetProtocol webSocket;



    private void Start()
    
    {

        if (rosSocket != null)
        {
            rosSocket.Close();
        }

        // WebSocketNetProtocol 사용
        rosSocket = new RosSocket(new WebSocketNetProtocol("ws://localhost:9090"));

        Debug.Log("Connected to ROS bridge server at ws://localhost:9090");

        // JointSatate 구독 요청
        rosSocket.Subscribe<sens_msgs.JointState>("/filtered_joint_states", JointStateCallback);    //.nan을 0.0으로

        // odom 구독 요청
        rosSocket.Subscribe<nav_msgs.Odometry>("/odom", OdometryCallback);      //실제 turtlebot3

        rosSocket.Subscribe<std_msgs.String>("/conveyor/control", ConveyorCallback);

        robot = GameObject.Find("turtlebot3_manipulation");

        joint0 = GameObject.Find("link1").GetComponent<Rigidbody>();
        joint1 = GameObject.Find("link2").GetComponent<Rigidbody>();
        joint2 = GameObject.Find("link3").GetComponent<Rigidbody>();
        joint3 = GameObject.Find("link4").GetComponent<Rigidbody>();
        joint4 = GameObject.Find("link5").GetComponent<Rigidbody>();        
        gripper_right = GameObject.Find("gripper_right_link").GetComponent<Rigidbody>();
        gripper_left = GameObject.Find("gripper_left_link").GetComponent<Rigidbody>();

        base_link = GameObject.Find("base_link").GetComponent<Rigidbody>(); 
        wheel_left_link = GameObject.Find("wheel_left_link").GetComponent<Rigidbody>();
        wheel_right_link = GameObject.Find("wheel_right_link").GetComponent<Rigidbody>();
        caster_back_right_link = GameObject.Find("caster_back_right_link").GetComponent<Rigidbody>();
        caster_back_left_link = GameObject.Find("caster_back_left_link").GetComponent<Rigidbody>();

        end_effector = GameObject.Find("end_effector_link");
        conveyor = GameObject.Find("Conveyor");
        basketbase = GameObject.Find("Basket_base");
        base_scan = GameObject.Find("base_scan");

        goal1 = GameObject.Find("Goal_base_1");
        goal2 = GameObject.Find("Goal_base_2");
        goal3 = GameObject.Find("Goal_base_3");

        cubes[0] = GameObject.Find("Cube_1").GetComponent<Rigidbody>();
        cubes[1] = GameObject.Find("Cube_2").GetComponent<Rigidbody>();
        cubes[2] = GameObject.Find("Cube_3").GetComponent<Rigidbody>();
        cubes[3] = GameObject.Find("Cube_4").GetComponent<Rigidbody>();
        cubes[4] = GameObject.Find("Cube_basket").GetComponent<Rigidbody>();

        mainCamera = Camera.main;       
        colorbutton = GameObject.Find("colorButton").GetComponent<Button>();

        joint0.isKinematic = true;
        joint1.isKinematic = true;
        joint2.isKinematic = true;
        joint3.isKinematic = true;
        joint4.isKinematic = true;
        gripper_right.isKinematic = true;
        gripper_left.isKinematic = true;

        base_link.isKinematic = true;
        wheel_left_link.isKinematic = true;
        wheel_right_link.isKinematic = true;
        caster_back_right_link.isKinematic = true;
        caster_back_left_link.isKinematic = true;


        if (Time.timeScale != 20)
        {
            Time.timeScale = 20;
        }


        if (robot == null || joint1 == null || joint2 == null || joint3 == null  || joint4 == null)
        {
            Debug.LogError("Some robot or joint objects were not found!");
            return;
        }
        else
        {
            Debug.Log("Robot and joints found successfully.");
        }

        // UI 요소 null 체크
        if (colorbutton == null)
        {
            Debug.LogError("UI 요소가 올바르게 설정되지 않았습니다.");
            return; // UI 요소가 설정되지 않은 경우 함수 종료
        }

        // color 버튼 클릭 시 Box Color 변경
        colorbutton.onClick.AddListener(ChangeColor);
    }

    void Update()
    {
        UpdateRobotMove();
        UpdateRobotJoints();
        UpdateCameraPosition();
        UpdateGripper();

    }

    // color 버튼 클릭 시 실행될 함수
    void ChangeColor()
    {
        Debug.Log("Change Color Button Clicked!");

        int first = random.Next(4);
        int second = random.Next(4);
        while(first == second)
            second = random.Next(4);

        for(int i=0; i<4; ++i) {
            if(i == first || i == second) {
                ChangeColorRigid(Color.blue, cubes[i]);
            } else {
                ChangeColorRigid(Color.red, cubes[i]);
            }
        }
    }

    public void ChangeColorRigid(Color color, Rigidbody rb)
    {
        // targetObject의 Renderer를 통해 색상 변경
        Renderer renderer = rb.GetComponent<Renderer>();
        
        if (renderer != null)
        {
            renderer.material.color = color;
            //Debug.Log($"{gameObject.name} color changed to {color}");
        }
        else
        {
            Debug.LogWarning("Renderer component not found on the target object.");
        }
    }

    public void ChangeColor(Color color, GameObject gameObject)
    {
        // targetObject의 Renderer를 통해 색상 변경
        Renderer objectRenderer = gameObject.GetComponent<Renderer>();
        
        if (objectRenderer != null)
        {
            objectRenderer.material.color = color;
            //Debug.Log($"{gameObject.name} color changed to {color}");
        }
        else
        {
            Debug.LogWarning("Renderer component not found on the target object.");
        }
    }

    void UpdateCameraPosition()
    {
        // 카메라의 목표 위치 = Robot의 위치 + 로봇의 뒤쪽 방향 * 거리 + 오프셋
        Vector3 targetPosition = robot.transform.position - robot.transform.forward * 1f + cameraOffset;

        // 카메라를 목표 위치로 부드럽게 이동
        mainCamera.transform.position = Vector3.Lerp(mainCamera.transform.position, targetPosition, cameraMoveSpeed * Time.deltaTime);

        // 로봇의 뒤쪽을 바라보도록 카메라의 회전을 설정
        Quaternion lookRotation = Quaternion.LookRotation(robot.transform.position - mainCamera.transform.position);
        mainCamera.transform.rotation = Quaternion.Slerp(mainCamera.transform.rotation, lookRotation, cameraMoveSpeed * Time.deltaTime);

        // 디버깅 로그로 카메라 위치 확인
        //Debug.Log($"Camera moved to position: {robot.transform.position}");
    }


    private void ConveyorCallback(std_msgs.String message)
    {
        try
        {
            // JSON 파싱
            Dictionary<string, object> command = JsonConvert.DeserializeObject<Dictionary<string, object>>(message.data);
            
            // control 값 추출
            string control = command["control"].ToString();

            if (control == "go")
            {
                // distance.mm 값 추출
                float distance_mm = float.Parse(command["distance.mm"].ToString());
                ConveyorMove.moveSpeed = conveyorSpeed; // 이동 속도 설정
                ConveyorMove.targetDistance = distance_mm;
                ConveyorMove.totalMovedDistance = 0f;
                Debug.Log($"conveyorSpeed: {ConveyorMove.moveSpeed},  distance_mm: {distance_mm}");

            } 
            
            if (control == "stop")
            {
                ConveyorMove.moveSpeed = 0f; // 이동 속도 0
                Debug.Log($"conveyorSpeed: {ConveyorMove.moveSpeed}");

            } 

        }
        catch (System.Exception e)
        {
            Debug.Log("JSON 파싱 실패: " + e.Message);
        }
    }


    private void JointStateCallback(sens_msgs.JointState jointState)

    {
        if (jointState.name != null && jointState.name.Length > 0)    // null과 길이를 체크합니다.
        
        {
            jointName = jointState.name;                                     // jointState.name을 jointNames에 할당
            Position = Array.ConvertAll(jointState.position, x => (float)x);
            //Velocity = Array.ConvertAll(jointState.velocity, x => (float)x);
            //Effort = Array.ConvertAll(jointState.effort, x => (float)x);

            //Debug.Log("Received Joint State Names: " + string.Join(", ", jointName));
            //Debug.Log("Received Joint State Position: " + string.Join(", ", Position));
            //Debug.Log("Received Joint State velocity: " + string.Join(", ", Velocity));
            //Debug.Log("Received Gripper Effort: " + string.Join(", ", Effort));

        }
        else
        {
            //Debug.LogWarning("Received JointState has a null or empty name array.");
        }
    }

    private void OdometryCallback(nav_msgs.Odometry odometryMsg)
    {
        // 받은 오도메트리 메시지 처리
        positionX = (float)odometryMsg.pose.pose.position.x;
        positionY = (float)odometryMsg.pose.pose.position.y;
        positionZ = (float)odometryMsg.pose.pose.position.z;

        orientationX = (float)odometryMsg.pose.pose.orientation.x;
        orientationY = (float)odometryMsg.pose.pose.orientation.y;
        orientationZ = (float)odometryMsg.pose.pose.orientation.z;
        orientationW = (float)odometryMsg.pose.pose.orientation.w;

        // 받은 데이터 출력 (디버깅용)
        //Debug.Log($"Received position: {positionX}, {positionY}, {positionZ}");
        //Debug.Log($"Received orientation: {orientationX}, {orientationY}, {orientationZ}, {orientationW}");
    }

    void UpdateRobotJoints()
    {
        if (jointName != null && jointName.Length > 0) 
        {

            if (jointName[1] == "joint2")
            {

                float newRotation = Position[1] * Mathf.Rad2Deg;
                Vector3 targetRotationEuler = new Vector3(newRotation, 0, 0);
                Quaternion targetRotation = Quaternion.Euler(targetRotationEuler);

                // 현재 회전값
                Quaternion currentRotation = joint2.transform.localRotation;
                //joint2.transform.localRotation = targetRotation;

                joint2.transform.localRotation = Quaternion.RotateTowards(currentRotation, targetRotation, jointVelocity * Time.deltaTime); 


            }

            if (jointName[3] == "joint1")
            {
                // 목표 회전값 계산 (Position은 -3.14 ~ 3.14 rad)
                float newRotation = Position[3] * Mathf.Rad2Deg; // rad → degree 변환 (-180° ~ 180°)
                Vector3 targetRotationEuler = new Vector3(0, -newRotation, 0);
                Quaternion targetRotation = Quaternion.Euler(targetRotationEuler);

                // 현재 회전값
                Quaternion currentRotation = joint1.transform.localRotation;
                //joint1.transform.localRotation = targetRotation;

                joint1.transform.localRotation = Quaternion.RotateTowards(currentRotation, targetRotation, jointVelocity * Time.deltaTime); 

            }
            
            
            if (jointName[4] == "joint4")
            {
                float newRotation = Position[4] * Mathf.Rad2Deg;

                Vector3 targetRotationEuler = new Vector3(newRotation, 0, 0);
                Quaternion targetRotation = Quaternion.Euler(targetRotationEuler);
                
                // 현재 회전값
                Quaternion currentRotation = joint4.transform.localRotation;
                //joint4.transform.localRotation = targetRotation;

                joint4.transform.localRotation = Quaternion.RotateTowards(currentRotation, targetRotation, jointVelocity * Time.deltaTime); 

            }


            if (jointName[7] == "joint3")
            {
                float newRotation = Position[7] * Mathf.Rad2Deg;

                Vector3 targetRotationEuler = new Vector3(newRotation, 0, 0);
                Quaternion targetRotation = Quaternion.Euler(targetRotationEuler);
                
                // 현재 회전값
                Quaternion currentRotation = joint3.transform.localRotation;
                //joint3.transform.localRotation = targetRotation;

                // 회전을 천천히 보간        
                joint3.transform.localRotation = Quaternion.RotateTowards(currentRotation, targetRotation, jointVelocity * Time.deltaTime); 
           
            }


            if (jointName[5] == "gripper_left_joint")
            {            
                // Gripper open, close 선택
                //if (Position[5] >  0.0f && boxhold == true)       //open 0.01 close 0.0
                //if (Position[5] <=  0.0f && boxhold == true)     //open -0.01  close 0.01
                if (Position[5] >  0.0f && boxhold == true)        //open 0.01  close -0.01

                { 
                    // Griper Open
                    DetachFromParent();
                    boxhold = false;
                    Debug.Log($"Applied to {jointName[5]}: Open = {jointName[5]}");
                }
                // Gripper open, close 선택
                //if (Position[5] <= 0.0f && boxhold == false)      //open 0.01 close 0.0
                //if (Position[5] > 0.0f && boxhold == false)       //open -0.01  close 0.01
                if (Position[5] <= 0.009f && boxhold == false)         //open 0.01  close -0.01
                {        
                    // Griper close
                    SetNewParent();
                    boxhold = true;
                    Debug.Log($"Applied to {jointName[5]}: Close = {jointName[5]}");
                }
            }
        }
        else
        {
             //Debug.LogWarning("Received JointState has a null or empty name array.");
        }

    }


    void UpdateRobotMove()
    {
        // 목표 위치와 회전 설정
        Vector3 targetPosition = new Vector3(-positionY * meterScale + robotOffset.x, 
                                            positionZ * meterScale + robotOffset.y, 
                                            positionX * meterScale + robotOffset.z); // Y와 Z 교환
        Quaternion targetRotation = new Quaternion(orientationX, -orientationZ, orientationY, orientationW); // 회전 좌표계 변환

        // 현재 위치 저장 (이전 프레임과 비교용)
        Vector3 previousPosition = robot.transform.position;

        // 위치 보간 (선형 이동)
        robot.transform.position = Vector3.MoveTowards(robot.transform.position, targetPosition, moveSpeed * Time.deltaTime);

        // 회전 보간 (회전 속도 제한)
        robot.transform.rotation = Quaternion.RotateTowards(robot.transform.rotation, targetRotation, rotateSpeed * Time.deltaTime);

        // 바퀴 회전 처리
        RotateWheels(previousPosition, robot.transform.position);
    }

    void RotateWheels(Vector3 oldPos, Vector3 newPos)
    {
        float distanceMoved = Vector3.Distance(oldPos, newPos); // 로봇이 이동한 거리
        float wheelRadius = 0.05f*meterScale; // 바퀴 반지름 (meter)
        float rotationAngle = (distanceMoved / wheelRadius) * Mathf.Rad2Deg; // 회전 각도 (degree)

        // 전진/후진 방향 계산
        Vector3 moveDirection = newPos - oldPos; // 이동 방향 벡터
        float forwardDot = Vector3.Dot(moveDirection.normalized, robot.transform.forward); 

        if (forwardDot < 0) // 후진일 경우
        {
            rotationAngle *= -1; // 바퀴 회전 방향 반대로 설정
        }

        // 바퀴가 X축을 중심으로 회전하도록 설정
        wheel_left_link.transform.Rotate(Vector3.down, rotationAngle);
        wheel_right_link.transform.Rotate(Vector3.down, rotationAngle);
    }


    void UpdateGripper()
    {
        if (jointName != null && jointName.Length > 0) 
        {
            // 오른쪽 그리퍼 이동
            Vector3 currentPosR = gripper_right.transform.localPosition;

            // Gripper open, close 선택
            //Vector3 targetPosR = new Vector3(Position[6]*0.81f+0.021f, currentPosR.y, currentPosR.z);     //open 0.01 close 0.0
            //Vector3 targetPosR = new Vector3(Position[6]*0.81f-0.035f, currentPosR.y, currentPosR.z);     //open -0.01  close 0.01
            Vector3 targetPosR = new Vector3(Position[6]*0.65f+0.021f, currentPosR.y, currentPosR.z);       //open 0.01 close -0.01


            //gripper_right.transform.localPosition = Vector3.MoveTowards(currentPosR, targetPosR, Time.deltaTime);
            gripper_right.transform.localPosition = targetPosR;


            // 왼쪽 그리퍼 이동
            Vector3 currentPosL = gripper_left.transform.localPosition;

            // Gripper open, close 선택
            //Vector3 targetPosL = new Vector3(-Position[5]*0.81f-0.021f, currentPosL.y, currentPosL.z);    //open 0.01 close 0.0
            //Vector3 targetPosL = new Vector3(-Position[5]*0.81f+0.035f, currentPosL.y, currentPosL.z);    //open -0.01  close 0.01
            Vector3 targetPosL = new Vector3(-Position[5]*0.65f-0.021f, currentPosL.y, currentPosL.z);      //open 0.01 close -0.01


            //gripper_left.transform.localPosition = Vector3.MoveTowards(currentPosL, targetPosL, Time.deltaTime);
            gripper_left.transform.localPosition = targetPosL;
        }
        
    }

    void SetNewParent()
    {
        // 일정 거리 이내의 박스를 선택
        closestBox = null;

        float maxDistance =100f; // 최대 거리 (100mm)
        float minDistance = float.MaxValue;

        foreach (Rigidbody box in cubes)
        {
            float distance = Vector3.Distance(end_effector.transform.position, box.transform.position);
            //Debug.Log($"Box found within {distance} mm of the end effector.");

            if (distance <= maxDistance && distance < minDistance)
            {
                minDistance = distance;
                closestBox = box;

            }
        }

        // 기준 거리 이내에 박스가 없으면 동작하지 않음
        if (closestBox == null)
        {
            //Debug.Log($"No boxes found within {maxDistance} mm of the end effector.");
            return;
        }        

        if (closestBox == cubes[4])
        {
        foreach (Rigidbody box1 in cubes)
            {
                // 바구니와 박스 사이의 거리 계산
                float distance = Vector3.Distance(basketbase.transform.position, box1.transform.position);
                //Debug.Log($"distance: {distance}");

                // 일정 거리 이내의 박스만 바구니의 자식으로 설정
                float maxDistance1 = 150f; // basketbase의 반경 (150mm)
                if (distance <= maxDistance1)
                {
                    // 박스의 Rigidbody를 kinematic으로 설정 (물리 영향 방지)
                    Rigidbody boxRigidbody = box1.GetComponent<Rigidbody>();
                    if (boxRigidbody != null)
                    {
                        boxRigidbody.isKinematic = true;
                    }

                    // 박스를 바구니의 자식으로 설정
                    box1.transform.SetParent(closestBox.transform,true);

                    // 바구니 내부에 상대 위치 초기화 (필요 시)
                    //box.transform.localPosition = Vector3.zero;

                    Debug.Log($"{box1.name} has been attached to {closestBox.name}");
                }
            } 
        }

        // end_effector의 월드 회전값 가져오기
        Quaternion endEffectorWorldRotation = end_effector.transform.rotation;

        // end_effector의 월드 회전값을 Euler 각도로 변환
        Vector3 endEffectorEuler = endEffectorWorldRotation.eulerAngles;
        //Debug.Log($"endEffectorEuler: {endEffectorEuler}");

        // 일정 거리 이내의 박스를 end_effector의 자식으로 설정하고 로컬 위치를 (0, 0, 0)으로 설정
        closestBox.isKinematic = true;
        closestBox.transform.SetParent(end_effector.transform);
        closestBox.transform.localPosition = Vector3.zero;

        if (closestBox == cubes[4])
        {
            closestBox.transform.localRotation = Quaternion.Euler(0,-endEffectorEuler.y, endEffectorEuler.x);
        }else{
            closestBox.transform.localRotation = Quaternion.Euler(0,0,0);

        }

        //Debug.Log($"{closestBox.name} within {maxDistance} mm has been set as a child of {end_effector.name}.");
    }

    void DetachFromParent()
    {
       
        if (closestBox == null)
        {
            return;
        }

        // Rigidbody
        closestBox.isKinematic = false;
        closestBox.transform.SetParent(null);
        closestBox =null;
    }

    private void OnApplicationQuit()
    {
        if (rosSocket != null)
        {
            rosSocket.Close();
        }
    }
}
