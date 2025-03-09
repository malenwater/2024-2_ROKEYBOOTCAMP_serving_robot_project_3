using System.Collections.Generic;
using UnityEngine;

public class ConveyorMove : MonoBehaviour
{
    public static float moveSpeed; // 컨베이어 이동 속도
    public static float targetDistance = 0f;

    private List<Rigidbody> boxesOnConveyor = new List<Rigidbody>(); // 컨베이어 위의 박스 리스트
    public static float totalMovedDistance = 0f; // 컨베이어가 이동한 총 거리

    private void OnCollisionEnter(Collision collision)
    {
        Rigidbody boxRigidbody = collision.rigidbody;

        if (boxRigidbody != null && !boxesOnConveyor.Contains(boxRigidbody))
        {
            // 새로운 박스를 리스트에 추가
            boxesOnConveyor.Add(boxRigidbody);
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        Rigidbody boxRigidbody = collision.rigidbody;

        if (boxRigidbody != null && boxesOnConveyor.Contains(boxRigidbody))
        {
            // 박스를 리스트에서 제거
            boxesOnConveyor.Remove(boxRigidbody);
        }
    }

    private void FixedUpdate()
    {
        // 컨베이어가 아직 targetDistance만큼 이동하지 않았다면 계속 이동
        if (totalMovedDistance < targetDistance)
        {
            //GravityRigidRobot.ChangeColor(Color.red, GravityRigidRobot.conveyor);
            totalMovedDistance += moveSpeed * Time.fixedDeltaTime;

            foreach (Rigidbody box in boxesOnConveyor)
            {
                // 박스를 Z 방향으로 이동
                Vector3 movement = new Vector3(0, 0, -moveSpeed * Time.fixedDeltaTime);
                box.MovePosition(box.position + movement);
            }

            // 현재 이동 상태 출력
            //Debug.Log($"Total Moved: {totalMovedDistance}, Target: {targetDistance}");
        }
        else
        {
            //GravityRigidRobot.ChangeColor(Color.green, GravityRigidRobot.conveyor);

        }
    }
}
