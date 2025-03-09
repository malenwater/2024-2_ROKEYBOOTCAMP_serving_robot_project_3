...
    while(rclpy.ok()):
        key_value = getkey.getkey()
        print(f"No. {key_value}")
        if key_value == '1':
            rclpy.spin_once(node)
            node.armrun = False
            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')        
            print(node.yolo_y, node.yolo_x)
            time.sleep(1)

        elif key_value == '2':
            rclpy.spin_once(node)
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '3':
            print(node.yolo_y, node.yolo_x)
            if node.yolo_x > 0 and node.yolo_y > 0:
                yolo_robot_y = node.yolo_x + right_low_y_offset
                yolo_robot_x = node.yolo_y + right_low_x_offset
            elif node.yolo_x > 0 and node.yolo_y < 0:
                yolo_robot_y = node.yolo_x + right_high_y_offset
                yolo_robot_x = node.yolo_y + right_high_x_offset
            elif node.yolo_x < 0 and node.yolo_y > 0:
                yolo_robot_y = node.yolo_x + left_low_y_offset
                yolo_robot_x = node.yolo_y + left_low_x_offset
            elif node.yolo_x < 0 and node.yolo_y < 0:
                yolo_robot_y = node.yolo_x + left_high_y_offset
                yolo_robot_x = node.yolo_y + left_high_x_offset

            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2, 0.122354)
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '4':
            pose_array = node.append_pose_init(0.14 - yolo_robot_x + 0.055 + 0.01, 0.0 - yolo_robot_y * 1.2, 0.095354)
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     
            time.sleep(1)

        elif key_value == '5':
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '6':
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '7':
            response = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '8':
            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '9':
            print(f'right low  :  yolo_robot_x [{right_low_x_offset}] yolo_robot_y[{right_low_y_offset}]')
            print(f'right high :  yolo_robot_x [{right_high_x_offset}] yolo_robot_y[{right_high_y_offset}]')
            print(f'left low   :  yolo_robot_x [{left_low_x_offset}] yolo_robot_y[{left_low_y_offset}]')
            print(f'left high  :  yolo_robot_x [{left_high_x_offset}] yolo_robot_y[{left_high_y_offset}]')

        elif key_value == '0':
            with open(file_path, "w") as file:
                file.write(f"right_low_x_offset : {right_low_x_offset}\n")
                file.write(f"right_low_y_offset : {right_low_y_offset}\n")
                file.write(f"right_high_x_offset : {right_high_x_offset}\n")
                file.write(f"right_high_y_offset : {right_high_y_offset}\n")
                file.write(f"left_low_x_offset : {left_low_x_offset}\n")
                file.write(f"left_low_y_offset : {left_low_y_offset}\n")
                file.write(f"left_high_x_offset : {left_high_x_offset}\n")
                file.write(f"left_high_y_offset : {left_high_y_offset}\n")
            print("Values saved.")

        elif key_value == 'q':
            break

    print("task end!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

