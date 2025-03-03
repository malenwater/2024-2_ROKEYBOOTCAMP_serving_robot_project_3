import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Arduino 연결됨. 데이터 수신 중...")

    while True:
        raw_data = ser.read_all()  # 원본 데이터 확인
        if raw_data:
            print(f"RAW 데이터: {raw_data}")  # 원본 바이트 데이터 출력
            decoded_data = raw_data.decode(errors='replace').strip()
            print(f"디코딩된 데이터: {decoded_data}")

except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    ser.close()

# import serial
# import time
# import threading

# # Arduino가 연결된 포트 설정 (환경에 맞게 수정)
# SERIAL_PORT = "/dev/ttyACM0"  # Windows: "COMx", Linux/macOS: "/dev/ttyUSBx" or "/dev/ttyACMx"
# BAUD_RATE = 115200

# # 시리얼 포트 열기
# try:
#     ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#     time.sleep(2)  # Arduino와의 통신 안정화를 위해 대기
#     print("Arduino와 연결되었습니다. 값을 입력하세요 (예: 1000)")
# except serial.SerialException as e:
#     print(f"시리얼 포트 연결 실패: {e}")
#     exit(1)

# # 수신 스레드 함수
# def read_serial():
#     while True:
#         print("hi")
#         if ser.in_waiting > 0:
#             response = ser.readline().decode().strip()
#             if response == ".":
#                 print("✅ 보낼 수 있는 상태")
#             elif response == "_":
#                 print("📩 값이 오는 상태")
#             elif response:
#                 print(f"📨 수신됨: {response}")

# # 수신 스레드 실행
# thread = threading.Thread(target=read_serial, daemon=True)
# thread.start()

# # 메인 루프 (사용자 입력 처리)
# try:
#     while True:
#         user_input = input("전송할 스텝 수 입력: ")  
        
#         if user_input.isdigit():  # 숫자인지 확인
#             ser.write(user_input.encode())  
#             ser.write(b'\n')  # 개행 문자 추가 (필요하면 제거 가능)
#             print(f"🚀 전송됨: {user_input}")

#         else:
#             print("⚠️ 숫자만 입력하세요.")

# except KeyboardInterrupt:
#     print("\n프로그램 종료")

# finally:
#     ser.close()  # 시리얼 포트 닫기
