import serial
import time
import threading
import os
import glob

# Arduino가 연결된 포트 설정 (환경에 맞게 수정)
BAUD_RATE = 115200

# 시리얼 포트 열기
def open_serial():
    port = find_serial_port()
    if not port:
        return None
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)  # Arduino와의 통신 안정화를 위해 대기
        print(f"Arduino와 연결되었습니다. ({port}) 값을 입력하세요 (예: 1000)")
        return ser
    except serial.SerialException as e:
        print(f"시리얼 포트 연결 실패: {e}")
        return None

# 연결된 시리얼 포트 찾기
def find_serial_port():
    ports = glob.glob('/dev/ttyACM*')  # 연결된 /dev/ttyACM* 포트를 찾습니다.
    if ports:
        return ports[0]  # 첫 번째로 발견된 포트를 사용합니다.
    else:
        print("연결된 시리얼 포트를 찾을 수 없습니다.")
        return None

# 수신 스레드 함수
def read_serial(ser):
    while True:
        if ser:
            try:
                if ser.in_waiting > 0:
                    response = ser.readline().decode().strip()
                    if response == ".":
                        print("✅ 보낼 수 있는 상태")
                    elif response == "_":
                        print("📩 값이 오는 상태")
                    elif response:
                        print(f"📨 수신됨: {response}")
            except serial.SerialException as e:
                print(f"시리얼 포트 에러: {e}")
                ser.close()  # 포트 닫기
                return  # 에러가 나면 종료하여 다시 연결하도록 처리

# 메인 루프 (사용자 입력 처리 및 연결 상태 감지)
def main():
    ser = open_serial()

    if ser:
        # 수신 스레드 실행
        thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
        thread.start()

    try:
        while True:
            if ser is None or not ser.is_open:  # 연결이 끊어졌으면
                print("⚠️ 연결이 끊어졌습니다. USB를 다시 꽂아주세요.")
                while ser is None or not ser.is_open:
                    time.sleep(1)  # 잠시 대기 후 재시도
                    ser = open_serial()  # 시리얼 포트를 다시 열기
                    if ser:
                        print("🚀 다시 연결되었습니다.")
                        thread = threading.Thread(target=read_serial, args=(ser,), daemon=True)
                        thread.start()

            user_input = input("전송할 스텝 수 입력: ")  
            
            if user_input.isdigit():  # 숫자인지 확인
                ser.write(user_input.encode())  
                ser.write(b'\n')  # 개행 문자 추가 (필요하면 제거 가능)
                print(f"🚀 전송됨: {user_input}")
            else:
                print("⚠️ 숫자만 입력하세요.")

    except KeyboardInterrupt:
        print("\n프로그램 종료")

    finally:
        if ser:
            ser.close()  # 시리얼 포트 닫기

if __name__ == "__main__":
    main()
