import serial
import time

# Serial communication setup
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)

# Function to send motor command via serial
def send_motor_command(motorA, motorB, motorC, motorD):
    motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
    ser.write(motor_command.encode())
    print(f"Sending motor command: {motor_command}")

# Function to control motors based on user input
def keyboard_control(user_input):
    base_speed = 110
    motorA = motorB = motorC = motorD = base_speed

    if user_input == 'w':  # Forward
        motorA = motorB = motorC = motorD = base_speed
        print("Moving forward")
    elif user_input == 's':  # Backward
        motorA = motorB = motorC = motorD = -base_speed
        print("Moving backward")
    elif user_input == 'a':  # Left
        motorA = motorB = base_speed - 30  # Slow down left motors
        motorC = motorD = base_speed + 30  # Speed up right motors
        print("Turning left")
    elif user_input == 'd':  # Right
        motorC = motorD = base_speed - 30  # Slow down right motors
        motorA = motorB = base_speed + 30  # Speed up left motors
        print("Turning right")

    send_motor_command(motorA, motorB, motorC, motorD)

# Main loop (keyboard control via input)
while True:
    user_input = input("입력하세요 (w: 앞으로, s: 뒤로, a: 왼쪽, d: 오른쪽, q: 종료, x: 모터 정지): ")
    
    if user_input == 'q':  # Quit if 'q' is pressed
        print("종료합니다.")
        send_motor_command(0, 0, 0, 0)  # 모든 모터 값을 0으로 설정
        break
    elif user_input == 'x':  # Stop all motors if 'x' is pressed
        print("모터를 정지합니다.")
        send_motor_command(0, 0, 0, 0)  # 모든 모터 값을 0으로 설정
    elif user_input in ['w', 's', 'a', 'd']:  # Check for valid commands
        keyboard_control(user_input)
    else:
        print("유효하지 않은 입력입니다.")  # Invalid input message
    time.sleep(1)  # 1초 대기

# Release resources
ser.close()

