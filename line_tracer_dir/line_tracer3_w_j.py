import os
import socket
import serial

# Serial communication setup
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)

# Create directory for saving video if it doesn't exist
try:
    if not os.path.exists('./data'):
        os.makedirs('./data')
except OSError:
    print("failed to make ./data folder")

# Function to calculate motor values based on angle
def calculate_motor_values(angle):
    # Base speed for motors
    base_speed = 110

    # Initialize motor speeds
    motorA = motorB = motorC = motorD = base_speed

    if angle < 86:
        # Turn left: reduce speed of motors on the left side and increase speed on the right side
        motorA = base_speed - (86 - angle)  # Slow down left front motor
        motorB = base_speed - (86 - angle)  # Slow down left back motor
        motorC = base_speed + (86 - angle)   # Speed up right front motor
        motorD = base_speed + (86 - angle)   # Speed up right back motor
    elif angle > 94:
        # Turn right: reduce speed of motors on the right side and increase speed on the left side
        motorC = base_speed - (angle - 94)  # Slow down right front motor
        motorD = base_speed - (angle - 94)  # Slow down right back motor
        motorA = base_speed + (angle - 94)  # Speed up left front motor
        motorB = base_speed + (angle - 94)  # Speed up left back motor

    return motorA, motorB, motorC, motorD

# Socket setup
HOST = '0.0.0.0'
PORT = 65432

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()

print(f"{HOST}:{PORT} waiting...")

conn, addr = server_socket.accept()
print(f"{addr} connected")

print("Opened serial ttyAMA1")

try:
    while True:
        data = conn.recv(4)
        if not data:
            break
        
        received_int = int.from_bytes(data, byteorder='big')
        print(f"Received message: {received_int}")

        # Use received value as the angle for motor control
        motorA, motorB, motorC, motorD = calculate_motor_values(received_int)

        # Send motor values via serial
        motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
        ser.write(motor_command.encode())
        print(f"Sending motor command: {motor_command}")

finally:
    # Set all motors to 0 before closing
    motor_command_zero = 'a:0 b:0 c:0 d:0\n'
    ser.write(motor_command_zero.encode())
    print(f"Sending motor command to stop all motors: {motor_command_zero}")

    # Close connections
    conn.close()
    server_socket.close()
    ser.close()

