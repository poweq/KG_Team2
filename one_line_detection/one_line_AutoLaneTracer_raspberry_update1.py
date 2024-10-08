import socket
import cv2
import numpy as np
import pickle
import struct
import math
import time
import serial
from jd_opencv_lane_detect import JdOpencvLaneDetect  # jd_opencv_lane_detect 모듈 임포트

# Serial communication setup (set port and baud rate)
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)

# Delay for motor control start
print("waiting... 5sec")
time.sleep(5)
print("starting motor control.")

# UDP settings
video_receive_port = 5005  # Port for receiving video data
angle_send_port = 5007  # Port for sending angle data
pi_ip = "172.30.1.100"  # Raspberry Pi IP address (destination for sending angle data)

# Create socket for receiving video
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", video_receive_port))

# Create socket for sending angle
angle_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create an instance of the lane detection class from jd_opencv_lane_detect
lane_detector = JdOpencvLaneDetect()

print("Waiting for video data and ready to send angle...")

# Main loop
try:
    while True:
        # Receive video data
        video_data, video_addr = video_sock.recvfrom(65507)  # Max UDP packet size
        video_size = struct.unpack("Q", video_data[:8])[0]   # Extract the size of the data
        video_frame_data = video_data[8:8 + video_size]      # Get the actual frame data
        frame = pickle.loads(video_frame_data)               # Deserialize the frame
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)        # Convert from JPEG to OpenCV image

        # Detect lane and calculate angle using JdOpencvLaneDetect class
        lane_lines, lane_image = lane_detector.get_lane(frame)
        angle, _ = lane_detector.get_steering_angle(lane_image, lane_lines)

        # If lane is not detected, rotate the car based on the last known angle
        if lane_lines is None:
            print("Cannot find lane. Rotating...")
            if angle < 90:
                motor_command = 'a:50 b:50 c:-50 d:-50\n'  # Rotate right
            else:
                motor_command = 'a:-50 b:-50 c:50 d:50\n'  # Rotate left
            ser.write(motor_command.encode())
            time.sleep(1)
        else:
            # Print and send the detected angle to Raspberry Pi
            angle_text = f"Angle: {angle}"
            print(f"Detected angle: {angle_text}")
            angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))

            # Send motor commands based on the angle
            motorA, motorB, motorC, motorD = 100, 100, 100, 100  # Adjust motor values accordingly
            if angle < 85:
                motorA, motorB = 50, 50
            elif angle > 95:
                motorC, motorD = 50, 50
            motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
            ser.write(motor_command.encode())
            print(f"Sending motor command: {motor_command}")

        # Show frame with detected lane
        if lane_image is not None:
            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)
        else:
            combined_image = frame
        cv2.imshow('Lane Detection with Real-time Video', combined_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop motors before exiting
    motor_command = 'a:0 b:0 c:0 d:0\n'
    ser.write(motor_command.encode())
    print(f"Sending motor stop command: {motor_command}")

    # Release resources
    video_sock.close()
    angle_send_sock.close()
    cv2.destroyAllWindows()
    ser.close()
