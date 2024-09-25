import cv2
import os
import serial
from jd_opencv_lane_detect import JdOpencvLaneDetect

# Serial communication setup
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)

# OpenCV line detector setup
cv_detector = JdOpencvLaneDetect()

# Open camera
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # Set width
cap.set(4, 240)  # Set height

# Create directory for saving video if it doesn't exist
try:
    if not os.path.exists('./data'):
        os.makedirs('./data')
except OSError:
    print("failed to make ./data folder")

# Setup video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_orig = cv2.VideoWriter('./data/car_video.avi', fourcc, 20.0, (320, 240))

# Initial frame capture
for i in range(30):
    ret, img_org = cap.read()
    if ret:
        lanes, img_lane = cv_detector.get_lane(img_org)
        angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
        if img_angle is None:
            print("can't find lane...")
            pass
        else:
            print(angle)
    else:
        print("camera error")

# Function to calculate motor values based on angle
def calculate_motor_values(angle):
    # Base speed for motors
    base_speed = 110

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

# Main loop
while True:
    ret, img_org = cap.read()
    if ret:
        # Save the frame to video
        video_orig.write(img_org)

        # Process the frame
        lanes, img_lane = cv_detector.get_lane(img_org)
        angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
        if img_angle is None:
            print("can't find lane...")
            pass
        else:
            cv2.imshow('lane', img_angle)
            print(f"Detected angle: {angle}")

            # Calculate motor values based on the angle
            motorA, motorB, motorC, motorD = calculate_motor_values(angle)

            # Send motor values via serial
            motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
            ser.write(motor_command.encode())
            print(f"Sending motor command: {motor_command}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("cap error")

# Release resources
cap.release()
video_orig.release()
cv2.destroyAllWindows()
ser.close()