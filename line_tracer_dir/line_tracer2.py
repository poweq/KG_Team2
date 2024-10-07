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
            print(angle)

            # Send angle as numeric value via serial
            if 80 <= angle <= 100:
                ser.write(f'{angle}\n'.encode())  # Send angle value as a string
                print(f"Sending '{angle}' via serial")
            elif 40 <= angle < 80:
                ser.write(f'{angle}\n'.encode())  # Send angle value as a string
                print(f"Sending '{angle}' via serial")
            elif 101 <= angle <= 120:
                ser.write(f'{angle}\n'.encode())  # Send angle value as a string
                print(f"Sending '{angle}' via serial")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("cap error")

# Release resources
cap.release()
video_orig.release()
cv2.destroyAllWindows()
ser.close()

