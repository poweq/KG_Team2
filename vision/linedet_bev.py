import cv2
import os
import numpy as np
from jd_opencv_lane_detect import JdOpencvLaneDetect

# Initialize lane detector
cv_detector = JdOpencvLaneDetect()

# Capture video from webcam
cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)

# Create data directory if it doesn't exist
if not os.path.exists('./data'):
    os.makedirs('./data')

# Set up video writer
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_orig = cv2.VideoWriter('./data/car_video.avi', fourcc, 20.0, (320, 240))

def bird_eye_view_transform(image):
    height, width = image.shape[:2]
    
    # Define source and destination points for perspective transform
    src_points = np.float32([[0, height], [width, height], [width * 0.55, height * 0.6], [width * 0.45, height * 0.6]])
    dst_points = np.float32([[0, height], [width, height], [width, 0], [0, 0]])
    
    # Compute perspective transform matrix and apply it
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    bird_eye_view = cv2.warpPerspective(image, matrix, (width, height))
    
    return bird_eye_view

# Prepare camera
for i in range(30):
    ret, img_org = cap.read()
    if not ret:
        print("Camera error")
        break

while True:
    ret, img_org = cap.read()
    if not ret:
        print("Cap error")
        break

    # Write original camera image to video
    video_orig.write(img_org)
    
    # Apply bird's eye view transformation
    bird_eye_view_img = bird_eye_view_transform(img_org)

    # Detect lanes and calculate angle on the bird's eye view image
    lanes, img_lane = cv_detector.get_lane(bird_eye_view_img)
    angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
    
    # Display the lane detection results
    if img_angle is not None:
        cv2.imshow('Lane Detection', img_angle)
        print("Steering angle:", angle)
    else:
        print("Can't find lane...")

    # Show the bird's eye view image
    cv2.imshow('Bird\'s Eye View', bird_eye_view_img)

    # Break loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
video_orig.release()
cv2.destroyAllWindows()
