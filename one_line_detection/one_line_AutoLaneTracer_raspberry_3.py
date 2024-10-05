import cv2
import numpy as np
import math
import serial
import time

# Serial communication setup (set port and baud rate)
ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)

# Camera setup (if the webcam ID is 0)
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Set frame width
cap.set(4, 480)  # Set frame height

# Add delay of 5 seconds before starting motor control
print("waiting... 5sec")
time.sleep(5)
print("starting motor control.")

# Function to calculate lane angle
def calculate_angle(x1, y1, x2, y2, frame_center_x):
    dy = y1 - y2
    dx = x2 - x1
    angle_radians = math.atan2(dy, dx)

    # Convert angle to positive (range from 0 to 180 degrees)
    angle_degrees = math.degrees(angle_radians)
    
    # atan2 provides range from -180 to 180, adjust to 0 to 180
    if angle_degrees < 0:
        angle_degrees += 180
    
    # Adjust so that the left side is 0 degrees and the right side is 180 degrees based on the frame
    if x2 >= frame_center_x:  # When on the right side
        angle_degrees = 180 - angle_degrees
    else:  # When on the left side
        angle_degrees = angle_degrees

    return int(angle_degrees)  # Return the angle as an integer

# Function for lane detection
def detect_curve_lane(frame):
    # Convert to HSV and filter colors (white and yellow lanes)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))  # Filter white
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))  # Filter yellow
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    filtered_frame = cv2.bitwise_and(frame, frame, mask=combined_mask)

    # Convert filtered frame to grayscale and apply Gaussian Blur
    gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Define a region of interest (ROI)
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[ 
        (0, height),
        (width, height),
        (int(width * 0.6), int(height * 0.6)),
        (int(width * 0.4), int(height * 0.6))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(edges, mask)

    # Hough Line detection with adjusted parameters
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 30, minLineLength=30, maxLineGap=200)

    if lines is None:
        return None, None, None

    points = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            points.append([x1, y1])
            points.append([x2, y2])

    points = np.array(points)
    if len(points) > 0:
        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        
        try:
            lefty = int((-x * vy / vx) + y)
            righty = int(((width - x) * vy / vx) + y)
        except (ZeroDivisionError, ValueError) as e:
            print(f"Error calculating lefty or righty: {e}")
            return None, None, None

        line_image = np.zeros_like(frame)

        # Draw the line if the coordinates are valid
        if isinstance(lefty, int) and isinstance(righty, int):
            cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)

        # Calculate the angle
        angle = calculate_angle(0, lefty, width - 1, righty, width // 2)

        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None

# Function to calculate motor speeds
def calculate_motor_values(angle):
    base_speed = 100
    turn_speed = 50
    motorA = motorB = motorC = motorD = base_speed

    left_boost = 60
    right_boost = 60

    if angle < 85:  # Left turn
        motorA = turn_speed
        motorB = turn_speed
        motorC = turn_speed + right_boost
        motorD = turn_speed + right_boost
    elif angle > 97:  # Right turn
        motorA = turn_speed + left_boost
        motorB = turn_speed + left_boost
        motorC = turn_speed
        motorD = turn_speed
    else:  # Straight
        motorA = base_speed
        motorB = base_speed 
        motorC = base_speed
        motorD = base_speed

    # Adjust motor A and B to match the lower speed
    min_AB = min(motorA, motorB)
    motorA = motorB = min_AB

    # Adjust motor C and D to match the lower speed
    min_CD = min(motorC, motorD)
    motorC = motorD = min_CD

    return motorA, motorB, motorC, motorD

# Main loop
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera error")
            break

        height, width = frame.shape[:2]
        frame_center_x = width // 2

        # Detect lane and calculate angle
        lane_center, lane_image, angle = detect_curve_lane(frame)

        if lane_center is None:
            print("Cannot find lane.")
            continue

        # Calculate motor speeds based on angle
        motorA, motorB, motorC, motorD = calculate_motor_values(angle)

        # Delay for 2 seconds before sending motor commands
        time.sleep(2)

        # Send motor commands via serial communication
        motor_command = f'a:{motorA} b:{motorB} c:{motorC} d:{motorD}\n'
        ser.write(motor_command.encode())
        print(f"Sending motor command: {motor_command}")

        # Draw a virtual vertical center line
        cv2.line(frame, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)

        # Display angle and distance between lane and center
        angle_text = f"Angle: {angle}"  # Output integer angle
        cv2.putText(frame, angle_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Overlay the lane image with the center line
        if lane_image is not None:
            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)
        else:
            combined_image = frame

        # Show the result
        cv2.imshow('Lane Detection with Adjusted Angle', combined_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Send motor control command to set all motors to 0 before exiting
    motor_command = 'a:0 b:0 c:0 d:0\n'
    ser.write(motor_command.encode())
    print(f"Sending motor stop command: {motor_command}")

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
