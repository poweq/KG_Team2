# -*- coding:utf-8 -*-
import socket
import cv2
import numpy as np
import pickle
import struct
import math

# UDP settings
video_receive_port = 5005  # Port number to receive video data from Raspberry Pi
angle_send_port = 6000  # Port number to send angle data to Raspberry Pi
pi_ip = "172.30.1.240"  # Raspberry Pi IP address (destination for sending angle data)

# Create socket for receiving video data and bind it
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", video_receive_port))

# Create socket for sending angle data
angle_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Waiting for video data from Raspberry Pi...")

# Function to calculate lane angle
def calculate_angle(x1, y1, x2, y2):
    dy = y2 - y1
    dx = x1 - x2
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    
    if angle_degrees < 0:
        angle_degrees += 180

    return int(angle_degrees)

# Function to detect lane
def detect_curve_lane(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Improved HSV range for better yellow detection
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Morphological operations to reduce noise and enhance detection
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

    filtered_frame = cv2.bitwise_and(frame, frame, mask=yellow_mask)

    gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.ones_like(edges) * 255  # ROI를 전체 화면으로 설정
    masked_edges = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 30, 
                            minLineLength=20, maxLineGap=10)

    if lines is None:
        return None, None, None

    points = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            points.append([x1, y1])
            points.append([x2, y2])

    points = np.array(points)
    if len(points) > 0:
        # Calculate bounding box that includes all detected points
        min_x = np.min(points[:, 0])
        max_x = np.max(points[:, 0])
        min_y = np.min(points[:, 1])
        max_y = np.max(points[:, 1])

        # Draw the extended bounding box around the detected lane
        cv2.rectangle(frame, (min_x, 0), (max_x, 255), (0, 255, 255), 2)

        bounding_box_width = max_x - min_x

        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        
        try:
            if vx == 0:
                return None, None, None
            lefty = int((-x * vy / vx) + y)
            righty = int(((width - x) * vy / vx) + y)
        except (ZeroDivisionError, ValueError) as e:
            print(f"Error calculating lefty or righty: {e}")
            return None, None, None

        line_image = np.zeros_like(frame)
        scale = 100
        end_x = width // 2 + int(vx * scale)
        end_y = height - int(vy * scale)
        start_x = width // 2 - int(vx * scale)
        start_y = height + int(vy * scale)
        
        try:
            cv2.line(line_image, (start_x, start_y), (end_x, end_y), (0, 255, 0), 5)
        except Exception as e:
            print(f"Error drawing line: {e}")

        angle = calculate_angle(width // 2, height, end_x, end_y)

        return (width // 2, height), line_image, angle, bounding_box_width
    else:
        return None, None, None, None

try:
    while True:
        try:
            video_data, video_addr = video_sock.recvfrom(65507)
            video_size = struct.unpack("Q", video_data[:8])[0]
            video_frame_data = video_data[8:8 + video_size]
            frame = pickle.loads(video_frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            
            if frame is None:
                print("Failed to decode frame, skipping this frame.")
                continue
        except Exception as e:
            print(f"Error receiving video: {e}")
            continue

        lane_center, lane_image, angle, bounding_box_width = detect_curve_lane(frame)
        height, width = frame.shape[:2]
        frame_center_x = width // 2

        if lane_center is None:
            print("Lane not found.")
            angle_text = "Angle: N/A"
            combined_image = frame
        else:
            angle_text = f"Angle: {angle}, Width: {bounding_box_width}"
            print(f"Detected angle: {angle}, Width: {bounding_box_width}")

            try:
                angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))
            except Exception as e:
                print(f"Error sending angle data: {e}")

            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

        cv2.line(combined_image, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)

        cv2.putText(combined_image, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if combined_image is not None:
            cv2.imshow("Lane Detection", combined_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    cv2.destroyAllWindows()
    video_sock.close()
    angle_send_sock.close()
    print("Program terminated successfully.")