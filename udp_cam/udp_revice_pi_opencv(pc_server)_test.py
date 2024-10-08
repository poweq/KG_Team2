import socket
import cv2
import numpy as np
import pickle
import struct
import math

# UDP settings
video_receive_port = 5005  # Port number to receive video data from Raspberry Pi
angle_send_port = 6000  # Port number to send angle data to Raspberry Pi
pi_ip = "172.30.1.100"  # Raspberry Pi IP address (destination for sending angle data)

# Create socket for receiving video data and bind it
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", video_receive_port))

# Create socket for sending angle data
angle_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Waiting for video data and ready to send angle...")

# Function to calculate lane angle
def calculate_angle(x1, y1, x2, y2, frame_center_x):
    dy = y1 - y2
    dx = x2 - x1
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)
    
    if angle_degrees < 0:
        angle_degrees += 180
    
    if x2 >= frame_center_x:
        angle_degrees = 180 - angle_degrees
    
    return int(angle_degrees)

# Function to detect both left and right lanes 
def detect_lanes(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))  # White lane filter
    yellow_mask = cv2.inRange(hsv, (15, 100, 100), (35, 255, 255))  # Yellow lane filter
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    filtered_frame = cv2.bitwise_and(frame, frame, mask=combined_mask)

    gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 50, 150)

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

    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 30, minLineLength=30, maxLineGap=200)

    if lines is None:
        return None, None, None

    left_lane = []
    right_lane = []
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # To avoid division by zero
            intercept = y1 - slope * x1
            if slope < 0:  # Left lane
                left_lane.append((slope, intercept, x1, y1, x2, y2))
            else:  # Right lane
                right_lane.append((slope, intercept, x1, y1, x2, y2))

    lane_image = np.zeros_like(frame)
    
    def draw_lane(lane_points):
        if len(lane_points) > 0:
            slope_avg, intercept_avg = np.mean(lane_points, axis=0)[:2]
            y1 = height
            y2 = int(height * 0.6)
            x1 = int((y1 - intercept_avg) / slope_avg)
            x2 = int((y2 - intercept_avg) / slope_avg)
            cv2.line(lane_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
            return (x1, y1), (x2, y2)
        return None

    left_points = draw_lane(left_lane)
    right_points = draw_lane(right_lane)

    if left_points and right_points:
        left_angle = calculate_angle(*left_points[0], *left_points[1], width // 2)
        right_angle = calculate_angle(*right_points[0], *right_points[1], width // 2)
        return lane_image, (left_angle, right_angle)
    else:
        return None, None

try:
    while True:
        # Receive video data
        try:
            video_data, video_addr = video_sock.recvfrom(65507)  # Maximum UDP packet size
            video_size = struct.unpack("Q", video_data[:8])[0]   # Read the size of the data
            video_frame_data = video_data[8:8 + video_size]      # Separate the actual video data
            frame = pickle.loads(video_frame_data)               # Deserialize
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)        # Convert JPEG to OpenCV image
        except Exception as e:
            print(f"Error receiving video: {e}")
            continue  # Keep running even if an error occurs

        # Detect lanes and calculate angles
        lane_image, angles = detect_lanes(frame)

        height, width = frame.shape[:2]
        frame_center_x = width // 2

        # Check if lanes are detected and set angle
        if angles is None:
            print("Lanes not found.")
            angle_text = "Angle: N/A"
            combined_image = frame  # Display original frame if no lanes are detected
        else:
            left_angle, right_angle = angles
            angle_text = f"Left: {left_angle}°, Right: {right_angle}°"
            print(f"Detected angles: {angle_text}")  # Print angles to console

            # Generate image with lane and angle information
            combined_image = cv2.addWeighted(frame, 0.8, lane_image, 1, 1)

            # Send angle data to Raspberry Pi
            try:
                angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))  # Send angle data
            except Exception as e:
                print(f"Error sending angle data: {e}")

        # Draw a virtual vertical center line
        cv2.line(combined_image, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)

        # Draw a virtual horizontal center line (at the bottom of the screen)
        cv2.line(combined_image, (0, height - 1), (width, height - 1), (0, 255, 0), 2)

        # Display the angle on the image
        cv2.putText(combined_image, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Show the result (even if no lane is detected)
        cv2.imshow("Lane Detection", combined_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Release resources
    cv2.destroyAllWindows()
    video_sock.close()
    angle_send_sock.close()
    print("Program terminated successfully.")
