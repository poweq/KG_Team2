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

# Function to detect lane
def detect_curve_lane(frame):
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

        # Check if the coordinates are valid before drawing the line
        if isinstance(lefty, int) and isinstance(righty, int):
            try:
                cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)
            except Exception as e:
                print(f"Error drawing line: {e}")

        angle = calculate_angle(0, lefty, width - 1, righty, width // 2)

        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None

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

        # Detect lane and calculate angle
        lane_center, lane_image, angle = detect_curve_lane(frame)

        height, width = frame.shape[:2]
        frame_center_x = width // 2

        # Check if lane is detected and set angle
        if lane_center is None:
            print("Lane not found.")
            angle_text = "Angle: N/A"
            combined_image = frame  # Display original frame if no lane is detected
        else:
            # Set angle text
            angle_text = f"Angle: {angle}°"
            print(f"Detected angle: {angle_text}")  # Print angle to console

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
        cv2.line(combined_image, (0, height - 1), (width, height - 1), (255, 0, 0), 2)

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
