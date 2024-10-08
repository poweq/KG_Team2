import cv2
import numpy as np

# 차선 각도 계산 함수
def calculate_angle(x1, y1, x2, y2, frame_center_x):
    dy = y1 - y2
    dx = x2 - x1

    angle_radians = np.arctan2(dy, dx)
    angle_degrees = np.degrees(angle_radians)
    
    if angle_degrees < 0:
        angle_degrees += 180
    
    if x2 >= frame_center_x:
        angle_degrees = 180 - angle_degrees
    
    return int(angle_degrees)

# 차선 검출 함수
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

        # 좌표값이 유효한지 확인한 후 선을 그립니다
        if isinstance(lefty, int) and isinstance(righty, int):
            try:
                cv2.line(line_image, (width - 1, righty), (0, lefty), (0, 255, 0), 5)
            except Exception as e:
                print(f"Error drawing line: {e}")

        angle = calculate_angle(0, lefty, width - 1, righty, width // 2)

        return (width // 2, (lefty + righty) // 2), line_image, angle
    else:
        return None, None, None
