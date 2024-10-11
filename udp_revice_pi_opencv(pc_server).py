# -*- coding:utf-8 -*-
import cv2
import numpy as np
import math
import socket
import pickle
import struct

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

while True:
    try:
        # Receive video data from Raspberry Pi
        video_data, video_addr = video_sock.recvfrom(65507)
        video_size = struct.unpack("Q", video_data[:8])[0]
        video_frame_data = video_data[8:8 + video_size]
        img = pickle.loads(video_frame_data)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)

        if img is None:
            print("Failed to decode frame, skipping this frame.")
            continue
    except Exception as e:
        print(f"Error receiving video: {e}")
        continue

    height, width, _ = img.shape
    center_x = width // 2
    center_y = height

    img_cvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_mask1 = cv2.inRange(img_cvt, np.array([0, 100, 100]), np.array([20, 255, 255]))
    img_mask2 = cv2.inRange(img_cvt, np.array([160, 100, 100]), np.array([180, 255, 255]))
    yellow_mask = cv2.inRange(img_cvt, np.array([15, 100, 100]), np.array([35, 255, 255]))
    img_mask = img_mask1 + img_mask2 + yellow_mask

    cont_list, hierachy = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    try:
        c = max(cont_list, key=cv2.contourArea)
        M = cv2.moments(c)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # 원점에서 무게중심까지 선 그리기
        img = cv2.line(img, (center_x, center_y), (cx, cy), (255, 0, 0), 2)

        # 각도 계산 (x축 기준으로 왼쪽이 180도, 오른쪽이 0도)
        delta_x = cx - center_x
        delta_y = center_y - cy
        angle = int(math.degrees(math.atan2(delta_y, delta_x)))
        if angle < 0:
            angle += 360
        angle = 180 - angle

        # 결과 출력
        print(f"Detected angle: {angle} degrees")

        # Send angle data to Raspberry Pi
        try:
            angle_text = f"{angle}"
            angle_send_sock.sendto(angle_text.encode(), (pi_ip, angle_send_port))
        except Exception as e:
            print(f"Error sending angle data: {e}")

        # 외곽선과 중심점 그리기
        img_con = cv2.drawContours(img, [c], -1, (0, 0, 255), 1)
        img = cv2.circle(img, (cx, cy), 10, (0, 255, 0), -1)
    except:
        pass

    # 화면에 x축과 y축 그리기
    img = cv2.line(img, (0, center_y), (width, center_y), (0, 255, 255), 1)  # x축
    img = cv2.line(img, (center_x, 0), (center_x, height), (0, 255, 255), 1)  # y축

    cv2.imshow('mask', img)
    key = cv2.waitKey(1)
    if key & 0xff == ord('q'):
        break

cv2.destroyAllWindows()
video_sock.close()
angle_send_sock.close()