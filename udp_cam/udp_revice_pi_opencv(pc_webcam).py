# -*- coding:utf-8 -*-
import cv2
import numpy as np
import math

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
cx = 0
cy = 0

while True:
    ret, img = cap.read()
    if not ret:
        break

    height, width, _ = img.shape
    center_x = width // 2
    center_y = height

    img_cvt = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_mask1 = cv2.inRange(img_cvt, np.array([0, 100, 100]), np.array([20, 255, 255]))
    img_mask2 = cv2.inRange(img_cvt, np.array([160, 100, 100]), np.array([180, 255, 255]))
    img_mask = img_mask1 + img_mask2

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
cap.release()