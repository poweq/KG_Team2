import cv2
import numpy as np
import serial
import time

def main():
    # ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)

    camera = cv2.VideoCapture(0)
    camera.set(3,160)
    camera.set(4,120)

    while(camera.isOpened()):
        ret, frame = camera.read()
        cv2.imshow('normal', frame)

        crop_img = frame[60:120, 0 : 160]

        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray,(5, 5), 0)

        ret,thresh1 = cv2.threshold(blur,130, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('imshow', mask)

        contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            #값이 40이상일때 최대 출력
            if cx >= 95 and cx <= 125:
                fa1=cx-85
                print('L %d' %fa1) #go left
            elif cx >= 39 and cx <= 65:
                fa2=85-cx
                print('R %d' %fa2) #go right
            else:
                print("G")

        if cv2.waitKey(1) == ord('q'):
            break

    ser.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
