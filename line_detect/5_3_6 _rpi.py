import cv2
import numpy as np
import picamera
import time
from picamera.array import PiRGBArray

def record_video():
    camera = picamera.PiCamera()
    camera.resolution = (1920, 1080)
    camera.framerate = 30

    camera.start_recording('video.h264')
    time.sleep(10)
    camera.stop_recording()

def process_video_stream():
    camera = picamera.PiCamera()
    camera.resolution = (160, 120)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(160, 120))

    time.sleep(0.1)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        image = cv2.flip(image, -1)

        crop_img = image[60:120, 0:160]

        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        ret, thresh1 = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('Mask', mask)

        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                if 95 <= cx <= 125:
                    print("Turn Left!")
                elif 39 <= cx <= 65:
                    print("Turn Right!")
                else:
                    print("Go")

        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    record_video()
    process_video_stream()
