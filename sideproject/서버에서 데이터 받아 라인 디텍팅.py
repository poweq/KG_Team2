import asyncio
import websockets
import cv2
import numpy as np
import os
from line_detect.jd_opencv_lane_detect import JdOpencvLaneDetect

# OpenCV line detector object
cv_detector = JdOpencvLaneDetect()

# Find ./data folder for labeling data
try:
    if not os.path.exists('./data'):
        os.makedirs('./data')
except OSError:
    print("failed to make ./data folder")

# Create video codec object. We use 'XVID' format for Raspberry pi.
fourcc = cv2.VideoWriter_fourcc(*'XVID')
# Video write object
video_orig = cv2.VideoWriter('./data/car_video.avi', fourcc, 20.0, (320, 240))

async def receive_and_detect():
    async with websockets.connect('ws://172.30.1.59:8765') as websocket:
        while True:
            data = await websocket.recv()
            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                # camera image writing
                video_orig.write(frame)

                # Find lane angle
                lanes, img_lane = cv_detector.get_lane(frame)
                angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
                
                if img_angle is None:
                    print("can't find lane...")
                    pass
                else:
                    cv2.imshow('lane', img_angle)
                    print("각도:", angle)
                    # 추가적으로 서보나 모터 제어 코드를 여기에 추가

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Error in receiving frame from server.")

    # 자원 해제
    video_orig.release()
    cv2.destroyAllWindows()

asyncio.get_event_loop().run_until_complete(receive_and_detect())
