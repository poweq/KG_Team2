import asyncio
import websockets
import cv2
import numpy as np

async def receive_data():
    async with websockets.connect('ws://172.30.1.59:8765') as websocket:
        while True:
            data = await websocket.recv()
            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # HSV로 변환
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # HSV 필터 적용 - 여기서 특정 범위로 필터를 설정 (예: 빨간색)
            lower_bound = np.array([0, 100, 100])
            upper_bound = np.array([10, 255, 255])
            hsv_filtered = cv2.inRange(hsv_frame, lower_bound, upper_bound)

            # 원본 프레임과 HSV 필터링된 화면을 동시에 표시
            cv2.imshow('Received Frame', frame)
            cv2.imshow('HSV Filtered Frame', hsv_filtered)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

asyncio.get_event_loop().run_until_complete(receive_data())
