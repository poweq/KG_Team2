# 리모트 PC의 코드
import cv2
import asyncio
import websockets
import numpy as np
import base64

async def send_video(uri):
    async with websockets.connect(uri) as websocket:
        cap = cv2.VideoCapture(0)  # 웹 캠 열기

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 프레임을 JPEG로 인코딩
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            # 웹 소켓으로 데이터 전송
            await websocket.send(jpg_as_text)

        cap.release()

# 웹 소켓 서버 URI
uri = "ws://192.168.100.1:8765"
asyncio.run(send_video(uri))

