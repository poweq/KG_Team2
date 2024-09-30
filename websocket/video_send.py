# remote_camera_sender.py
import cv2
import asyncio
import websockets
import base64

async def send_video(uri):
    async with websockets.connect(uri) as websocket:
        cap = cv2.VideoCapture(0)  # 웹 캠 열기

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture image")
                break

            # 프레임을 JPEG로 인코딩
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')

            # 웹 소켓으로 데이터 전송
            await websocket.send(jpg_as_text)
            print("Frame sent")

            # 호스트에서 angle 값 수신
            angle_data = await websocket.recv()
            print(f"Received from host: {angle_data}")

        cap.release()

# 웹 소켓 서버 URI
uri = "ws://192.168.100.1:8765"
asyncio.run(send_video(uri))

