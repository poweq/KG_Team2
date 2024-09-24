#승환씨 코드(카메라 송신)
import asyncio
import websockets
import cv2

async def camera_data(websocket, path):
    cap = cv2.VideoCapture(0)  # 카메라 장치 열기
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # 프레임을 바이트로 변환
        _, buffer = cv2.imencode('.jpg', frame)
        await websocket.send(buffer.tobytes())

start_server = websockets.serve(camera_data, '0.0.0.0', 8765)  # 포트 번호는 적절히 설정

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()