# 영상 받을 PC의 웹 소켓 서버 및 영상 처리 코드
import asyncio
import websockets
import numpy as np
import cv2
import base64

async def handle_client(websocket, path):
    while True:
        try:
            # 웹 소켓으로부터 데이터 수신
            jpg_as_text = await websocket.recv()
            jpg_original = base64.b64decode(jpg_as_text)

            # JPEG 데이터를 NumPy 배열로 변환
            np_arr = np.frombuffer(jpg_original, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 영상 표시
            cv2.imshow('Video', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except websockets.ConnectionClosed:
            break

    cv2.destroyAllWindows()

async def main():
    async with websockets.serve(handle_client, "0.0.0.0", 8765):
        await asyncio.Future()  # run forever

asyncio.run(main())
