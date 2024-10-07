import asyncio
import websockets
import cv2

async def send_camera_data():
    # 카메라 장치 열기
    cap = cv2.VideoCapture(0)

    # PC로 연결 (IP 주소와 포트를 지정)
    async with websockets.connect('ws://192.168.137.1:8765/camera_frame') as websocket:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("카메라를 읽을 수 없습니다.")
                break

            # 프레임을 JPEG로 인코딩 후 바이트로 변환하여 전송
            _, buffer = cv2.imencode('.jpg', frame)
            await websocket.send(buffer.tobytes())

            # 'q' 키를 누르면 전송 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()

# 비동기 웹 소켓 클라이언트 시작
asyncio.run(send_camera_data())

