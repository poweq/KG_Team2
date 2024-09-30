import cv2
import asyncio
import websockets
import base64

async def send_video(websocket):
    cap = cv2.VideoCapture(0)  # 웹캠 열기

    while cap.isOpened():  # 카메라가 열려 있는 동안
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            break

        # 프레임을 JPEG로 인코딩
        _, buffer = cv2.imencode('.jpg', frame)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')

        # 웹 소켓으로 데이터 전송
        await websocket.send(jpg_as_text)

        # 전송 확인 로그
#        print("Frame sent")

        # 좌표 수신을 위해 잠시 대기
        await asyncio.sleep(0.1)

    cap.release()

async def receive_coordinates(websocket):
    while True:
        try:
            # 호스트로부터 x, y 좌표 수신
            coords = await websocket.recv()  # 호스트에서 데이터를 기다림
            print(f"Received coordinates from host: {coords}")
        except Exception as e:
            print(f"Error receiving coordinates: {e}")

async def main(uri):
    async with websockets.connect(uri) as websocket:
        # 프레임 전송과 좌표 수신을 동시에 처리
        await asyncio.gather(
            send_video(websocket),       # 비디오 전송
            receive_coordinates(websocket)  # 좌표 수신
        )

# 웹 소켓 서버 URI
uri = "ws://192.168.100.1:8765"
asyncio.run(main(uri))

