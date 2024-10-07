import asyncio
import websockets
import cv2

async def send_camera_data():
    # 카메라 장치 열기
    cap = cv2.VideoCapture(0)

    # PC로 연결
    async with websockets.connect('ws://192.168.137.1:8765') as websocket:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("카메라를 읽을 수 없습니다.")
                break

            # 프레임을 JPEG로 인코딩 후 바이트로 변환
            _, buffer = cv2.imencode('.jpg', frame)
            # 웹소켓을 통해 프레임 데이터 전송 (이벤트 이름 명시)
            await websocket.send(buffer.tobytes())

            # 'q' 키를 누르면 전송 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()

# 웹소켓 클라이언트 시작
asyncio.run(send_camera_data())

