import asyncio
import websockets
import cv2

async def camera_data():
    # 카메라 장치 열기
    cap = cv2.VideoCapture(0)
    
    # 메인 서버에 연결
    async with websockets.connect('ws://172.30.1.61:8765') as websocket:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("카메라를 읽을 수 없습니다.")
                break

            # 프레임을 바이트로 변환하여 메인 서버로 송신
            _, buffer = cv2.imencode('.jpg', frame)
            await websocket.send(buffer.tobytes())

            # 메인 서버로부터 차선 각도 값 수신
            angle = await websocket.recv()
            print(f"수신된 차선 각도: {angle}")

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # 자원 해제
    cap.release()

# 웹소켓 클라이언트 시작
asyncio.get_event_loop().run_until_complete(camera_data())
asyncio.get_event_loop().run_forever()
