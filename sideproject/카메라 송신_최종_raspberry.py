import asyncio
import websockets
import cv2

async def camera_data():
    # 카메라 장치 열기
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    try:
        # 메인 서버에 연결
        async with websockets.connect('ws://172.30.1.61:8765') as websocket:
            print("메인 서버에 연결되었습니다.")
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("카메라를 읽을 수 없습니다.")
                    break

                # 프레임을 바이트로 변환하여 메인 서버로 송신
                success, buffer = cv2.imencode('.jpg', frame)
                if not success:
                    print("프레임 인코딩에 실패했습니다.")
                    continue

                await websocket.send(buffer.tobytes())

                # 메인 서버로부터 차선 각도 값 수신
                try:
                    angle = await asyncio.wait_for(websocket.recv(), timeout=5)
                    print(f"수신된 차선 각도: {angle}")
                except asyncio.TimeoutError:
                    print("서버로부터 응답을 받지 못했습니다.")
                
                # 프레임 전송 속도 조절 (예: 30 FPS)
                await asyncio.sleep(0.033)
    except websockets.exceptions.ConnectionClosed as e:
        print(f"웹소켓 연결이 종료되었습니다: {e}")
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        # 자원 해제
        cap.release()
        print("카메라 자원이 해제되었습니다.")

def main():
    try:
        asyncio.run(camera_data())
    except KeyboardInterrupt:
        print("사용자에 의해 종료되었습니다.")

if __name__ == "__main__":
    main()
