import cv2
import os
import numpy as np
import asyncio
import websockets
from line_detect.jd_opencv_lane_detect import JdOpencvLaneDetect

cv_detector = JdOpencvLaneDetect()

async def process_camera_data(websocket):
    async with websockets.connect('ws://172.30.1.59:9065') as angle_websocket:  # 클라이언트 서버에 다시 연결
        # 데이터 폴더가 없으면 생성
        if not os.path.exists('./data'):
            os.makedirs('./data')

        # 비디오 저장 객체 생성
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        video_orig = cv2.VideoWriter('./data/car_video.avi', fourcc, 20.0, (320, 240))

        while True:
            # 클라이언트 서버로부터 이미지 데이터 수신
            data = await websocket.recv()
            np_arr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                print("이미지를 디코딩할 수 없습니다.")
                continue

            # 차선 인식
            lanes, img_lane = cv_detector.get_lane(frame)
            angle, img_angle = cv_detector.get_steering_angle(img_lane, lanes)
            
            if img_angle is None:
                print("차선을 찾지 못했습니다.")
            else:
                print(f"차선 각도: {angle}")
                cv2.imshow('lane', img_angle)  # 차선 각도 표시

                # 클라이언트 서버로 각도 값 전송
                await angle_websocket.send(str(angle))  # 각도를 문자열로 변환해 송신

            # 비디오 파일에 프레임 저장
            video_orig.write(frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 자원 해제
        video_orig.release()
        cv2.destroyAllWindows()

# 웹소켓 서버 설정
async def receive_data(websocket, path):
    await process_camera_data(websocket)

start_server = websockets.serve(receive_data, '0.0.0.0', 8765)  # 적절한 포트 번호 사용

# 이벤트 루프 실행
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()