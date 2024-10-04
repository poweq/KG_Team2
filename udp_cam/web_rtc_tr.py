import cv2
import asyncio
from aiohttp import web
from aiortc import VideoStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaStreamTrack, MediaPlayer, MediaRecorder
from av import VideoFrame
import aiohttp_cors

# 비디오 트랙 클래스 정의 (카메라 데이터를 WebRTC로 송신)
class CameraVideoStreamTrack(VideoStreamTrack):
    def __init__(self, camera_index=0):
        super().__init__()  # VideoStreamTrack 초기화
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # 해상도 설정 (너비)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 해상도 설정 (높이)
        self.cap.set(cv2.CAP_PROP_FPS, 15)  # 프레임 속도 설정 (15 FPS)

    async def recv(self):
        # 프레임을 캡처하고 WebRTC 비디오 프레임으로 변환
        ret, frame = self.cap.read()
        if not ret:
            return

        # OpenCV BGR -> RGB로 변환
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # VideoFrame 생성 및 반환
        video_frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        video_frame.pts = self.cap.get(cv2.CAP_PROP_POS_MSEC) * 90
        video_frame.time_base = VideoFrame.TIME_BASE
        return video_frame

# WebRTC Offer 요청을 처리하는 함수 정의
async def offer(request):
    # WebRTC 피어 연결 설정
    params = await request.json()
    pc = RTCPeerConnection()  # WebRTC 피어 연결 객체 생성
    pcs.add(pc)

    # 카메라 트랙 추가
    video_track = CameraVideoStreamTrack()
    pc.addTrack(video_track)

    # WebRTC offer 생성 및 응답
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response(
        {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
    )

# WebRTC 서버 초기화
pcs = set()  # 활성화된 WebRTC 피어 연결 집합
app = web.Application()  # aiohttp 웹 애플리케이션 생성

# /offer 경로에 대한 POST 요청 라우트 추가 및 라우트 객체 반환
route = app.router.add_post("/offer", offer)

# CORS 설정 추가
cors = aiohttp_cors.setup(app)

# 반환된 라우트 객체에 CORS 설정 적용
cors.add(route, {
    "*": aiohttp_cors.ResourceOptions(
        allow_credentials=True,
        expose_headers="*",
        allow_headers="*",
    )
})

# Signaling Server 실행
if __name__ == "__main__":
    web.run_app(app, host='0.0.0.0', port=8080)
