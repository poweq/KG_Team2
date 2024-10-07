from picamera import PiCamera
from time import sleep

# 카메라 객체 생성
camera = PiCamera()

# 카메라 해상도 설정 (옵션)
camera.resolution = (1920, 1080)

# 카메라 프리뷰 시작
camera.start_preview()

# 2초 대기 (카메라가 적응할 시간을 줌)
sleep(2)

# 사진 촬영 및 저장
camera.capture('/home/pi/Desktop/image.jpg')

# 카메라 프리뷰 종료
camera.stop_preview()

# 카메라 해제
camera.close()
