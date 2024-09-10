import cv2
import numpy as np
import threading
# 전역 변수로 프레임 저장
frame = None
stop_threads = False
def capture_frames():
    global frame, stop_threads
    camera = cv2.VideoCapture(0)
    camera.set(3, 160)  # width 설정
    camera.set(4, 120)  # height 설정
    while not stop_threads:
        ret, frame = camera.read()  # 카메라에서 프레임을 읽음
        if not ret:
            break
    camera.release()
def process_frames():
    global frame, stop_threads
    while not stop_threads:
        if frame is None:
            continue
        # 이미지 처리
        crop_img = frame[60:120, 0:160]
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        ret, thresh1 = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY_INV)
        # 마스크 처리
        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # 윤곽선 찾기
        contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        # 윤곽선이 있으면 처리
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # 좌우 방향 결정
                if cx >= 95 and cx <= 125:
                    print("Turn Left!")
                elif cx >= 39 and cx <= 65:
                    print("Turn Right")
                else:
                    print("go")
        # 화면 출력
        cv2.imshow('normal', frame)
        cv2.imshow('processed', mask)
        if cv2.waitKey(1) == ord('q'):
            stop_threads = True
            break
    cv2.destroyAllWindows()
if __name__ == '__main__':
    # 스레드 생성
    thread1 = threading.Thread(target=capture_frames)
    thread2 = threading.Thread(target=process_frames)
    # 스레드 시작
    thread1.start()
    thread2.start()
    # 메인 스레드는 스레드가 끝날 때까지 대기
    thread1.join()
    thread2.join()
    print("프로그램 종료")