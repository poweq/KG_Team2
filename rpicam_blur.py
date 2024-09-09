import cv2
import os
import time

def main():
    camera = cv2.VideoCapture(0)  # 0은 기본 웹캠을 의미합니다.
    camera.set(3, 320)  # 해상도 설정 (320x240)
    camera.set(4, 240)

    filepath = "/home/pi/shim/picture"
    print(f"Saving to: {os.path.abspath(filepath)}")

    if not os.path.exists(filepath):
        os.makedirs(filepath)
        print(f"Directory created: {filepath}")

    i = 0

    while camera.isOpened():
        ret, image = camera.read()
        if not ret:
            print("Failed to grab frame")
            break

        # 이미지를 상하 반전
        image = cv2.flip(image, -1)
        cv2.imshow('normal', image)

        # 이미지 크기 확인 및 하단 절반 자르기
        height, width, _ = image.shape
        crop_img = image[height // 2 :, :]

        # 자른 이미지를 그레이스케일로 변환
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        # 그레이스케일 이미지에 가우시안 블러 적용
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        ret, thresh1 = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY_INV)

        # 마스크 생성 및 모폴로지 연산
        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('mask', mask)

        # 블러 처리된 이미지를 저장 (JPEG 형식으로 압축 품질 90으로 설정)
        filename = os.path.join(filepath, f"{i:05d}.jpg")
        print(f"Saving image: {filename}")

        success = cv2.imwrite(filename, blur, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if success:
            print(f"Image saved successfully: {filename}")
        else:
            print(f"Failed to save image: {filename}")

        i += 1
        time.sleep(1.0)  # 저장 간격 설정

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) == ord('q'):
            break

    # 모든 창 닫기
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
