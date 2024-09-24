import cv2
import datetime

def capture_photo_on_key():
    # V4L2 백엔드를 사용하여 카메라 초기화
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not camera.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    print("카메라가 열렸습니다. 's' 키를 눌러 사진을 저장하고, 'q' 키를 눌러 종료합니다.")

    while True:
        # 카메라에서 프레임 읽기
        ret, frame = camera.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # 프레임을 화면에 표시
        cv2.imshow("Camera", frame)

        # 키 입력을 대기
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):  # 's' 키를 눌렀을 때
            # 현재 날짜와 시간에 기반한 파일 이름 생성
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            file_path = f"captured_image_{timestamp}.jpg"

            # 현재 프레임을 파일로 저장
            cv2.imwrite(file_path, frame)
            print(f"사진이 저장되었습니다: {file_path}")

        elif key == ord('q'):  # 'q' 키를 눌렀을 때
            print("프로그램을 종료합니다.")
            break

    # 카메라 해제 및 모든 창 닫기
    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_photo_on_key()
