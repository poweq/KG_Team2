import subprocess
import datetime

def take_photo(filename):
    try:
        # 현재 날짜와 시간을 기준으로 파일 이름을 생성
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_path = f"{filename}_{timestamp}.jpg"
        
        # libcamera-still 명령어를 사용하여 사진 촬영
        # -o 옵션은 출력 파일 이름을 지정, -t 옵션은 촬영 대기 시간을 설정 (단위: 밀리초, 0은 즉시 촬영)
        command = ["libcamera-still", "-o", file_path, "-t", "0"]
        
        # 명령어 실행
        subprocess.run(command, check=True)
        
        print(f"사진이 성공적으로 저장되었습니다: {file_path}")
    except Exception as e:
        print(f"사진 촬영 중 오류가 발생했습니다: {e}")

if __name__ == "__main__":
    # 저장할 파일의 기본 이름을 지정
    take_photo("captured_image")
