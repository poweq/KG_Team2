import cv2
import numpy as np
import tensorflow as tf
import serial
import threading

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyAMA2', 115200, timeout=2)

# 모델 로드
model = tf.keras.models.load_model('mini_vgg_flag.h5')

# 라벨 맵 설정
labels_map = {0: 'Argentina_flag', 1: 'china_flag', 2: 'japan_flag', 3: 'korea_flag'}

# 스레드 락 설정
lock = threading.Lock()


# 시리얼 데이터 전송 스레드 함수
def serial_thread():
    global label_to_send
    ser.write(b'hello serial thread\r\n')
    while True:
        with lock:
            if label_to_send is not None:
                try:
                    # 인덱스 값을 아스키로 전송
                    if 0 <= label_to_send <= 3: 
                        ascii_char = chr(48 + label_to_send)  # 48은 '0'의 아스키 코드
                        ser.write(ascii_char.encode())
                        print(f"Sent: {ascii_char}")  # 디버깅용 출력
                    else:
                        print(f"Invalid label_to_send value: {label_to_send}")
                    label_to_send = None
                except Exception as e:
                    print(f"Error in serial communication: {e}")
# 초기화
label_to_send = None

# 시리얼 스레드 시작
thread = threading.Thread(target=serial_thread)
thread.daemon = True
thread.start()
print("Serial thread started.")

# 비디오 캡처 설정 (USB 카메라 사용)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # 이미지 전처리
    image = cv2.resize(frame, (64, 64))
    image = np.expand_dims(image, axis=0)
    image = image / 255.0

    # 추론
    preds = model.predict(image)
    label = np.argmax(preds)
    label_name = labels_map[label]
    confidence = preds[0][label] * 100

    # 결과를 시리얼로 전송 (인덱스 값을 아스키로 전송)
    with lock:
        label_to_send = label
        print(label)

    # 화면에 결과 표시
    cv2.putText(frame, f"{label_name}: {confidence:.2f}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Camera', frame)

    # 'q' 키를 눌러서 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 처리
cap.release()
cv2.destroyAllWindows()
ser.close()

