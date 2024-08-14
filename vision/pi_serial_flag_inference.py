import cv2
import numpy as np
import tensorflow as tf
import serial

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyAMA2', 115200, timeout=2)

# 모델 로드
model = tf.keras.models.load_model('mini_vgg_flag.h5')

# 라벨 맵 설정
labels_map = {0: 'Argentina_flag', 1: 'china_flag', 2: 'japan_flag', 3: 'korea_flag'}

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
    ser.write(chr(label).encode('ascii'))  # 라벨 인덱스를 아스키 값으로 전송
    print(f"{label}")
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

