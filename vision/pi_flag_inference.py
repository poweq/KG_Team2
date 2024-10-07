import cv2
import numpy as np
import tensorflow as tf



# TensorFlow 모델 로드
model = tf.keras.models.load_model('mini_vgg_flag.h5')

# 클래스 레이블 정의
class_labels = ["Argentina_flag", "china_flag", "japan_flag", "korea_flag"]

# USB 카메라 열기
camera = cv2.VideoCapture(0)

# 카메라가 열렸는지 확인
if not camera.isOpened():
    print("Error: Camera not accessible.")
    exit()

def preprocess_image(image):
    image = cv2.resize(image, (64, 64))  # 모델 입력 크기에 맞게 조정
    image = image.astype("float32") / 255.0
    return np.expand_dims(image, axis=0)

while True:
    # 카메라로부터 프레임 캡처
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame")
        break

    # 이미지 전처리
    preprocessed_image = preprocess_image(frame)

    # 추론
    predictions = model.predict(preprocessed_image)
    class_index = np.argmax(predictions[0])
    confidence = predictions[0][class_index]



    # 결과 출력
    print(f"Detected: {class_labels[class_index]} with confidence {confidence:.2f}")

    # 결과를 화면에 표시
    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라와 창 해제
camera.release()
cv2.destroyAllWindows()

