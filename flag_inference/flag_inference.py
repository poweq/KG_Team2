import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array

# 모델과 라벨 로드
model = load_model('mini_vgg_flag.h5')
labels = ["Argentina_flag", "china_flag", "japan_flag", "korea_flag"]

def preprocess_frame(frame):
    # 이미지 전처`리
    frame = cv2.resize(frame, (64, 64))  # 모델 입력 크기
    frame = frame.astype("float") / 255.0  # 정규화
    frame = img_to_array(frame)
    frame = np.expand_dims(frame, axis=0)
    return frame

# 카메라 실행
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()  # 카메라에서 프레임 읽기
    if not ret:
        break

    # 프레임 전처리 및 예측
    preprocessed_frame = preprocess_frame(frame)
    preds = model.predict(preprocessed_frame)[0]
    pred_idx = np.argmax(preds)
    label = labels[pred_idx]
    confidence = preds[pred_idx] * 100

    # 예측 결과를 프레임에 표시
    text = f"{label}: {confidence:.2f}%"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # 프레임 출력
    cv2.imshow("Flag Detection", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료
cap.release()
cv2.destroyAllWindows()
