import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from skimage import exposure
import warnings
warnings.filterwarnings('ignore')

# 비디오 로드
cap = cv2.VideoCapture('data\._FILE240529-151746.AVI')
ret, frame = cap.read()

if not ret:
    print("비디오를 로드하는 데 실패했습니다.")
else:
    # 프레임 크기 조정
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    
    # 프레임 크기 가져오기
    height, width = frame.shape[:2]
    print("프레임 크기:", frame.shape[:2])
    
    # 관심 영역 추출 및 표시
    temp = frame[220:height-12, :width, 2]
    
    # 이미지 표시
    plt.imshow(temp, cmap='gray')
    plt.title("빨간색 채널의 관심 영역")
    plt.axis('off')
    plt.show()

# 비디오 캡처 객체 해제
cap.release()
