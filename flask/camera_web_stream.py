from flask import Flask, Response
import cv2

app = Flask(__name__)

# 비디오 캡처 객체 생성 (0은 기본 USB 카메라를 의미)
video_capture = cv2.VideoCapture(0)
if not video_capture.isOpened():
    print("Error: Could not open video capture.")

def generate_frames():
    while True:
        # 프레임 캡처
        success, frame = video_capture.read()
        if not success:
            break
        else:
            # JPEG 인코딩
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

        # MJPEG 스트리밍
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return '''
        <html>
        <body>
            <h1>USB Camera Stream</h1>
            <img src="/video_feed">
        </body>
        </html>
    '''

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)

