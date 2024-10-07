from flask import Flask, render_template, request
import serial

app = Flask(__name__)

# UART 설정
ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

@app.route('/', methods=['GET', 'POST'])
def index():
    received_data = None
    if request.method == 'POST':
        color = request.form['color']
        if color in ['l', 'g', 'r']:
            ser.write(color.encode())  # UART로 문자 전송

            # 데이터를 송신한 후 루프백 테스트로 수신 확인
            #test code
            #received_data = ser.read(1).decode()  # 1바이트 읽기

    return render_template('led_control_index.html', received_data=received_data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

