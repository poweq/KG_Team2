from flask import Flask, render_template_string, request
import serial

app = Flask(__name__)

# UART 설정
ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

# HTML 코드
html_template = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>UART Control</title>
</head>
<body>
    <h1>UART Control Panel</h1>
    <form method="post">
        <button name="color" value="r">Red</button>
        <button name="color" value="g">Green</button>
        <button name="color" value="b">Blue</button>
    </form>

    {% if received_data %}
    <p>Received data: {{ received_data }}</p>
    {% endif %}
</body>
</html>
'''

@app.route('/', methods=['GET', 'POST'])
def index():
    received_data = None
    if request.method == 'POST':
        color = request.form['color']
        if color in ['r', 'g', 'b']:
            ser.write(color.encode())  # UART로 문자 전송

            # 데이터를 송신한 후 루프백 테스트로 수신 확인
            received_data = ser.read(1).decode()  # 1바이트 읽기

    return render_template_string(html_template, received_data=received_data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

