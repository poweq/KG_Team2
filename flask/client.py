# client.py (Raspberry Pi 측)
import socketio

# 서버의 주소와 포트를 입력하세요.
sio = socketio.Client()

@sio.event
def connect():
    print("Connected to server")
    sio.emit('message', {'data': 'Hello from Raspberry Pi!'})

@sio.event
def disconnect():
    print("Disconnected from server")

@sio.on('response')
def handle_response(data):
    print(f"Received response from server: {data['message']}")

def main():
    try:
        # 서버에 연결 (PC의 IP 주소와 포트)
        sio.connect('http://192.168.100.1:8050')
        sio.wait()
    except Exception as e:
        print(f"Connection failed: {e}")

if __name__ == '__main__':
    main()

