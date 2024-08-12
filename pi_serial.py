import serial
import time

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyS0', 9600)  # /dev/ttyS0  GPIO 14 (TXD)와 GPIO 15 (RXD) 
time.sleep(2)  # 잠시 대기하여 연결을 안정화합니다.

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').rstrip()
        print("Received:", data)
        ser.write(b'Hello from Raspberry Pi\n')
