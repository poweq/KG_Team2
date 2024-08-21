import cv2
import numpy as np
import serial
import threading
import time


class uart_class(object):
    try:
        def __init__(self,com = "/dev/ttyAMA2"):
        #def __init__(self,com = "COM7"):
            self.ser_init=serial.Serial(com,115200,timeout=2)
            #포트 연결 확인
            if self.ser_init.isOpen():
                print('Serial port open!')
            else:
                print("can't open Serial port open...")
                #안정화를 위한 대기
                time.sleep(1)
    except Exception as e:
        print(f"Error initializing serial port: {e}")
    
    #py에서는 동작될일 x    
    def recive_data(self):
        #입렵 버퍼를 초기화
        self.ser_init.flushInput()

        #data를 받기 위해 무한루프
        while True:
            try:
            #문자 1개만 받아서 동작시키기
                data = self.ser_init.read(1)
                if data :
                    print(f"input data : {data}")
            
            except Exception as e:
                print(f"Error receiving data: {e}")
            
    def send_data(self, data):
        try : 
            self.ser_init.write(data.encode('ascii'))
        #하드웨어 에러 발생시
        except Exception as e :
            print(f"Error sending data: {e}")

    def recevie_thread(self):
        try:
           task_name = "Serial_Thread"
           rx_task = threading.Thread(target=self.recive_data,name=task_name)
           rx_task.setDaemon(True)
           rx_task.start()
           print("start RX Thread")
           time.sleep(.05)
        except Exception as e:
            print(f"Error starting serial thread: {e}")


def main():
    camera = cv2.VideoCapture(0)
    camera.set(3,160)
    camera.set(4,120)

    while(camera.isOpened()):
        ret, frame = camera.read()
        cv2.imshow('normal', frame)

        crop_img = frame[60:120, 0 : 160]

        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray,(5, 5), 0)

        ret,thresh1 = cv2.threshold(blur,130, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.erode(thresh1, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('imshow', mask)

        contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            if cx >= 95 and cx <= 125:
                print("Turn Left!")
            elif cx >= 39 and cx <= 65:
                print("Turn Right")
            else:
                print("go")

        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    com = "/dev/ttyAMA2"
    #com = 'COM7'
    bot = uart_class(com)
    time.sleep(1)
    #Thread 시작
    bot.recevie_thread()

    main()
        

