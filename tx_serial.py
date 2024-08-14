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


if __name__ == "__main__":
    com = "/dev/ttyAMA2"
    #com = 'COM7'
    bot = uart_class(com)
    time.sleep(1)
    #Thread 시작
    bot.recevie_thread()

    while True:
        try:
            user_input = input("Enter ASCII data to send: ")
            bot.send_data(user_input)
        except Exception as e:
            print(f"Error processing input: {e}")
