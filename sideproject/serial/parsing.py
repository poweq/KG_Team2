import serial
import threading
import time

class JDamr(object):
    def __init__(self, com="/dev/ttyAMA2"):
        self.ser = serial.Serial(com, 115200)
        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")
        time.sleep(1)

    def receive_data(self):
        self.ser.flushInput()

        while True:
            head = bytearray(self.ser.read())[0]
            if head == 0xf5:
                payload = []
                length = bytearray(self.ser.read())[0]
                for i in range(length):
                    value = bytearray(self.ser.read())[0]
                    payload.append(value)
                self.parse_cmd(payload)

    def receive_thread(self):
        try:
            task_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=task_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread")
            time.sleep(.05)
        except Exception as e:
            print(f"Error starting serial thread: {e}")

    def parse_cmd(self, payload):
        print(payload)
        encode1 = int.from_bytes(payload, byteorder="big")
        # print(encode1)  # Uncomment if you want to see the decoded integer value

    def send_data(self, data):
        try:
            self.ser.write(data.encode('ascii'))  # Encode ASCII data to bytes before sending
        except Exception as e:
            print(f"Error sending data: {e}")

if __name__ == '__main__':
    com = '/dev/ttyAMA2'
    bot = JDamr(com)
    time.sleep(1)
    bot.receive_thread()

    while True:
        try:
            user_input = input("Enter ASCII data to send: ")
            bot.send_data(user_input)
        except Exception as e:
            print(f"Error processing input: {e}")
