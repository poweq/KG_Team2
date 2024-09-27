import socket
import serial

HOST = '0.0.0.0'  
PORT = 65432   

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()


print(f"{HOST}:{PORT} wating...")

conn, addr = server_socket.accept()
print(f"{addr}connect")

arduino = serial.Serial('/dev/ttyAMA1', 9600)

while True:
    data = conn.recv(4)  
    if not data:
        break
    received_int = int.from_bytes(data, byteorder='big')
    print(f"message: {received_int}")

    as2_code = f"{received_int}\n"  
    arduino.write(as2_code.encode())

conn.close()
server_socket.close()
