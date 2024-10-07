import bluetooth

def start_server():
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

    try:
        # 블루투스 모듈에 바인딩
        server_sock.bind(("", bluetooth.PORT_ANY))
        server_sock.listen(1)

        port = server_sock.getsockname()[1]
        uuid = "00001101-0000-1000-8000-00805F9B34FB"  # SPP UUID
        bluetooth.advertise_service(server_sock, "RaspberryPiServer", service_id=uuid,
                                    service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                                    profiles=[bluetooth.SERIAL_PORT_PROFILE])
        print("Waiting for connection on RFCOMM channel", port)

        client_sock, client_info = server_sock.accept()
        print("Accepted connection from", client_info)

        try:
            while True:
                data = client_sock.recv(1024)
                if not data:
                    break
                print("Received:", data.decode("utf-8"))
                client_sock.send(data)  # Echo the data back
        except OSError as e:
            print("Error during data transmission:", e)

    except bluetooth.BluetoothError as e:
        print("Bluetooth error:", e)

    finally:
        try:
            if client_sock:
                client_sock.close()
        except Exception as e:
            print("Error closing client socket:", e)
        try:
            server_sock.close()
        except Exception as e:
            print("Error closing server socket:", e)
        print("All done.")

if __name__ == "__main__":
    start_server()

