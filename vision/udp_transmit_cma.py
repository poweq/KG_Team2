import cv2
import socket
import struct
import pickle
import time

# UDP ¼³Á¤
host_ip = "172.30.1.61"  # ¼­¹ö IP ÁÖ¼Ò
port = 5005  # ¼­¹ö Æ÷Æ®
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Ä«¸Þ¶ó ¼³Á¤
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # ÇÁ·¹ÀÓÀ» JPEG Çü½ÄÀ¸·Î ÀÎÄÚµù
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    
    # µ¥ÀÌÅÍ¸¦ Àü¼ÛÇÒ ¼ö ÀÖµµ·Ï Á÷·ÄÈ­
    data = pickle.dumps(buffer)
    
    # µ¥ÀÌÅÍ Å©±â¸¦ Àü¼Û
    sock.sendto(struct.pack("Q", len(data)) + data, (host_ip, port))

    # ¼­¹ö·ÎºÎÅÍ °¢µµ ¼ö½Å
    try:
        angle_data, _ = sock.recvfrom(1024)  # ÃÖ´ë 1024¹ÙÀÌÆ® ¼ö½Å
        print(f"¼ö½ÅµÈ °¢µµ: {angle_data.decode('utf-8')}¡Æ")
    except socket.timeout:
        print("°¢µµ ¼ö½Å ½Ã°£ ÃÊ°ú")

    time.sleep(0.1)  # 0.1ÃÊ °£°ÝÀ¸·Î Àü¼Û

cap.release()
sock.close()