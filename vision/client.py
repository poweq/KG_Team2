import cv2
import socket
import struct
import pickle
import time

# UDP ¼³Á¤
host_ip = "172.30.1.61"  # ¼­¹ö IP ÁÖ¼Ò
video_port = 5005  # ºñµð¿À Æ÷Æ®
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ºñµð¿À Ä¸Ã³
cap = cv2.VideoCapture(0)
fps = 0
frame_count = 0
start_time = time.time()

# ºñµð¿À ÇØ»óµµ ¼³Á¤
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # ÇÁ·¹ÀÓ ¸®»çÀÌÁî
    frame = cv2.resize(frame, (320, 240))

    # ¼­¹ö¿¡ ¿äÃ» Àü¼Û
    sock.sendto(b"request", (host_ip, video_port))  # ¿äÃ» ¸Þ½ÃÁö Àü¼Û

    # ¼­¹ö¿¡¼­ ·£´ý ¼ýÀÚ ¼ö½Å
    try:
        angle_data, _ = sock.recvfrom(1024)  # ÃÖ´ë 1024¹ÙÀÌÆ® ¼ö½Å
        print(f"¼ö½ÅµÈ ·£´ý ¼ýÀÚ: {angle_data.decode('utf-8')}")
    except socket.timeout:
        print("·£´ý ¼ýÀÚ ¼ö½Å ½Ã°£ ÃÊ°ú")

    # ÇÁ·¹ÀÓ JPEG ÀÎÄÚµù
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    data = pickle.dumps(buffer)

    # ºñµð¿À µ¥ÀÌÅÍ Àü¼Û
    sock.sendto(struct.pack("Q", len(data)) + data, (host_ip, video_port))

    # FPS °è»ê
    frame_count += 1
    if time.time() - start_time >= 1:
        fps = frame_count
        frame_count = 0
        start_time = time.time()

    # FPS È­¸é¿¡ Ç¥½Ã
    cv2.putText(frame, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Camera Sender", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
sock.close()