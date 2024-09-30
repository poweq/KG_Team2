import cv2
import os
import time
import paramiko  # pip install paramiko

# SSH ¿¬°á Á¤º¸
hostname = "172.30.1.61"  # Windows PCÀÇ IP ÁÖ¼Ò
username = "sally"  # Windows »ç¿ëÀÚ ÀÌ¸§
password = "tlgus6368!"  # Windows »ç¿ëÀÚ ºñ¹Ð¹øÈ£
remote_path = "C:\\Users\\sally\\Desktop\\sidepj\\checkboardimg"  # Windows¿¡¼­ ÆÄÀÏÀ» ÀúÀåÇÒ °æ·Î

# USB Ä«¸Þ¶ó ÃÊ±âÈ­
cap = cv2.VideoCapture(0)

# ÀúÀåÇÒ ÀÌ¹ÌÁö ÆÄÀÏ °æ·Î
image_folder = './images'
if not os.path.exists(image_folder):
    os.makedirs(image_folder)

# SSH ¿¬°á ¼³Á¤
def upload_image(image_path):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, username=username, password=password)
        
        sftp = ssh.open_sftp()
        sftp.put(image_path, os.path.join(remote_path, os.path.basename(image_path)))
        sftp.close()
        ssh.close()
        print(f"Uploaded {image_path} to {remote_path}")
    except Exception as e:
        print(f"Failed to upload {image_path}: {e}")

# »çÁø Âï±â ¹× ¾÷·Îµå
for i in range(10):
    ret, frame = cap.read()
    if ret:
        image_path = os.path.join(image_folder, f'image_{i}.jpg')
        cv2.imwrite(image_path, frame)
        print(f"Captured {image_path}")

        # ÀÌ¹ÌÁö ¾÷·Îµå
        upload_image(image_path)

    time.sleep(5)  # 5ÃÊ ´ë±â

# ÀÚ¿ø ÇØÁ¦
cap.release()
cv2.destroyAllWindows()
