import cv2
import subprocess
import datetime

def take_photo(filename):
    try:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        file_path = f"{filename}_{timestamp}.jpg"
        command = ["libcamera-still", "-o", file_path, "-t", "1"]
        subprocess.run(command, check=True)
        print(f"Photo saved as: {file_path}")
    except Exception as e:
        print(f"Error: {e}")

def process_camera_and_take_photo():
    camera = cv2.VideoCapture(0)
    
    if not camera.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 's' to take a photo, 'q' to quit")

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        cv2.imshow('Camera Feed', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            take_photo("captured_image")
        elif key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    process_camera_and_take_photo()
