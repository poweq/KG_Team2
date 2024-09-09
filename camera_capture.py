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
    print("Press 's' to take a photo, 'q' to quit")

    while True:
        key = input("Press a key: ").lower()
        
        if key == 's':
            take_photo("captured_image")
        elif key == 'q':
            break

if __name__ == "__main__":
    process_camera_and_take_photo()
