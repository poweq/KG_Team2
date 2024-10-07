import cv2

def test_camera():
    camera = cv2.VideoCapture(0)

    if not camera.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 'q' to quit")

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        cv2.imshow('Camera Test', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera()
