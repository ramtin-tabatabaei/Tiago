import cv2
import numpy as np

# Initialize video capture
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

count = 19
try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break
        
        # Display the frame with markers
        cv2.imshow('Frame', frame)

        # Check for key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            cv2.imwrite(f"Photos2/frame{count}.jpg", frame)
            print(f"Saved frame{count}.jpg")
            count += 1
        # Inside the loop, after capturing a frame:
        if ret:
            print(f"Captured frame resolution: {frame.shape[1]}x{frame.shape[0]}")


finally:
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
