import cv2

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)

# Set a lower resolution, e.g., 320x240
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width to 320
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height to 240

# Check if the camera is opened
if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
    else:
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

cap.release()
