import cv2

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
  # Try with default device

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
