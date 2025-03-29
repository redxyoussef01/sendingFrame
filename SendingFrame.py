import cv2
import requests

ngrok_url = 'https://458f-41-251-195-136.ngrok-free.app'

# Open video capture from /dev/video0 (the camera device)
cap = cv2.VideoCapture('/dev/video0')

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the resolution of the captured frames (e.g., 640x480)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set height

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Encode the frame as JPEG
    _, encoded_frame = cv2.imencode('.jpg', frame)

    # Send the frame to your server
    response = requests.post(f'{ngrok_url}/video_frame', data=encoded_frame.tobytes())
    print(response.json())

cap.release()

# Send shutdown signal to the server
requests.post(f'{ngrok_url}/shutdown')
