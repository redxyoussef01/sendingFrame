import cv2
import requests

# Replace with your ngrok URL
ngrok_url = 'https://76a7-41-143-49-120.ngrok-free.app'

cap = cv2.VideoCapture('/dev/video14')
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    _, encoded_frame = cv2.imencode('.jpg', frame)
    response = requests.post(f'{ngrok_url}/video_frame', data=encoded_frame.tobytes())
    print(response.json())
cap.release()

# Sending shutdown signal
requests.post(f'{ngrok_url}/shutdown')
