import cv2
import requests

# Replace with your ngrok URL
ngrok_url = 'https://76a7-41-143-49-120.ngrok-free.app'

cap = cv2.VideoCapture(10)
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
