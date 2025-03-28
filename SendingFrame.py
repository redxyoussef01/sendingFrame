import cv2
import requests

# Replace with your ngrok URL
ngrok_url = 'https://f9ce-105-71-18-245.ngrok-free.app'

cap = cv2.VideoCapture(10)
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
