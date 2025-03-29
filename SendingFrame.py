
import cv2
import requests

ngrok_url = ' https://458f-41-251-195-136.ngrok-free.app'
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    _, encoded_frame = cv2.imencode('.jpg', frame)
    response = requests.post(f'{ngrok_url}/video_frame', data=encoded_frame.tobytes())
    print(response.json())
cap.release()
requests.post(f'{ngrok_url}/shutdown')