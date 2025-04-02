import cv2
import requests

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    _, encoded_frame = cv2.imencode('.jpg', frame)
    response = requests.post('https://963e-105-157-85-231.ngrok-free.app/video_frame', data=encoded_frame.tobytes())
    print(response.json())
cap.release()
requests.post('https://963e-105-157-85-231.ngrok-free.app/shutdown')