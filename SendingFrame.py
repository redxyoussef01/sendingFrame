import cv2
import requests

cap = cv2.VideoCapture("drive.mp4")
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    _, encoded_frame = cv2.imencode('.jpg', frame)
    response = requests.post('http://127.0.0.1:4002/video_frame', data=encoded_frame.tobytes())
    print(response.json())
cap.release()
requests.post('http://127.0.0.1:4002/shutdown')