import cv2
import requests

URL = 'https://1d64-196-115-119-83.ngrok-free.app/video_frame'

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    _, encoded_frame = cv2.imencode('.jpg', frame)

    try:
        response = requests.post(URL, data=encoded_frame.tobytes(), verify=False, timeout=5)
        print(response.status_code, response.text)
    except requests.exceptions.SSLError as e:
        print("SSL Error:", e)
    except requests.exceptions.RequestException as e:
        print("Request Error:", e)

cap.release()
requests.post(' https://1d64-196-115-119-83.ngrok-free.app/shutdown', verify=False)
