import cv2
import requests

# Replace with your ngrok URL
ngrok_url = 'https://f9ce-105-71-18-245.ngrok-free.app'

for source in range(0, 41):
    print(f"Testing source {source}...")
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    
    if not cap.isOpened():
        print(f"Source {source} is not available.")
        continue  # Skip to the next source

    ret, frame = cap.read()
    if not ret:
        print(f"Source {source} could not capture a frame.")
        cap.release()
        continue
    
    _, encoded_frame = cv2.imencode('.jpg', frame)
    response = requests.post(f'{ngrok_url}/video_frame', data=encoded_frame.tobytes())
    
    print(f"Source {source} response:", response.json())

    cap.release()

# Sending shutdown signal
requests.post(f'{ngrok_url}/shutdown')
