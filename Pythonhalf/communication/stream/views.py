import cv2
from django.http import StreamingHttpResponse
from django.shortcuts import render

def generate_camera_stream():
    cap = cv2.VideoCapture(0)  # 0 = default webcam

    while True:
        success, frame = cap.read()
        if not success:
            break

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

def camera_feed(request):
    return StreamingHttpResponse(
        generate_camera_stream(),
        content_type='multipart/x-mixed-replace; boundary=frame'
    )
    
def mediapipe(request, *args, **kwargs):
    return render(request, 'stream/mediapipe.html')