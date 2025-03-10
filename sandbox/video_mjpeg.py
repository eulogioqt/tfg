from flask import Flask, Response
import cv2

app = Flask(__name__)

# Abre la cámara
cap = cv2.VideoCapture(0)

def generate():
    first_print = True
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        if first_print:
            first_print = False
            print(frame.shape)

        # Codifica la imagen como JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            break
        
        # Convierte la imagen JPEG a bytes y envía el flujo MJPEG
        frame_bytes = jpeg.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n\r\n')

@app.route('/video')
def video():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080)
