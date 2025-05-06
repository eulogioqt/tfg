import asyncio
import websockets
import cv2
import base64
import json

async def send_video(websocket):
    video_path = './video.mp4' 
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: No se pudo abrir el video.")
        return
    
    first_print = True
    while True:
        ret, frame = cap.read()

        if not ret:
            print("El video ha terminado, reiniciando.")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0) 
            continue

        if first_print:
            first_print = False
            print(frame.shape)

        ret, jpeg = cv2.imencode('.jpg', frame)

        if ret:
            frame_base64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')

            message = {
                "type": "DISPLAY_DATA",
                "data": {
                    "type": "IMAGE",
                    "value": frame_base64
                }
            }
            message_json = json.dumps(message)

            await websocket.send(message_json)
       # await asyncio.sleep(0.01) 

async def main():
    async with websockets.serve(send_video, "0.0.0.0", 8765):
        url = f"http://localhost:{8765}"
        print(f"Running on {url}")  # Este mensaje es clave para que VSCode lo detecte

        print(f"Listening on port {8765}...")
        await asyncio.Future()

asyncio.run(main())
