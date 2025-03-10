import asyncio
import websockets
import cv2
import base64

async def send_video(websocket, path):
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara.")
        return
    
    first_print = True
    while True:
        # Lee un frame
        ret, frame = cap.read()

        if not ret:
            print("Error: No se pudo leer el frame.")
            break

        if first_print:
            first_print = False
            print(frame.shape)

        # Convierte el frame a JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)

        if ret:
            # Convierte el JPEG a base64 para enviarlo por WebSocket
            frame_base64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
            
            # Enviar el frame codificado a través de WebSocket
            await websocket.send(frame_base64)
        await asyncio.sleep(0.01)  # Ajusta la velocidad de la transmisión

    cap.release()

# Inicia el servidor WebSocket
async def main():
    async with websockets.serve(send_video, "localhost", 8765):
        await asyncio.Future()  # Corre el servidor indefinidamente

# Ejecuta el servidor
asyncio.run(main())
