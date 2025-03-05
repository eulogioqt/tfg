import websockets
import asyncio
import random

from websockets import WebSocketServerProtocol

async def send_data(websocket: WebSocketServerProtocol, path):
    while True:
        n = random.randint(1, 100)
        n_encoded = str(n).encode("utf-8")

        print(f"Enviando {n} a {websocket.remote_address[0]}:{websocket.remote_address[1]}")
        await websocket.send(n_encoded)
        
        await asyncio.sleep(1)

async def main():
    async with websockets.serve(send_data, "localhost", 8765):
        print("Servidor WebSocket inicializado correctamente [127.0.0.1:8765]")
        await asyncio.Future()

asyncio.run(main())