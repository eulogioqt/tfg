import websockets
import asyncio
import random
import json

from websockets import WebSocketServerProtocol

async def handle_client(websocket: WebSocketServerProtocol, path=None):
    async def send_message(message_dict, verbose=True):
        message_json = json.dumps(message_dict)
        if verbose:
            print(f"Enviando {message_dict['type']} a {websocket.remote_address[0]}:{websocket.remote_address[1]}")

        await websocket.send(message_json)

    async def send_data():
        while True:
            n = random.randint(1, 100)
            message_dict = {
                "type": "DISPLAY_DATA",
                "data": {
                    "type": "NUMBER",
                    "value": n
                }
            }
            
            await send_message(message_dict, verbose=False)
            await asyncio.sleep(1)

    async def receive_data():
        async for message in websocket:
            message_json = json.loads(message) # hacer lo del formato y todo el rollo para hacerlo bien que lo he cambiao con el uuid
            print(f"Recibido de {websocket.remote_address[0]}:{websocket.remote_address[1]} → {message}")

            message_reversed = message[::-1]
            message_dict = {
                "type": "RESPONSE",
                "data": message_reversed
            }
            
            await send_message(message_dict)

    await asyncio.gather(send_data(), receive_data())

async def main():
    async with websockets.serve(handle_client, "localhost", 8765):
        url = f"http://localhost:{8765}"
        print(f"Running on {url}")  # Este mensaje es clave para que VSCode lo detecte

        print(f"Listening on port {8765}...")
        await asyncio.Future()

asyncio.run(main())
