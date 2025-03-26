import asyncio
import websockets

from queue import Queue


class WebSocketServer:

    def __init__(self, on_message, on_user_connect, on_user_disconnect, port=8765):
        self.port = port

        self.on_message = on_message
        self.on_user_connect = on_user_connect
        self.on_user_disconnect = on_user_disconnect

        self.clients = set()
        self.queue = Queue()
        self.stop_event = asyncio.Event()

    def get_connection_count(self):
        return len(self.clients)

    def broadcast_message(self, msg):
        self.queue.put(msg)

    def stop_program(self):
        if not self.stop_event.is_set():
            self.stop_event.set()

    async def _handler(self, websocket, path): # Hacer objeto websocket propio para wrappear al de websockets y tener abstraccion
        client_ip, client_port = websocket.remote_address

        self.clients.add(websocket)
        response = self.on_user_connect(client_ip, client_port)
        if response is not None:
            await self._send_message(websocket, response)

        try:
            async for message in websocket:
                response = self.on_message(message)
                if response is not None:
                    await self._send_message(websocket, response)
                    
        except Exception as e:
            print(f"Error en el cliente: {e}")
        finally:
            self.clients.remove(websocket)
            self.on_user_disconnect(client_ip, client_port)

    async def _broadcast(self):
        try:
            while not self.stop_event.is_set():
                if not self.queue.empty():
                    if self.clients:
                        msg = self.queue.get()

                        await asyncio.gather(*[self._send_message(client, msg) for client in self.clients]) # LO SABIA RETURNS EXCEPTION
                        await asyncio.sleep(0)
                else:
                    await asyncio.sleep(1)
        finally:
            pass

    async def _send_message(self, client, message):
        try:
            await client.send(str(message))
        except Exception as e:
            print(f"Error al enviar mensaje al cliente: {e}")

    async def _websocket_server(self):
        try:
            async with websockets.serve(self._handler, "0.0.0.0", self.port):
                await self.stop_event.wait()
        finally:
            for client in self.clients:
                await client.close()

    async def _main(self):    
        websocket_task = asyncio.create_task(self._websocket_server())
        broadcast_task = asyncio.create_task(self._broadcast())

        try:
            await asyncio.gather(websocket_task, broadcast_task)
        finally:
            self.stop_program()

    def run(self):
        try: 
            asyncio.run(self._main())
        except KeyboardInterrupt: pass
        except Exception: pass
        finally: pass