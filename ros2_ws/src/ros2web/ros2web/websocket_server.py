"""TODO: Add module documentation."""
import asyncio
import websockets


class WebSocketServer:

"""TODO: Describe class."""
    def __init__(self, on_message, on_user_connect, on_user_disconnect, port=8765):
    """TODO: Describe __init__.
Args:
    on_message (:obj:`Any`): TODO.
    on_user_connect (:obj:`Any`): TODO.
    on_user_disconnect (:obj:`Any`): TODO.
    port (:obj:`Any`): TODO.
"""
        self.port = port

        self.on_message = on_message
        self.on_user_connect = on_user_connect
        self.on_user_disconnect = on_user_disconnect

        self.clients = {}
        self.stop_event = asyncio.Event()

    def get_client(self, key):
    """TODO: Describe get_client.
Args:
    key (:obj:`Any`): TODO.
"""
        return self.clients.get(key)

    def get_connection_count(self):
    """TODO: Describe get_connection_count.
"""
        return len(self.clients.keys())

    def broadcast_message(self, message):
    """TODO: Describe broadcast_message.
Args:
    message (:obj:`Any`): TODO.
"""
        asyncio.run(self._send_to_all(message))

    def send_message(self, sender, message):
    """TODO: Describe send_message.
Args:
    sender (:obj:`Any`): TODO.
    message (:obj:`Any`): TODO.
"""
        asyncio.run(self._send_message(sender, message))

    def stop_program(self):
    """TODO: Describe stop_program.
"""
        if not self.stop_event.is_set():
            self.stop_event.set()

    async def _handler(self, websocket): # Depende de la version hay que poner un argumento, path despues de websocket
    """TODO: Describe _handler.
Args:
    websocket (:obj:`Any`): TODO.
"""
        client_ip, client_port = websocket.remote_address
        key = f"{client_ip}:{client_port}"

        self.clients[key] = websocket
        response = self.on_user_connect(key)
        if response is not None:
            await self._send_message(websocket, response)

        try:
            async for message in websocket:
                response = self.on_message(key, message)
                if response is not None:
                    await self._send_message(websocket, response)
                    
        except Exception as e:
            print(f"Error en el cliente: {e}")
        finally:
            if key in self.clients.keys():
                self.clients.pop(key)

            self.on_user_disconnect(key)

    async def _broadcast(self):
    """TODO: Describe _broadcast.
"""
        try:
            while not self.stop_event.is_set():
                if not self.queue.empty():
                    if len(self.clients.keys()) > 0:
                        msg = self.queue.get()

                        asyncio.run(self._send_to_all(msg))
                        await asyncio.sleep(0)
                else:
                    await asyncio.sleep(1)
        finally:
            pass

    async def _send_to_all(self, message):
    """TODO: Describe _send_to_all.
Args:
    message (:obj:`Any`): TODO.
"""
        if len(self.clients.keys()) > 0:
            tasks = [asyncio.create_task(self._send_message(client, message)) for client in self.clients.values()]
            await asyncio.wait(tasks)

    async def _send_message(self, client, message):
    """TODO: Describe _send_message.
Args:
    client (:obj:`Any`): TODO.
    message (:obj:`Any`): TODO.
"""
        try:
            await client.send(str(message))
        except Exception as e:
            print(f"Error al enviar mensaje al cliente: {e}")

    async def _websocket_server(self):
    """TODO: Describe _websocket_server.
"""
        try:
            async with websockets.serve(self._handler, "0.0.0.0", self.port):
                print(f"WebSocket running on https://localhost:{self.port}")
                await self.stop_event.wait()
        finally:
            for client in self.clients.values():
                await client.close()

    async def _main(self): # Revisar esto, ahora mismo el broadcast se hace en el spin de server, pero lo suyo seria que se hiciese asi
    """TODO: Describe _main.
"""
        websocket_task = asyncio.create_task(self._websocket_server())
        broadcast_task = asyncio.create_task(self._broadcast())

        try:
            await asyncio.gather(websocket_task, broadcast_task)
        finally:
            self.stop_program()

    def run(self):
    """TODO: Describe run.
"""
        try:
            asyncio.run(self._websocket_server())
        except KeyboardInterrupt: pass
        except Exception: pass
        finally: pass
