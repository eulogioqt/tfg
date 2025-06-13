"""TODO: Add module documentation."""
import threading


class WebSocketThreadMixer:

"""TODO: Describe class."""
    def __init__(self, websocket, *others):
    """TODO: Describe __init__.
Args:
    websocket (:obj:`Any`): TODO.
    *others (:obj:`Any`): TODO.
"""
        self.websocket = websocket
        self.others = others 
        self.threads = {}

    def run(self):
    """TODO: Describe run.
"""
        for other in self.others: # Arrancamos hilos
            thread = threading.Thread(target=other.run, daemon=False)
            thread.start()

            self.threads[other.__class__.__name__] = thread
            
        try: self.websocket.run() # Principal websockets
        finally:
            print("WebSocketServer finalizado.")

            for other in self.others: # Paramos hilos
                other.stop()

            for th_name in self.threads.keys(): # Esperamos su muerte
                self.threads[th_name].join()
                print(th_name + " finalizado.")

            print("Programa terminado correctamente.")
