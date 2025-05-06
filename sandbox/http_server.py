import os
import webbrowser
from flask import Flask, send_from_directory
from werkzeug.serving import make_server
import socket

def get_local_ip():
    ip = "127.0.0.1"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

class HTTPServer:
    def __init__(self, host="0.0.0.0", port=8080):
        self.host = host
        self.port = port
        self.app = Flask(__name__, static_folder=None) 
        self.server = None

        self.app.add_url_rule('/', 'index', self.serve_html)
        self.app.add_url_rule('/<path:path>', 'static', self.serve_static)

    def serve_html(self):
        return self.serve_static("websocket_client.html")

    def serve_static(self, path):
        project_root = os.path.dirname(os.path.abspath(__file__))
        webclient_dir = os.path.join(project_root, '')
        return send_from_directory(webclient_dir, path)

    def stop(self):
        if self.server:
            self.server.shutdown()

    def run(self):
        url = f"http://localhost:{self.port}"
        print(f"Running on {url}")  # Este mensaje es clave para que VSCode lo detecte
        webbrowser.open(url)

        self.server = make_server(self.host, self.port, self.app)
        print(f"Listening on port {self.port}...")
        self.server.serve_forever()