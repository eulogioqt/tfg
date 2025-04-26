import rclpy
import uvicorn

from .service.app import app
from .service.v1_faceprints import set_faceprint_api
from .service.v1_logs import set_log_api
from .service.v1_sessions import set_session_api

from .client_node import ClientNode

from .faceprint_api import FaceprintAPI
from .log_api import LogAPI
from .session_api import SessionAPI

def main(args=None):
    rclpy.init(args=args)

    client_node = ClientNode()

    faceprint_api = FaceprintAPI(node=client_node)
    set_faceprint_api(faceprint_api)

    log_api = LogAPI(node=client_node)
    set_log_api(log_api)

    session_api = SessionAPI(node=client_node)
    set_session_api(session_api)

    uvicorn.run(app, host="localhost", port=7654)

    rclpy.shutdown()