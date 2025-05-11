import rclpy
import uvicorn

from .service.app import app
from .service.v1_faceprints import set_faceprint_api
from .service.v1_logs import set_log_api
from .service.v1_sessions import set_session_api

from .engines import ServiceEngine

from .apis import FaceprintAPI, LogAPI, SessionAPI


def main(args=None):
    rclpy.init(args=args)

    client_node = ServiceEngine.create_client_node()

    faceprint_api = FaceprintAPI(node=client_node)
    set_faceprint_api(faceprint_api)

    log_api = LogAPI(node=client_node)
    set_log_api(log_api)

    session_api = SessionAPI(node=client_node)
    set_session_api(session_api)

    uvicorn.run(app, host="0.0.0.0", port=7654)

    rclpy.shutdown()