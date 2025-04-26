import rclpy
import uvicorn

from .service.app import app
from .service.v1_faceprints import set_faceprint_api
from .service.v1_logs import set_log_api

from .client_node import ClientNode

from .faceprint_api import FaceprintAPI
from .log_api import LogAPI

def main(args=None):
    rclpy.init(args=args)

    client_node = ClientNode()

    faceprint_api = FaceprintAPI(node=client_node)
    set_faceprint_api(faceprint_api)

    log_api = LogAPI(node=client_node)
    set_log_api(log_api)

    uvicorn.run(app, host="localhost", port=7654)

    rclpy.shutdown()