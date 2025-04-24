import rclpy
import uvicorn

from .service.app import app
from .service.v1 import set_faceprint_api

from .faceprint_api import FaceprintAPI


def main(args=None):
    rclpy.init(args=args)

    faceprint_api = FaceprintAPI()
    set_faceprint_api(faceprint_api)
    uvicorn.run(app, host="localhost", port=7654)

    rclpy.shutdown()