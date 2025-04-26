import json

from .client_node import ClientNode
from .interfaces import LogAPIInterface, HTTPException, JSONResponse


class LogAPI(LogAPIInterface):

    def __init__(self, node: ClientNode):
        self.node = node

    def get_all_logs(self):
        faceprints_json = self.node.get_faceprint_request()
        faceprints = json.loads(faceprints_json)

        return JSONResponse(content=faceprints)

    def get_log(self, name):
        faceprint_json = self.node.get_faceprint_request(json.dumps({ "name": name }))

        faceprint = json.loads(faceprint_json)
        if faceprint is None:
            return HTTPException(status_code=404, detail=f"Faceprint con nombre {name} no encontrado")
        
        return JSONResponse(content=faceprint)
