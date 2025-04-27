import json

from .client_node import ClientNode
from .interfaces import SessionAPIInterface, JSONResponse


class SessionAPI(SessionAPIInterface):

    def __init__(self, node: ClientNode):
        self.node = node

    def get_all_sessions(self):
        faceprints_json = self.node.get_sessions_request()
        faceprints = json.loads(faceprints_json)

        return JSONResponse(content=faceprints)

    def get_session(self, name):
        faceprint_json = self.node.get_sessions_request(json.dumps({ "id": id }))
        faceprint = json.loads(faceprint_json)

        return JSONResponse(content=faceprint)
