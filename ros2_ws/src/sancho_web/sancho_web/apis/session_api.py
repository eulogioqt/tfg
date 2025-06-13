"""TODO: Add module documentation."""
import json

from ..engines import SessionEngine
from .api_responses import JSONResponse, APIResponse


class SessionAPI:

"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        self.engine = SessionEngine(node)

    def get_all_sessions(self, faceprint_id: str = None) -> APIResponse:
    """TODO: Describe get_all_sessions.
Args:
    faceprint_id (:obj:`Any`): TODO.
"""
        if faceprint_id is not None:
            sessions_json = self.engine.get_sessions_request(json.dumps({ "faceprint_id": faceprint_id }))
        else:
            sessions_json = self.engine.get_sessions_request()

        sessions = json.loads(sessions_json)

        return JSONResponse(content=sessions)

    def get_session_by_id(self, id: str) -> APIResponse:
    """TODO: Describe get_session_by_id.
Args:
    id (:obj:`Any`): TODO.
"""
        session_json = self.engine.get_sessions_request(json.dumps({ "id": id }))
        session = json.loads(session_json)

        return JSONResponse(content=session)

    def get_sessions_summary(self) -> APIResponse:
    """TODO: Describe get_sessions_summary.
"""
        sessions_summary_json = self.engine.get_sessions_summary_request()
        sessions_summary = json.loads(sessions_summary_json)

        return JSONResponse(content=sessions_summary)    
