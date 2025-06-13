"""TODO: Add module documentation."""
import json

from ..engines import LogEngine
from .api_responses import JSONResponse, APIResponse


class LogAPI:

"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        self.engine = LogEngine(node)

    def get_all_logs(self) -> APIResponse:
    """TODO: Describe get_all_logs.
"""
        logs_json = self.engine.get_logs_request()
        
        logs = json.loads(logs_json)

        return JSONResponse(content=logs)

    def get_log_by_id(self, id: str) -> APIResponse:
    """TODO: Describe get_log_by_id.
Args:
    id (:obj:`Any`): TODO.
"""
        log_json = self.engine.get_logs_request(json.dumps({ "id": id }))
        log = json.loads(log_json)

        return JSONResponse(content=log)
