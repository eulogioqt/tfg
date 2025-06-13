"""TODO: Add module documentation."""
import fastapi.responses as responses

from abc import ABC


class APIResponse(ABC):
"""TODO: Describe class."""
    def __init__(self, status_code, content):
    """TODO: Describe __init__.
Args:
    status_code (:obj:`Any`): TODO.
    content (:obj:`Any`): TODO.
"""
        self.status_code = status_code
        self.content = content
    
    def to_fastapi(self):
    """TODO: Describe to_fastapi.
"""
        return responses.JSONResponse(status_code=self.status_code, content=self.content)

class HTTPException(APIResponse):
"""TODO: Describe class."""
    def __init__(self, status_code=400, detail=""):
    """TODO: Describe __init__.
Args:
    status_code (:obj:`Any`): TODO.
    detail (:obj:`Any`): TODO.
"""
        super().__init__(status_code, {"detail": detail})

class JSONResponse(APIResponse):
"""TODO: Describe class."""
    def __init__(self, status_code=200, content=""):
    """TODO: Describe __init__.
Args:
    status_code (:obj:`Any`): TODO.
    content (:obj:`Any`): TODO.
"""
        super().__init__(status_code, content)
