from abc import ABC, abstractmethod
import fastapi.responses as responses

class APIResponse(ABC):
    def __init__(self, status_code, content):
        self.status_code = status_code
        self.content = content
    
    def to_fastapi(self):
        return responses.JSONResponse(status_code=self.status_code, content=self.content)

class HTTPException(APIResponse):
    def __init__(self, status_code=400, detail=""):
        super().__init__(status_code, {"detail": detail})

class JSONResponse(APIResponse):
    def __init__(self, status_code=200, content=""):
        super().__init__(status_code, content)

class FaceprintDatabaseInterface(ABC):

    @abstractmethod
    def get_all_faceprints(self) -> APIResponse:
        pass

    @abstractmethod
    def get_faceprint(self, name: str) -> APIResponse:
        pass

    @abstractmethod
    def create_faceprint(self, name: str, image_base64: str) -> APIResponse:
        pass

    @abstractmethod
    def update_faceprint(self, name: str, faceprint: str) -> APIResponse:
        pass

    @abstractmethod
    def delete_faceprint(self, name: str) -> APIResponse:
        pass