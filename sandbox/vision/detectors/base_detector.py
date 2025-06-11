from abc import ABC, abstractmethod


class BaseDetector(ABC):

    @abstractmethod
    def get_faces(frame):
        pass