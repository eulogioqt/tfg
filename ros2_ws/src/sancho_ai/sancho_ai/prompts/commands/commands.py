"""TODO: Add module documentation."""
from enum import Enum

class SmartStrEnum(str, Enum):
"""TODO: Describe class."""
    def __str__(self):
    """TODO: Describe __str__.
"""
        return self.value

    def __repr__(self):
    """TODO: Describe __repr__.
"""
        return self.value

class COMMANDS(SmartStrEnum):
"""TODO: Describe class."""
    DELETE_USER = "DELETE_USER",
    RENAME_USER = "RENAME_USER",
    TAKE_PICTURE = "TAKE_PICTURE"
    
    UNKNOWN = "UNKNOWN"

class COMMAND_RESUITS(SmartStrEnum):
"""TODO: Describe class."""
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    MISSING_ARGUMENT = "MISSING_ARGUMENT"
