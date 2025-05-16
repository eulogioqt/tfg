from enum import Enum

class COMMANDS(str, Enum):
    DELETE_USER = "DELETE_USER",
    RENAME_USER = "RENAME_USER",
    TAKE_PICTURE = "TAKE_PICTURE"
    
    UNKNOWN = "UNKNOWN"
