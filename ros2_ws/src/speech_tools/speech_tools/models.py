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

class STT_MODELS(SmartStrEnum):
"""TODO: Describe class."""
    WHISPER = "whisper"
    GOOGLE = "google"

class TTS_MODELS(SmartStrEnum):
"""TODO: Describe class."""
    BARK = "bark"
    CSS10 = "css10"
    GOOGLE = "google"
    PIPER = "piper"
    TACOTRON2 = "tacotron2"
    XTTS = "xtts"
    YOUR_TTS = "your_tts"

class TTS_SPEAKERS:
"""TODO: Describe class."""
    class BARK(SmartStrEnum):
    """TODO: Describe class."""
        ES_SPEAKER_0 = "es_speaker_0"

    class CSS10(SmartStrEnum):
    """TODO: Describe class."""
        pass

    class GOOGLE(SmartStrEnum):
    """TODO: Describe class."""
        ES = "es"
        COM_MX = "com.mx"
        US = "us"
    
    class PIPER(SmartStrEnum):
    """TODO: Describe class."""
        DAVEFX = "davefx"
        SHARVARD = "sharvard"
    
    class TACOTRON2(SmartStrEnum):
    """TODO: Describe class."""
        pass

    class XTTS(SmartStrEnum):
    """TODO: Describe class."""
        ALMA_MARIA = "Alma María"
        ANA_FLORENCE = "Ana Florence"
        LUIS_MORAY = "Luis Moray"
        VIKTOR_EKA = "Viktor Eka"
        MARCOS_RUDASKI = "Marcos Rudaski"
        FERRAN_SIMEN = "Ferran Simen"
        XAVIER_HAYASAKA = "Xavier Hayasaka"

    class YOUR_TTS(SmartStrEnum):
    """TODO: Describe class."""
        FEMALE_EN_5 = "female-en-5"
        MALE_EN_2 = "male-en-2"

STT_NEEDS_API_KEY = {
    STT_MODELS.GOOGLE
}

TTS_NEEDS_API_KEY = {

}
