from .tts_model import TTSModel
from .bark_tts import BarkTTS
from .css10_tts import CSS10TTS
from .google_tts import GoogleTTS
from .piper_tts import PiperTTS
from .tacotron2_tts import Tacotron2TTS
from .xtts import XTTS
from .your_tts import YourTTS

from enum import Enum


class SmartStrEnum(str, Enum):
    def __str__(self):
        return self.value

    def __repr__(self):
        return self.value

class TTS_MODELS(SmartStrEnum):
    BARK = "bark"
    CSS10 = "css10"
    GOOGLE = "google"
    PIPER = "piper"
    TACOTRON2 = "tacotron2"
    XTTS = "xtts"
    YOUR_TTS = "your_tts"

class TTS_SPEAKERS:
    class BARK(SmartStrEnum):
        ES_SPEAKER_0 = "es_speaker_0"

    class CSS10(SmartStrEnum):
        pass

    class GOOGLE(SmartStrEnum):
        ES = "es"
        COM_MX = "com.mx"
        US = "us"
    
    class PIPER(SmartStrEnum):
        DAVEFX = "davefx"
        SHARVARD = "sharvard"
    
    class TACOTRON2(SmartStrEnum):
        pass

    class XTTS(SmartStrEnum):
        ALMA_MARIA = "Alma María"
        ANA_FLORENCE = "Ana Florence"
        LUIS_MORAY = "Luis Moray"
        MARCOS_RUDASKI = "Marcos Rudaski"
        FERRAN_SIMEN = "Ferran Simen"
        XAVIER_HAYASAKA = "Xavier Hayasaka"

    class YOUR_TTS(SmartStrEnum):
        FEMALE_EN_5 = "female_en_5"
        MALE_EN_2 = "male_en_2"