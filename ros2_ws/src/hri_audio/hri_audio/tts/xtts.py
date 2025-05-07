from .coqui_multi_tts import CoquiMultiTTS


class XTTS(CoquiMultiTTS):

    def __init__(self, speaker="Alma María"):
        super().__init__(
            speaker=speaker,
            model="tts_models/multilingual/multi-dataset/xtts_v2",
            language="es"    
        )
