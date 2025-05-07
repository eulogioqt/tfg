from .coqui_multi_tts import CoquiMultiTTS


class YourTTS(CoquiMultiTTS):

    def __init__(self, speaker="female-en-5"):
        super().__init__(
            speaker=speaker,
            model="tts_models/multilingual/multi-dataset/your_tts",
            language="en"    
        )
