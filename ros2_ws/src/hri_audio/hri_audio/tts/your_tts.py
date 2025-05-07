from .coqui_multi_tts import CoquiMultiTTS


class YourTTS(CoquiMultiTTS):

    def __init__(self):
        super().__init__(
            model="tts_models/multilingual/multi-dataset/your_tts",
            language="en"    
        )
