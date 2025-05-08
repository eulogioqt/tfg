from .coqui_multi_tts import CoquiMultiTTS


class YourTTS(CoquiMultiTTS):

    def __init__(self):
        super().__init__(
            model="tts_models/multilingual/multi-dataset/your_tts",
            language="en"    
        )

    def get_speakers(self) -> list[str]:
        return ["female_en_5"]