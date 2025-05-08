from .coqui_single_tts import CoquiSingleTTS


class Tacotron2TTS(CoquiSingleTTS):

    def __init__(self):
        super().__init__(
            model="tts_models/es/mai/tacotron2-DDC"
        )

    def get_speakers(self) -> list[str]:
        return [""]