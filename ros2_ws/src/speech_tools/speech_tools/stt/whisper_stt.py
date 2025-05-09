import os
import torch
import torchaudio
import tempfile

from faster_whisper import WhisperModel

from .stt_model import STTModel


# a futuro hacer faster whisper y whisper como difernetes, si eso meter mas STT y poner lo de provider y model como en los llms
class WhisperSTT(STTModel):
    def __init__(self, model_size: str = "base", device: str = "cuda", compute_type: str = "float16"):
        self.model = WhisperModel(model_size, device=device, compute_type=compute_type)

    def transcribe(self, audio: bytes, sample_rate: int) -> str:
        try:
            audio_tensor = torch.tensor(torch.frombuffer(audio, dtype=torch.int16).float() / 32768.0).unsqueeze(0)

            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmpfile:
                tmp_path = tmpfile.name
                torchaudio.save(tmp_path, audio_tensor, sample_rate)

            segments, _ = self.model.transcribe(tmp_path, language="es")
            text = " ".join([segment.text for segment in segments])

            return text

        except Exception as e:
            print(f"[WhisperSTT] Error: {e}")
            return ""

        finally:
            if 'tmp_path' in locals() and os.path.exists(tmp_path):
                os.remove(tmp_path)
