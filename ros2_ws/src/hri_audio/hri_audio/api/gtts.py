from gtts import gTTS
from io import BytesIO
from pydub import AudioSegment
import numpy as np

ACCENT_MEXICO = 'com.mx'
ACCENT_SPAIN = 'es'
ACCENT_US = 'us'

def tts(text, language='es', accent=ACCENT_SPAIN):
    mp3_fp = BytesIO()
    tts = gTTS(text, lang=language, tld=accent)
    tts.write_to_fp(mp3_fp)
    mp3_fp.seek(0)

    audio = AudioSegment.from_mp3(mp3_fp)
    audio = audio.set_channels(1)
    audio = audio.set_frame_rate(22050)

    samples = np.array(audio.get_array_of_samples()).astype(np.float32)
    
    samples /= np.iinfo(audio.array_type).max

    sample_rate = audio.frame_rate
    return samples.tolist(), sample_rate
