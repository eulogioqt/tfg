import numpy as np
from unittest.mock import Mock, patch

from hri_audio.utils.stt_hotword import STTHotword

# Black-box: validate _check_hotword identifies name regardless of formatting

def test_check_hotword_positive():
    hotword = STTHotword(lambda *args, **kwargs: "")
    assert hotword._check_hotword("Hola Sancho") is True


def test_check_hotword_negative():
    hotword = STTHotword(lambda *args, **kwargs: "")
    assert hotword._check_hotword("Hello robot") is False


# White-box: ensure _transcribe handles exceptions and returns empty string

def test_transcribe_handles_exception():
    stt_fn = Mock(side_effect=Exception("fail"))
    hotword = STTHotword(stt_fn)
    result = hotword._transcribe(np.array([1, 2], dtype=np.int16), 16000)
    assert result == ""


# Hybrid: detect should call _transcribe and _check_hotword when buffer exceeds limit

def test_detect_calls_transcribe_and_check():
    stt_fn = Mock(return_value="")
    hotword = STTHotword(stt_fn, max_seconds=0.1)
    with patch.object(STTHotword, "_transcribe", return_value="test") as trans_mock, \
         patch.object(STTHotword, "_check_hotword", return_value=False) as check_mock:
        hotword.detect([0] * 1600, 16000)  # 0.1s of audio
        trans_mock.assert_called_once()
        check_mock.assert_called_once_with("test")
