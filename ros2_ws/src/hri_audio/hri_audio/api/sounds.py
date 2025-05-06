import os

from .sound import load

dirname = os.path.dirname(os.path.dirname(__file__))

activationPath = 'install/hri_audio/share/hri_audio/sounds/activation_sound.wav'
timeOutPath = 'install/hri_audio/share/hri_audio/sounds/time_out_sound.wav'

ACTIVATION_SOUND = load(activationPath)
TIME_OUT_SOUND = load(timeOutPath)
