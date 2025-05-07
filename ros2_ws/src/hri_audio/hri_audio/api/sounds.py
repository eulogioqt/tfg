from .sound import load

activation_path = 'install/hri_audio/share/hri_audio/sounds/activation_sound.wav'
time_out_path = 'install/hri_audio/share/hri_audio/sounds/time_out_sound.wav'

ACTIVATION_SOUND = load(activation_path)
TIME_OUT_SOUND = load(time_out_path)
