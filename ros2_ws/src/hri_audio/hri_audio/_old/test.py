"""TODO: Add module documentation."""
import pyttsx3

def list_voices():
"""TODO: Describe list_voices.
"""
    engine = pyttsx3.init()
    voices = engine.getProperty('voices')
    
    for i, voice in enumerate(voices):
        print(f"Voice {i}:")
        print(f" - ID: {voice.id}")
        print(f" - Name: {voice.name}")
        print(f" - Languages: {voice.languages}")
        print(f" - Gender: {voice.gender}")
        print(f" - Age: {voice.age}")

list_voices()
