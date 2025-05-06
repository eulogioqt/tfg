from models import PiperTTS
import time

model = PiperTTS()
start = time.time()
text = "La inteligencia artificial está transformando rápidamente el mundo en que vivimos, desde asistentes personales hasta sistemas autónomos que aprenden, razonan y actúan. Modelos avanzados de lenguaje como ChatGPT son capaces de comprender contextos complejos, generar contenido original y adaptarse a tareas específicas con una precisión sorprendente. Esta revolución tecnológica plantea tanto oportunidades como desafíos éticos y sociales, especialmente en ámbitos como la educación, el empleo y la privacidad. Es esencial que el desarrollo de estas tecnologías esté guiado por principios sólidos de transparencia, equidad y responsabilidad. En paralelo, los usuarios deben aprender a interactuar con estos sistemas de forma crítica, entendiendo sus límites y potencial. El futuro dependerá de la colaboración entre humanos y máquinas, donde el conocimiento, la creatividad y la empatía seguirán siendo cualidades exclusivamente humanas, pero aumentadas por el poder de la inteligencia artificial."
audio = model.synthesize(text)
end = time.time() - start
model.play(audio, model.get_sample_rate(), wait=False)
model.save(audio, model.get_sample_rate(), "./xd.wav")

print(text)
print(f"Longitud: {len(text)}")
print(f"Tiempo: {end:.2f}")
print(f"Velocidad: {len(text) / end}")