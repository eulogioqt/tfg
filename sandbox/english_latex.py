import re
from pathlib import Path
from shutil import copytree, rmtree

# Lista de palabras técnicas en inglés (minúsculas)
english_words = {
    "hardware", "software", "frontend", "backend", "framework", "embedding", "embeddings", "dataset", "chunk",
    "hotword", "streaming", "input", "output", "prompt", "websocket", "session", "sessions",
    "faceprint", "pipeline", "feature", "listener", "speaker", "faceprints", "logger", "log", "logs",
    "event", "events", "interface", "prompting", "cloud", "model", "models", "token",
    "tokens", "default", "wrapper", "provider", "providers", "server", "client", "clients", "update",
    "websockets", "voiceprint", "helper", "manager", "database", "callback", "worker",
    "reset", "shutdown", "monitoring", "storage", "slot", "alias", "setup", "localhost", "timestamp",
    "training", "inference", "fine-tuning", "container", "abstract", "launch", "thread", "debug",
    "endpoint", "script", "testing", "validate", "performance", "latency", "runtime", "deployment",
    "switch", "stream", "match", "component", "idempotent",
    "singleton", "boolean", "state", "flush", "cache", "getter", "setter", "event-driven",
    "middleware", "constructor", "instance", "microservice", "hot-reload", "scroll", "scrolling",
    "layout", "style", "tooltip", "avatar"
}


counter = 0  # contador global

def already_cursivated(text, start, end):
    """Comprueba si ya está dentro de \textit{}"""
    before = text[:start]
    after = text[end:]
    open_pos = before.rfind(r"\textit{")
    close_pos = before.rfind("}")
    return open_pos > close_pos

def is_invalid_context(text, start, end):
    """Devuelve True si está dentro de una palabra técnica (., _, \_, / alrededor)"""
    before = text[start - 1] if start > 0 else ''
    after = text[end] if end < len(text) else ''
    invalid_chars = {'.', '/', '_'}
    if before in invalid_chars or after in invalid_chars:
        return True
    if text[start-2:start] == r"\_" or text[end:end+2] == r"\_":
        return True
    return False

def cursivar_ingles(text):
    global counter
    pattern = re.compile(r'\b([A-Za-z][\w\-]*)\b(?:\s+([A-Za-z][\w\-]*))?')
    replacements = []

    for match in pattern.finditer(text):
        w1, w2 = match.group(1), match.group(2)
        start, _ = match.start(), match.end()
        w1_lower = w1.lower()
        w2_lower = w2.lower() if w2 else None

        if w1_lower in english_words:
            if w2 and w2_lower in english_words:
                phrase = f"{w1} {w2}"
                end = start + len(phrase)
            else:
                phrase = w1
                end = match.start(1) + len(w1)

            if not already_cursivated(text, start, end) and not is_invalid_context(text, start, end):
                replacements.append((start, end, f"\\textit{{{phrase}}}"))
                counter += 1
                print(f"{counter}.- Se ha cursivado: {phrase}")

    # Aplicar desde el final para que no se desplacen los índices
    for start, end, replacement in reversed(replacements):
        text = text[:start] + replacement + text[end:]

    return text

# Rutas
input_dir = Path("./tex_input")
output_dir = Path("./tex_output")

# Preparar estructura
if output_dir.exists():
    rmtree(output_dir)
copytree(input_dir, output_dir)

# Procesar todos los archivos .tex
for tex_file in output_dir.rglob("*.tex"):
    with open(tex_file, "r", encoding="utf-8") as f:
        content = f.read()

    new_content = cursivar_ingles(content)

    with open(tex_file, "w", encoding="utf-8") as f:
        f.write(new_content)

print(f"\n✅ Todo listo. Total de palabras cursivadas: {counter}. Archivos modificados en la carpeta 'tex_output'.")
