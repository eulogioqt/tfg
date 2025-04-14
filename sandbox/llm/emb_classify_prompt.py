import os
import json
import openai

from dotenv import load_dotenv
from scipy.spatial.distance import cosine

load_dotenv()

client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

THRESHOLD = 0.70  # más bajo porque comparamos con frases individuales

def get_embedding(text):
    return client.embeddings.create(
        model="text-embedding-3-small",
        input=[text]
    ).data[0].embedding

def classify(prompt, command_db):
    prompt_emb = get_embedding(prompt)
    best_score = -1
    best_command = None

    for command, examples in command_db.items():
        for example in examples:
            score = 1 - cosine(prompt_emb, example["embedding"])
            print(f"Similitud con '{command}' / '{example['text']}': {score:.4f}")
            if score > best_score:
                best_score = score
                best_command = command

    if best_score >= THRESHOLD:
        return best_command, best_score
    else:
        return "NINGUNO", best_score

if __name__ == "__main__":
    with open("command_db_by_example.json", "r", encoding="utf-8") as f:
        db = json.load(f)

    while True:
        text = input("\n🗣️ Escribe algo: ")
        if text.lower() == "salir":
            break
        resultado, score = classify(text, db)
        print(f"✅ Clasificado como: {resultado} (similitud: {score:.4f})")