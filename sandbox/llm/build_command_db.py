import os
import json
import openai

from dotenv import load_dotenv

load_dotenv()

client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

def get_embedding(text):
    return client.embeddings.create(
        model="text-embedding-3-small",
        input=[text]
    ).data[0].embedding

# Base por ejemplo
command_db = {}

with open("commands.json", "r", encoding="utf-8") as f:
    data = json.load(f)

for command, examples in data.items():
    command_db[command] = []
    for ex in examples:
        emb = get_embedding(ex)
        command_db[command].append({
            "text": ex,
            "embedding": emb
        })

with open("command_db_by_example.json", "w", encoding="utf-8") as f:
    json.dump(command_db, f)