import os
import json

from datetime import datetime
from threading import RLock


class FaceprintsDatabase:
    def __init__(self, db_path='faceprints_db.json'):
        self.db_path = db_path
        self.faceprints = {}
        self._lock = RLock()

    def get_all_names(self):
        with self._lock:
            return list(self.faceprints.keys())

    def get_all(self):
        with self._lock:
            return list(self.faceprints.values())

    def get_by_name(self, name):
        with self._lock:
            return self.faceprints.get(name)

    def add(self, name, features, face, score):
        with self._lock:
            if name in self.faceprints:
                return None  # Ya existe

            new_faceprint = {
                'name': name,
                'face': face,
                'face_score': score,
                'features': [features],
                'size': [1],
                'learning_date': datetime.now().timestamp()
            }
            self.faceprints[name] = new_faceprint

            return new_faceprint

    def update(self, name, new_data):
        with self._lock:
            original_entry = self.get_by_name(name)
            if original_entry is None:
                return None # No existe

            new_name = new_data.get("name")
            if new_name and new_name != name and new_name in self.faceprints:
                return None # Estas intentando cambiar de nombre a otro que ya existe

            for key, value in new_data.items(): # Actualizamos los campos
                if key in original_entry:
                    original_entry[key] = value
            
            if new_name and new_name != name: # Si son distintos, cambia nombre
                self.faceprints[new_name] = original_entry
                self.remove(name)
            else:
                self.faceprints[name] = original_entry

            return original_entry

    def remove(self, name):
        with self._lock:
            if name in self.faceprints:
                del self.faceprints[name]

    def save(self):
        with self._lock:
            with open(self.db_path, "w", encoding="utf-8") as f:
                json.dump(self.faceprints, f, indent=4)

    def load(self):
        with self._lock:
            os.makedirs(os.path.dirname(self.db_path) or ".", exist_ok=True)

            if not os.path.exists(self.db_path): # Si no existe, lo creamos vacio
                self.save()

            with open(self.db_path, "r", encoding="utf-8") as f:
                self.faceprints = json.load(f)

