import os
import json

from datetime import datetime
from threading import RLock


class FaceprintsDatabase:
    def __init__(self, db_path='faceprints_db.json'):
        self.db_path = db_path
        self.faceprints = {}
        self._lock = RLock()

    def get_all(self):
        with self._lock:
            return list(self.faceprints.values())

    def get_by_name(self, name):
        with self._lock:
            return self.faceprints.get(name)

    def add(self, name, features):#, face
        with self._lock:
            if name in self.faceprints:
                return None  # Ya existe

            new_faceprint = {
                'name': name,
                # 'face': face,
                'features': [features],
                'size': [1],
                'learning_date': datetime.now().timestamp()
            }
            self.faceprints[name] = new_faceprint

            return new_faceprint

    def update(self, name, new_data):
        with self._lock:
            existing = self.get_by_name(name)
            if existing is None:
                return None

            for key, value in new_data.items():
                if key in existing:
                    existing[key] = value

            new_name = existing["name"]
            if new_name != name:
                self.faceprints[new_name] = existing
                self.remove(name)
            else:
                self.faceprints[name] = existing

            return existing

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

