import json
from datetime import datetime
from tinydb import TinyDB, Query


class FaceprintsDatabase:
    def __init__(self, db_path='faceprints_db.json'):
        self.db = TinyDB(db_path)
        self.User = Query()

    def add_user(self, name, features, size):#, face):
        if self.db.search(self.User.name == name):
            return False  # Ya existe
        self.db.insert({
            'name': name,
            'features': json.dumps(features),
            'size': json.dumps(size),
            'learning_date': self._timestamp(),
            #'face': face
        })
        return True

    def update_user(self, name, new_features=None, new_size=None):#, new_face):
        update_data = {}
        if new_features:
            update_data['features'] = json.dumps(new_features)
        if new_size:
            update_data['size'] = json.dumps(new_size)
        #if new_face:
        #    update_data['face'] = new_face
        if update_data:
            self.db.update(update_data, self.User.name == name)
            return True
        return False

    def remove_user(self, name):
        self.db.remove(self.User.name == name)

    def get_user_by_name(self, name):
        results = self.db.search(self.User.name == name)
        return results[0] if results else None

    def get_all_users(self):
        return self.db.all()

    def save_from_dictionary(self, users_dict_characteristics, users_dict_size):#, users_dict_faces):
        for name, features in users_dict_characteristics.items():
            size = users_dict_size.get(name, {})
            #face = users_dict_faces.get(name, "")
            existing = self.get_user_by_name(name)
            if existing:
                self.update_user(name, features, size)#, face)
            else:
                self.add_user(name, features, size)#, face)
        
        db_users = [user["name"] for user in self.get_all_users()]
        for user in db_users: # delete deleted ones
            if user not in users_dict_characteristics.keys():
                self.remove_user(user)
            
    def load_as_dictionary(self):
        users_dict_characteristics = {}
        users_dict_size = {}
        #users_dict_faces = {}
        for user in self.get_all_users():
            name = user.get('name')
            features = json.loads(user.get('features', '{}'))
            size = json.loads(user.get('size', '{}'))
            #face = user.get('face', '')
            users_dict_characteristics[name] = features
            users_dict_size[name] = size
            #users_dict_faces[name] = face
        return users_dict_characteristics, users_dict_size#, users_dict_faces

    def _timestamp(self):
        return datetime.now().timestamp()