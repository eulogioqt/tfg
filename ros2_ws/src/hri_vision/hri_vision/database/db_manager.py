from datetime import datetime
from tinydb import TinyDB, Query

# creo q lo mejor va a ser usar directamente esto en tiempo real no tener una copia y luego volcarla,
# porque al final es un json nama ya de por si es rapido
# estudarlo pero vaya si es esta opcion pues super mega isiii

# en el recognizer quitar lo de los mensajes y poner codigos qu es como realmente se hace zabe
class FaceprintsDatabase:
    def __init__(self, db_path='faceprints_db.json'):
        self.db = TinyDB(db_path)
        self.User = Query()

    def add_user(self, name, features, size):#, face):
        if self.db.search(self.User.name == name):
            return False  # Ya existe
        self.db.insert({
            'name': name,
            'features': features,
            'size': size,
            'learning_date': self._timestamp(),
            #'face': face
        })
        return True

    def update_user(self, name, new_features=None, new_size=None):#, new_face):
        update_data = {}
        if new_features:
            update_data['features'] = new_features
        if new_size:
            update_data['size'] = new_size
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
            features = user.get('features', '{}')
            size = user.get('size', '{}')
            #face = user.get('face', '')
            users_dict_characteristics[name] = features
            users_dict_size[name] = size
            #users_dict_faces[name] = face
        return users_dict_characteristics, users_dict_size#, users_dict_faces

    def _timestamp(self):
        return datetime.now().timestamp()