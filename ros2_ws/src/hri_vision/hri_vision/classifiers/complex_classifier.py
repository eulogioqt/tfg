import os
import cv2
import json
import base64

from ..api.utils import normalized_cosine_similarity_distance
from ..database.db_manager import FaceprintsDatabase

# make_complex_classifier an Object that extends from a classifier interface or abstract object
# Nota: el size es necesario porque lelva la cuenta de cuantos refinamientos tiene cada vector especifico

# Hacer un sistema de mensajes con json para enviar parametros y para devolver informacion de la ejecucion de los comandos tanto para el panel de control
# como para python

# para enviar cmd y args en json o algo asi

# este objeto va a tener la base de datos, va a ser el dueño, y es lo mejor ya que asi
# la api rest consulta este objeto que va a tener en sus diccionarios en todo momento los datos actualizados
# y no pueden ocurrir inconsistencias, si la bd quiere eliminar a alguien, entonces
# llegara aqui y el complex lo borra de su diccionario y ya al rato lo volcara en la bd
# no que si la bd la tiene la api rest el nodo este se jode y al final todo es solo para este nodo
# hacer servicios para todo, get all, get, put (para rename), delete (que ya lo hay)

# refactorizar esto para hacerlo bien en condiciones:
# tener un diccionario donde cada clave sea el name y el valor el objeto tal cual de la bd, y asi es easy de couyons
# para hacer el load el save y todo
# los modelos serian compartidos con la api rest

class ComplexClassifier:

    def __init__(self, use_database = False):
        '''Inits classifier

        Args:
            use_database (str): If True, uses and stores new data into database.
        '''

        self.use_database = use_database
        self.db_path = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), "database/faceprints_db.json"))
        self.db = FaceprintsDatabase(self.db_path)

        if self.use_database:
            self.load()
            self.print_people()

    def classify_face(self, new_features):
        '''Given a feature vector, gives the closest class.
        
        Args:
            new_features (Array: float): Feature vector.
        
        Returns:
            closest_class (str): The name of the closest class.
            closes_distance (float): The normalized cosine distance to the closest_class.
            position (int): The position of the vector in the array of vectors of the class that
                was the closest to the given vector.
        '''

        closest_class = None
        closest_distance = 0
        position = 0
        for faceprint in self.db.get_all():
            class_name = faceprint["name"]
            feature_list = faceprint["features"]

            for i in range(0, len(feature_list)):
                distance = normalized_cosine_similarity_distance(new_features, feature_list[i])

                if distance > closest_distance:
                    closest_distance = distance
                    closest_class = class_name
                    position = i

        return closest_class, closest_distance, position

    def refine_class(self, class_name, features, position):
        '''Makes a certain feature vector more precise by averaging with a new feature vector.

        Args:
            class_name (str): The class.
            features (Array: float): The new feature vector.
            position (int): The position of the known feature vector we want to make more precise.
        '''

        faceprint = self.db.get_by_name(class_name)
        
        new_size = faceprint["size"][position] + 1
        faceprint["size"][position] = new_size

        faceprint["features"][position] = [(x * (new_size - 1) + y) / new_size for x, y in 
                                             zip(faceprint["features"][position], features)]
        
        self.db.update(class_name, faceprint)

        result = 1
        message = "La clase " + class_name + " ha sido refinada"

        return result, message

    def add_features(self, class_name, features):
        '''Adds a new feature vector to the array of vectors that describes a class.
        
        Args:
            class_name (str): The class.
            features (Array: float): The new feature vector.
        '''
        
        faceprint = self.db.get_by_name(class_name)

        faceprint["features"].append(features)
        faceprint["size"].append(1)

        self.db.update(class_name, faceprint)
        
        result = 1
        message = ("La clase " + class_name + " ahora tiene " + 
            str(len(faceprint["features"])) + " vectores de características independientes.")

        self.save()
        return result, message

    def add_class(self, class_name, features):
        '''Adds a new class with a unique feature vector.

        Args:
            class_name (str): The class.
            features (Array: float): The feature vector.
        '''

        already_known = self.db.add(class_name, features) is None

        if already_known:
            _, message = self.add_features(class_name, features)
        else:
            message = "Nueva clase aprendida: " + class_name

        result = int(already_known)

        self.save()
        return result, message

    def rename_class(self, class_name, new_name):
        '''Renames a class.

        Args:
            class_name (str): The actual class.
            new_name (str): The new class name.
        '''

        all_names = self.db.get_all_names()
        if class_name == new_name:
            result = 1
            message = "Has intentado renombrar con el mismo nombre, no se ha cambiado nada"
        elif class_name not in all_names:
            result = 0 
            message = "La clase " + class_name + " no existe."
        elif new_name in all_names:
            result = 0 
            message = "La clase " + new_name + " ya existe."
        else:
            self.db.update(class_name, { "name": new_name })
        
            result = 1
            message = "La clase " + class_name + " ha sido renombrada a " + new_name

        self.save()
        return result, message

    def delete_class(self, class_name):
        '''Removes a class.
        
        Args:
            class_name (str): The class.
        '''
        
        if class_name in self.db.get_all_names():
            self.db.remove(class_name)

            result = 1
            message = "La clase " + class_name + " ha sido eliminada correctamente"
        else:
            result = -1
            message = "La clase " + class_name + " no existe"

        self.save()
        return result, message
    
    def save_face(self, class_name, face, face_score):
        '''Saves face image as a representation of the class

        Args:
            class_name (str): The class.
            face (cv2-Image): Cropped face image.
            face_score (float): Score of the detection.
        '''

        faceprint = self.db.get_by_name(class_name)
        is_best_face = faceprint and face_score > faceprint.get("face_score", 0)
        
        if is_best_face:
            target_size = (128, 128)
            resized = face#cv2.resize(face, target_size, interpolation=cv2.INTER_AREA)

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
            _, jpeg = cv2.imencode('.jpg', resized, encode_param)

            face_base64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
            self.db.update(class_name, { 
                "face": face_base64, 
                "face_score": face_score 
            })
        
        return is_best_face

    def save(self):
        '''Saves the learned data to a file.'''
        
        if self.use_database:
            self.db.save()

    def load(self):
        '''Loads the learned data from a file.'''
        
        if self.use_database:
            self.db.load()

    def get_people(self):
        '''Get all people names.
        
        Returns:
            str: JSON Array with all people names.
        '''

        return json.dumps(self.db.get_all_names())
    
    def print_people(self):
        '''Prints all known people'''

        print("Known people:")
        for name in self.db.get_all_names():
            print(f"- {name}")