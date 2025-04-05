import os
import json

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

        self.people = {}
        self.size = {}

        self.use_database = use_database
        if self.use_database:
            self.db_path = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), "database/faceprints_db.json"))
            self.database = FaceprintsDatabase(self.db_path)
            self.load()

    def save(self):
        '''Saves the learned data to a file.'''
        
        if self.use_database:
            self.database.save_from_dictionary(self.people, self.size)
            print("Data saved to database")

    def load(self):
        '''Loads the learned data from a file.'''
        
        if self.use_database:
            self.people, self.size = self.database.load_as_dictionary()
        
        self.print_people()

    def get_people(self):
        '''Get all people names.
        
        Returns:
            str: JSON Array with all people names.
        '''
        return json.dumps(list(self.people.keys()))

    def print_people(self):
        '''Prints the names of the existing people.'''

        print("Personas que conozco:")
        for key in self.people.keys():
            print(key)

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

        if not self.people:
            return closest_class, closest_distance, position

        for class_name, feature_list in self.people.items():
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

        new_size = self.size[class_name][position] + 1
        self.size[class_name][position] = new_size

        self.people[class_name][position] = [(x * (new_size - 1)+ y) / new_size for x, y in 
                                             zip(self.people[class_name][position], features)]

        result = 1
        message = "La clase " + class_name + " ha sido refinada"

        return result, message

    def add_features(self, class_name, features):
        '''Adds a new feature vector to the array of vectors that describes a class.
        
        Args:
            class_name (str): The class.
            features (Array: float): The new feature vector.
        '''

        self.people[class_name].append(features)
        self.size[class_name].append(1)

        result = 1
        message = ("La clase " + class_name + " ahora tiene " + 
            str(len(self.people[class_name])) + " vectores de características independientes.")

        return result, message

    def add_class(self, class_name, features):
        '''Adds a new class with a unique feature vector.

        Args:
            class_name (str): The class.
            features (Array: float): The feature vector.
        '''

        already_known = class_name in self.people

        if already_known:
            _, message = self.add_features(class_name, features)
        else:
            self.people[class_name] = [features]
            self.size[class_name] = [1]
            message = "Nueva clase aprendida: " + class_name

        result = int(already_known)

        return result, message

    def rename_class(self, old_name, new_name):
        '''Renames a class.

        Args:
            old_name (str): The actual class.
            new_name (str): The new class name.
        '''

        if old_name not in self.people or old_name not in self.size:
            result = 0 
            message = "La clase " + old_name + " no existe."
        elif new_name in self.people or new_name in self.size:
            result = 0 
            message = "La clase " + new_name + " ya existe."
        else:
            self.people[new_name] = self.people.pop(old_name)
            self.size[new_name] = self.size.pop(old_name)
        
            result = 1
            message = "La clase " + old_name + " ha sido renombrada a " + new_name

        return result, message

    def delete_class(self, class_name):
        '''Removes a class.
        
        Args:
            class_name (str): The class.
        '''
        
        if class_name in self.people:
            self.people.pop(class_name)
            self.size.pop(class_name)

            result = 1
            message = "La clase " + class_name + " ha sido eliminada correctamente"
        
        else:
            result = -1
            message = "La clase " + class_name + " no existe"

        return result, message