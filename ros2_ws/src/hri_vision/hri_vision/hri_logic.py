import time
import json
from queue import Queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from hri_msgs.srv import Detection, Recognition, Training, GetString

from .hri_bridge import HRIBridge
from .api.gui import get_name, ask_if_name, mark_face

# refactorizar esta vaina
# tema de que se congele si te pide el nombre o lo que sea
# revisar que a veces estas mirando pa otro lao y cierras la ventana y te pilla una foto antigua

# tema hacerlo mejor tipo quiza que todo el proceos se haga en otro lao y haya un sitio donde se vaya cogiendo
# la info que se publica de deteccion y reconocimiento y entonces solo coja siempre el ultimo frame asin fluido
# y con la ultima info de reconocimiento y deteccion se pinte. Entonces tendriamos la camara fluida y con esa info,
# no que ahora va lentita

# para la bd investigar y ver como hacer. Yo creo que el classifier deberia manejar todo el tema bd o incluso otro nodo para la bd
# con ros pero que luego hubiese un api rest cuyas peticiones usen el nodo de ros para acceder a esa bd tambien, o por lo contrario
# que lo haga externamente, pero que sea un diseño asin epico

# para lo de que se queda pillado si te esta preguntando el nombre, pues poner que en vez de quitarse el video ponga "sin señal" o se quede el ultimo frame y ya

# revisar tema prints y logs que es un caos
# en la interfaz web poner de nuevo lo de ver la gente conectada
# hacer algo para cuando borrar que esto lo detecte
# algo tipo un topic informativo de la bd y si publica que se ha borrado pues borra aqui tmb

# al publicar el actual people es un stringify del json, quiza en el r2wsubscribe o lo q sea meter un argumento de json parse q si es true sabes
# q lo q esta publicnado es json y lo parsee
# la idea es que llegue todo a la web en json y no dependiendo ed una cosa u otra lo jsonice o no, pensar esto bien

# en el recognizer y eso importar todo al principio o algo asi para que no de un lagazo en el primer reconocimiento

class HRILogicNode(Node):

    def __init__(self, hri_logic : "HRILogic"):
        """Initializes the logic node. Subscribes to camera and publishes recognition images"""

        super().__init__('hri_logic')

        self.hri_logic = hri_logic
        self.data_queue = Queue(maxsize = 1) # Queremos que olvide frames antiguos, siempre a por los mas nuevos

        self.get_actual_people_service = self.create_service(GetString, 'logic/get/actual_people', self.hri_logic.get_actual_people)

        self.subscription_camera = self.create_subscription(Image, 'camera/color/image_raw', self.frame_callback, 1)

        self.publisher_recognition = self.create_publisher(Image, 'camera/color/recognition', 1)
        self.publisher_people = self.create_publisher(String, 'logic/info/actual_people', 1)

        self.detection_client = self.create_client(Detection, 'detection')
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detection service not available, waiting again...')
        
        self.recognition_client = self.create_client(Recognition, 'recognition')
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Recognition service not available, waiting again...')

        self.training_client = self.create_client(Training, 'recognition/training')
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Training service not available, waiting again...')
        
        self.input_tts = self.create_publisher(String, 'input_tts', 10)
        self.br = HRIBridge()

        self.get_logger().info("HRI Logic Node initializated succesfully")

    def frame_callback(self, frame_msg):
        if self.data_queue.empty(): # We don't want blocking
            self.data_queue.put(frame_msg)


class HRILogic():

    LOWER_BOUND = 0.75
    MIDDLE_BOUND = 0.80
    UPPER_BOUND = 0.90

    def __init__(self, ask_unknowns = True, draw_rectangle = True, show_distance = True, show_score = True):
        """Initializes logic that uses the logic node. This class has all the logic related to how to train the recognizer,
        when to detect faces, when we will ask a new person for his name, which values we consider reliable etc...
        
        Args:
            ask_unknowns (bool): If true will ask for new people. If false will only recognize already known people.
            draw_rectangle (bool): If true will draw a rectangle around the detected face on the frame published on recognition topic.
            show_distance (bool): If true will draw recognition score below the rectangle.
            show_score (bool):  If true will draw detector score below the distance.
        """

        self.ask_unknowns = ask_unknowns
        self.draw_rectangle = draw_rectangle
        self.show_distance = show_distance
        self.show_score = show_score

        self.actual_people = {}
        self.node = HRILogicNode(self)
    
    def spin(self):
        """Spins the logic node searching for new frames. If one is detected, process the frame."""

        while rclpy.ok():
            if not self.node.data_queue.empty():
                frame_msg = self.node.data_queue.get()
                self.process_frame(frame_msg)
                
            rclpy.spin_once(self.node)

    def process_frame(self, frame_msg):
        """Performs all the logic. Process the frame detecting and recognizing faces on the frame.
        
        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format
        """

        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")
        
        # people_json_msg = self.get_people_request() # Obtiene las personas existentes
        # people = json.loads(people_json_msg.data)

        # deleted_classes = [person for person in self.actual_people.keys() if person not in people]
        # for person in deleted_classes:
        #    self.actual_people.pop(person)

        positions_msg, scores_msg = self.detection_request(frame_msg)                   # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = \
                self.recognition_request(frame_msg, positions_msg[i])                   # Recognition
            face_aligned, features, classified, distance, pos = \
                self.node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg)
     
            if distance < self.LOWER_BOUND: # No sabe quien es (en teoria nunca lo ha visto), pregunta por el nombre
                classified = None
                if scores[i] >= 1 and self.ask_unknowns: # Si la imagen es buena, pregunta por el nombre, para que no coja una imagen mala
                    self.read_text("¿Cual es tu nombre?")
                    classified = get_name(face_aligned) # Poner aqui una lista o si no que meta un nuevo a mano, pero por si ya es que seleccione
                    if classified is not None:
                        self.actual_people[classified] = time.time()

                        already_known, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                            "class_name": classified,
                            "features": features
                        }))) # Añadimos clase (en teoria es alguien nuevo)
                        self.node.get_logger().info(message.data)

                        distance = 1

                        if already_known == 1:
                            self.read_text("Perdona " + classified + ", no te había reconocido bien")
                        elif already_known == 0:
                            self.read_text("Bienvenido " + classified + ", no te conocía")
                        else:
                            self.node.get_logger().info(">> ERROR: Algo salio mal al agregar una nueva clase")

            elif distance < self.MIDDLE_BOUND: # Cree que es alguien, pide confirmacion
                if scores[i] > 1 and self.ask_unknowns: # Pero solo si la foto es buena
                    self.read_text("Creo que eres " + classified + ", ¿es cierto?")
                    answer = ask_if_name(face_aligned, classified)
                    if answer: # Si dice que si es esa persona
                        self.actual_people[classified] = time.time()

                        output, message = self.training_request(String(data="add_features"), String(data=json.dumps({
                            "class_name": classified,
                            "features": features,
                        }))) # Añadimos otro vector distinto de features a la clase
                        self.node.get_logger().info(message.data)

                        if output >= 0:
                            self.read_text("Gracias " + classified + ", me gusta confirmar que estoy reconociendo bien")
                        else:
                            self.node.get_logger().info(">> ERROR: Algo salio mal al agregar features a una clase")
                    else: # Si dice que no, le pregunta el nombre
                        self.read_text("Entonces, ¿Cual es tu nombre?")
                        classified = get_name(face_aligned)
                        if classified is not None:
                            self.actual_people[classified] = time.time()

                            already_known, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                                "class_name": classified,
                                "features": features
                            }))) # Añadimos clase (en teoria es alguien nuevo)
                            self.node.get_logger().info(message.data)
                            
                            distance = 1

                            if already_known == 1:
                                self.read_text(classified + ", no me marees, por favor.")
                            elif already_known == 0:
                                self.read_text("Encantando de conocerte " + classified + ", perdona por confundirte")
                            else:
                                self.node.get_logger().info(">> ERROR: Algo salio mal al agregar una nueva clase")

            elif distance < self.UPPER_BOUND: # Sabe que es alguien pero lo detecta un poco raro
                self.actual_people[classified] = time.time()
                #classifier.addFeatures(classified, features)
            else: # Reconoce perfectamente
                if classified not in self.actual_people or (time.time() - self.actual_people[classified]) > 30:
                    self.read_text("Bienvenido de vuelta " + classified)

                self.actual_people[classified] = time.time()
                
                output, message = self.training_request(String(data="refine_class"), String(data=json.dumps({
                    "class_name": classified,
                    "features": features,
                    "position": pos
                }))) # Refinamos la clase
                self.node.get_logger().info(message.data)

                if output < 0:
                    self.node.get_logger().info(">> ERROR: Al refinar una clase")

            mark_face(frame, positions[i], distance, self.MIDDLE_BOUND, self.UPPER_BOUND, classified=classified, 
                      drawRectangle=self.draw_rectangle, score=scores[i], showDistance=self.show_distance, showScore=self.show_score)

        actual_people_time = self.get_actual_people_time()
        actual_people_json = json.dumps(actual_people_time)
        self.node.publisher_people.publish(String(data=actual_people_json))
        self.node.publisher_recognition.publish(self.node.br.cv2_to_imgmsg(frame, "bgr8"))

    def get_actual_people_time(self):
        actual_people_time = {}
        for key, value in self.actual_people.items():
            actual_people_time[key] = time.time() - value
        return actual_people_time

    def read_text(self, text):
        """Reads text with speech to text
        
        Args:
            text (str): Text to be readed.
        """
        
        print(f"[SANCHO] {text}")
        self.node.input_tts.publish(String(data=text))

    # Clients
    def detection_request(self, frame_msg):
        """Makes a detection request to the detection service.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format.
        
        Returns:
            positions (int[4][]): Array of 4 ints that determine the square surronding each face.
            scores (int[]): Array of scores of the detection in the same order as the positions.
        """

        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.node.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg):
        """Makes a recognition request to the recognition service.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format.
            position_msg (int[4]): 4 ints determining the square surronding the face.
        
        Returns:
            face_aligned (Image-ROS2): The face aligned horizontally.
            features (float[]): Features vector of the face.
            classified (String): Class of the recognized face.
            distance (float): Recognition score.
            pos (int): Position of the vector with the best distance.
        """

        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg

        future_recognition = self.node.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self.node, future_recognition)
        result_recognition = future_recognition.result()

        return (result_recognition.face_aligned, result_recognition.features,
                result_recognition.classified, result_recognition.distance, result_recognition.pos)

    def training_request(self, cmd_type_msg, args_msg):
        """Makes a training request to the training service.
        
        Args:
            cmd_type_msg (String): Training type (str) in ROS2 format (String).
            classified_msg (String): Class of the face.
            features_msg (float[]): Feature vector of the face.
            pos (int): Position of the vector with best distance (used to train).
        
        Returns:
            response (Training.srv): Result. -1 means something went wrong. 0 means everything is okay
                and in case of cmd_type = add_class, also means that the class wasn't already known. 1 means 
                that the class was already known, and means the same as 0 for cmd_type != add_class. Message...
        """

        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message

    # Services
    def get_actual_people(self, request, response):
        actual_people_time = self.get_actual_people_time()
        actual_people_json = json.dumps(actual_people_time)
        response.text = String(data=actual_people_json)
        return response

def main(args=None):
    rclpy.init(args=args)

    hri_logic = HRILogic()

    hri_logic.spin()
    rclpy.shutdown()
