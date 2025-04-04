import time
import random
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from hri_msgs.msg import BodyOrientation, FaceGuiRequest, HeadOrientation
from hri_msgs.srv import Detection, Recognition, Training, AudioLocation, GetString, FaceGuiResponse
from .hri_bridge import HRIBridge

from .camera_parameters import RESOLUTION, get_parameters
from .api.gui import mark_face

class HRILogicV2Node(Node):

    def __init__(self, hri_logic):
        super().__init__("hri_logic_v2")

        self.br = HRIBridge()
        self.hri_logic = hri_logic

        self.last_frame_msg = None
        self.subscription_camera = self.create_subscription(Image, "camera/color/image_raw", self.frame_callback, 1)
        self.sub_detection_data = self.create_subscription(JointState, "/wxxms/joint_states", self.head_position_callback, 10)
        
        self.publisher_recognition = self.create_publisher(Image, "camera/color/recognition", 1)
        self.head_movement = self.create_publisher(HeadOrientation, "/head/movement", 1)
        self.input_tts = self.create_publisher(String, "input_tts", 1)
        self.publisher_face_name = self.create_publisher(FaceGuiRequest, "cara", 10)
        self.publisher_people = self.create_publisher(String, "robot/info/actual_people", 1)
        self.body = self.create_publisher(Twist, "/cmd_vel", 1)
        self.robot_orientation = self.create_publisher(BodyOrientation, "/robot/orientation", 1)
        self.gui_timeout_pub = self.create_publisher(String, "/gui/timeout", 1)

        self.ask_if_name_service = self.create_service(FaceGuiResponse, "peticion_yesno", self.ask_if_name)
        self.asking_service = self.create_service(FaceGuiResponse, "peticion_nombre", self.get_face_name)
    

        self.audio_location_client = self.create_client(AudioLocation, "audio_location")
        while not self.audio_location_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('AudioLocation service not available, waiting again...')
        
        self.detection_client = self.create_client(Detection, "detection")
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Detection service not available, waiting again...")

        self.recognition_client = self.create_client(Recognition, "recognition")
        while not self.recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Recognition service not available, waiting again...")

        self.training_client = self.create_client(Training, "recognition/training")
        while not self.training_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Training service not available, waiting again...")

        self.get_people_client = self.create_client(GetString, "recognition/get_people")
        while not self.get_people_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Get People service not available, waiting again...")
        
    def frame_callback(self, frame_msg):
        self.last_frame_msg = frame_msg

    def head_position_callback(self, msg):
        self.hri_logic.pan = msg.position[0]
        self.hri_logic.tilt = msg.position[1]
        
    def get_face_name(self, request, response):
        self.hri_logic.face_name = str(request.texto.data).split()[0]
        response.result = 1
        return response
    def ask_if_name(self, request, response):
        self.hri_logic.yes_no_result = str(request.texto.data)
        response.result = 1
        return response
    
class HRILogicV2:

    LOWER_BOUND = 0.75
    MIDDLE_BOUND = 0.80
    UPPER_BOUND = 0.90

    def __init__(self, ask_unknowns=True, draw_rectangle=True, show_distance=True, show_score=True):       
        self.ask_unknowns = ask_unknowns
        self.draw_rectangle = draw_rectangle
        self.show_distance = show_distance
        self.show_score = show_score

        self.last_voice = 0
        self.voice_thr = 1
        self.last_voice_time= time.time()
        self.pan = 0
        self.tilt = 0

        self.asking_name = False
        self.ask_if_name_mode = False

        self.ask_name_if_no = False

        self.yes_no_result = None
        self.face_name = None
        
        self.angle_threshold = 0.10
        self.max_time_without_interaction = 7
        
        self.last_interaction = 0
        self.interlocutor = None
        
        self.timer_ask_name = time.time()
        

        self.camera_matrix, self.camera_matrix_inv, self.dist_coeffs = get_parameters(RESOLUTION)
    
        self.actual_people = {}
        self.node = HRILogicV2Node(self)

    def read_text(self, text):
        """Reads text with speech to text

        Args:
            text (str): Text to be readed.
        """

        self.node.input_tts.publish(String(data=text))

    def spin(self):
                
        while rclpy.ok():
            start_time = time.time()  # Inicia el temporizador
            voice_intensity, human_voice_pan = self.process_audio_location(audio_threshold=self.voice_thr)

            data = [] # Lista de [id, box, score, face_msg, distance, features, pos]
            if self.node.last_frame_msg is not None:
                data = self.process_frame(self.node.last_frame_msg)
            
            if self.interlocutor is None: # Modo no-interaccion
                if voice_intensity != -1: # Si hay voz humana
                    self.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), pan=human_voice_pan, tilt=0.0))
                    self.node.get_logger().info(f"Moviendo a pan: {human_voice_pan:.3f}")

                    i = 0
                    while i < len(data) and self.interlocutor is None:
                        id, box, score, _, distance, _,_ = data[i]
                        face_pan, face_tilt = self.get_angles_from_box(box)

                        if abs(human_voice_pan - (self.pan + face_pan)) < self.angle_threshold:  # Si hay una cara en el ángulo de incidencia del sonido
                            self.interlocutor = id  # Tenemos nuevo interlocutor y nos movemos hacia él
                            self.last_interaction = time.time() # Guardamos su interaccion

                            if id.startswith("0x"): # Si es un ID
                                if score > 1 and self.ask_unknowns: # Si la imagen tiene buena calidad (asegurar que no esta confundiendo)
                                    self.read_text("Hola, ¿Cual es tu nombre?")
                                    self.asking_name = True # Preguntamos el nombre
                                    
                            else:
                                if distance >= self.LOWER_BOUND and distance < self.MIDDLE_BOUND: # Si no es un ID y reconoce regular
                                    if score > 1 and self.ask_unknowns: # Si la imagen tiene buena calidad (asegurar que no esta confundiendo)
                                        self.read_text("Um, no te reconozco con seguridad, ¿eres %s" % id)
                                        self.ask_if_name_mode = True
                                        self.timer_ask_name = time.time()
                            self.node.get_logger().info(f"Interactuando con {id}")
                            self.node.head_movement.publish(HeadOrientation(mode=String(data="relative"), pan=face_pan, tilt=face_tilt))
                            
                        i += 1

            else: # Modo interacción
                interlocutor_data = next(((id, box, score, face_aligned_msg, distance, features, pos) 
                                          for id, box, score, face_aligned_msg, distance, features, pos in data if id == self.interlocutor), None)

                if interlocutor_data: # Si el interlocutor está en el rango de visión (si no seria None)
                    id, box, score, face_aligned_msg, distance, features, _ = interlocutor_data
                    
                    if self.asking_name and  time.time() - self.timer_ask_name > 2: # Si estoy en el modo de preguntar el nombre
                        if self.face_name is None: # Si aun no hay respuesta, envio la cara del interlocutor
                            self.node.publisher_face_name.publish(FaceGuiRequest(mode=0, face= face_aligned_msg))
                        else: # Si hay respuesta
                            self.asking_name = False # Ya no estamos preguntando el nombre
   
                            result, message = self.training_request(String(data="rename_class"), String(data=json.dumps({
                                "old_name": id,
                                "new_name": self.face_name
                            }))) # Renombramos la clase (al nombre de la persona)
                            self.node.get_logger().info(message.data)
                            self.actual_people[self.face_name] = time.time() # Registramos su interaccion

                            if result == 0:
                                self.read_text("Perdona " + self.face_name + ", no te había reconocido bien")
                            elif result == 1:
                                self.read_text("Bienvenido " + self.face_name + ", no te conocía")

                            self.face_name = None

                    elif self.ask_if_name_mode: # Si estoy en el modo preguntar si es X
                        if self.yes_no_result is None: # Si aun no hay respuesta, envio la cara del interlocutor
                            self.node.publisher_face_name.publish(FaceGuiRequest(mode=1, face=face_aligned_msg, texto=String(data=id)))
                        else: # Si hay respuesta
                            self.ask_if_name_mode = False # Ya no estamos preguntando si es X

                            if self.yes_no_result == "Si":
                                result, message = self.training_request(String(data="add_features"), String(data=json.dumps({
                                    "class_name": id,
                                    "features": features,
                                }))) # Refinamos la clase
                                self.node.get_logger().info(message.data)
                                
                                self.read_text("Gracias " + id + ", me gusta confirmar que estoy reconociendo bien")
                            elif self.yes_no_result == "No":
                                self.read_text("Entonces, ¿Cual es tu nombre?")
                                self.ask_name_if_no = True
                                
                            self.yes_no_result = None

                    elif self.ask_name_if_no: # Si me ha dicho que no a "¿Eres tal?"
                        if self.face_name is None: # Si no hay respuesta
                            self.node.publisher_face_name.publish(FaceGuiRequest(mode=0, face=face_aligned_msg)) 
                        else: # Si hay respuesta
                            self.ask_name_if_no = False                  
          
                            if self.face_name is not None:
                                result, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                                    "class_name": self.face_name,
                                    "features": features
                                }))) # Refinamos la clase
                                self.node.get_logger().info(message.data)

                                distance = 1
                                if result == 1:
                                    self.read_text("Perdona " + self.face_name + ", no te había reconocido bien")
                                elif result == 0:
                                    self.read_text("Bienvenido " + self.face_name + ", no te conocía" )
         
                            self.face_name = None               
                   
                    face_pan, face_tilt = self.get_angles_from_box(box) # Nos movemos hacia él
                    
                    # Mueve solo tilt con la cabeza
                    #self.node.robot_orientation.publish(BodyOrientation(mode=String(data="absolute"), rot_z=float(self.pan + face_pan)))
                    #self.node.head_movement.publish(JointGroupCommand(name="turret", cmd=[0, self.tilt + face_tilt]))

                    # Mueve pan cabeza angulos pequeños
                    self.node.head_movement.publish(HeadOrientation(mode=String(data="relative"), pan=face_pan, tilt=face_tilt))

                    self.node.get_logger().info(f"Moviendo a la cara del interlocutor ({id})")

                    if abs(human_voice_pan - (self.pan + face_pan)) < self.angle_threshold and voice_intensity != -1: # Si hay voz donde está el interlocutor
                        self.last_interaction = time.time() # Reiniciamos el timeout por seguir interactuando
                        self.node.get_logger().info(f"El interlocutor ({id}) sigue interactuando ({human_voice_pan:.3f})")
                    else:
                        self.node.get_logger().info(f"Veo al interlocutor pero no está interactuando... (Está en {(self.pan + face_pan):.3f} y escucho voz en {human_voice_pan:.3f})")
                else:
                    self.node.get_logger().info("No veo al interlocutor en mi campo de visión")

                if time.time() - self.last_interaction > self.max_time_without_interaction: # Si hace timeout
                    self.node.gui_timeout_pub.publish(String(data="Interlocutor timeout"))

                    self.asking_name = False
                    self.ask_if_name_mode = False
                    self.ask_name_if_no = False
                
                    self.interlocutor = None # Dejamos de interactuar con él

                    self.node.get_logger().info("Dejando de interactuar por timeout. Voy a [0, 0]")
                    self.node.head_movement.publish(HeadOrientation(mode=String(data="absolute"), pan=0.0, tilt=0))

            end_time = time.time()  # Finaliza el temporizador
            
            elapsed_time = end_time - start_time  # Calcula el tiempo de ejecución
            
            print(f"Tiempo de ejecución de una iteración: {elapsed_time:.6f} segundos")
            rclpy.spin_once(self.node)
           

    def process_audio_location(self, audio_threshold, time_threshold =1):
        voice_intensity, human_voice_pan = self.audio_location_request()
        if abs(human_voice_pan - self.last_voice) > audio_threshold and (time.time() - self.last_voice_time) < time_threshold:
            human_voice_pan = self.last_voice
        else:
            self.last_voice = human_voice_pan
            self.last_voice_time = time.time()
        
        return voice_intensity, human_voice_pan
   
    def get_angles_from_box(self, box):
        (x, y, w, h) = box
        head_position = [x + (w / 2), y + (h / 2)]

        head_position_homogeneous = np.array([head_position[0], head_position[1], 1])
        head_position_camera = self.camera_matrix_inv @ head_position_homogeneous

        relative_pan_angle = -np.arctan(head_position_camera[0])
        relative_tilt_angle = np.arctan(head_position_camera[1])
        
        return relative_pan_angle, relative_tilt_angle

    def process_frame(self, frame_msg):
        data = []

        people_json_msg = self.get_people_request() # Obtiene las personas existentes
        people = json.loads(people_json_msg.data)

        deleted_classes = [person for person in self.actual_people.keys() if person not in people]
        for person in deleted_classes:
            self.actual_people.pop(person)

        frame = self.node.br.imgmsg_to_cv2(frame_msg, "bgr8")

        positions_msg, scores_msg = self.detection_request(frame_msg)  # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
                self.recognition_request(frame_msg, positions_msg[i]))  # Recogniion
            face_aligned, features, classified, distance, pos = (
                self.node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg))

            if distance < self.LOWER_BOUND: # No reconoce a la persona
                classified = None

                if scores[i] >= 1 and self.ask_unknowns:  # Si la imagen es buena, asignamos ID
                    while classified is None or classified in people: 
                        classified = "0x" + random.randbytes(3).hex() # Random ID

                    result, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                        "class_name": classified,
                        "features": features
                    }))) # Añadimos nueva clase
                    self.node.get_logger().info(message.data)
            elif distance < self.MIDDLE_BOUND: # Cree que es alguien pero no está seguro
                pass

            elif distance < self.UPPER_BOUND: # Sabe que es alguien pero lo detecta un poco raro
                if not classified.startswith("0x"): # No es random
                    self.actual_people[classified] = time.time()
                        
            else: # Reconoce bien a la persona
                if classified is not None and not classified.startswith("0x"): # No es random
                    if classified not in self.actual_people or (time.time() - self.actual_people[classified]) > 30:
                        self.read_text("Bienvenido de vuelta " + classified)

                    self.actual_people[classified] = time.time()

                result, message = self.training_request(String(data="refine_class"), String(data=json.dumps({
                    "class_name": classified,
                    "features": features,
                    "position": pos
                }))) # Refinamos la clase

            if classified is not None:
                data.append([classified, positions[i], scores[i], face_aligned_msg, distance, features, pos])

            inter_time = self.max_time_without_interaction - (time.time() - self.last_interaction)
            mark_face(frame, positions[i], distance, self.MIDDLE_BOUND, self.UPPER_BOUND, classified=classified, drawRectangle=self.draw_rectangle,
                score=scores[i], showDistance=self.show_distance, showScore=self.show_score, 
                interlocutor=self.interlocutor, inter_time=0 if inter_time < 0 else inter_time)
            
        actual_people_json = json.dumps(self.actual_people)
        self.node.publisher_people.publish(String(data=actual_people_json))
        self.node.publisher_recognition.publish(self.node.br.cv2_to_imgmsg(frame, "bgr8"))
        
        return data

    def audio_location_request(self):
        audio_location_request = AudioLocation.Request()

        future_audio_location = self.node.audio_location_client.call_async(audio_location_request)
        rclpy.spin_until_future_complete(self.node, future_audio_location)
        result_audio_location = future_audio_location.result()

        return result_audio_location.intensity, result_audio_location.angle_rad

    def detection_request(self, frame_msg):
        detection_request = Detection.Request()
        detection_request.frame = frame_msg

        future_detection = self.node.detection_client.call_async(detection_request)
        rclpy.spin_until_future_complete(self.node, future_detection)
        result_detection = future_detection.result()

        return result_detection.positions, result_detection.scores

    def recognition_request(self, frame_msg, position_msg):
        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg

        future_recognition = self.node.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self.node, future_recognition)
        result_recognition = future_recognition.result()

        return (
            result_recognition.face_aligned,
            result_recognition.features,
            result_recognition.classified,
            result_recognition.distance,
            result_recognition.pos,
        )

    def training_request(self, cmd_type_msg, args_msg):
        training_request = Training.Request()
        training_request.cmd_type = cmd_type_msg
        training_request.args = args_msg

        future_training = self.node.training_client.call_async(training_request)
        rclpy.spin_until_future_complete(self.node, future_training)
        result_training = future_training.result()

        return result_training.result, result_training.message

    def get_people_request(self):
        get_people_request = GetString.Request()

        future_get_people = self.node.get_people_client.call_async(get_people_request)
        rclpy.spin_until_future_complete(self.node, future_get_people)
        result_get_people = future_get_people.result()

        return result_get_people.text


def main(args=None):
    rclpy.init(args=args)

    hri_logic = HRILogicV2()
    hri_logic.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
