import os
import json
from queue import Queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from hri_msgs.srv import Detection, Recognition, Training, GetString
from hri_msgs.msg import Log
from rumi_msgs.msg import SessionMessage

from sancho_web.database.system_database import CONSTANTS
from .database.people_manager import PeopleManager

from .hri_bridge import HRIBridge
from .api.gui import get_name, ask_if_name, mark_face


class HRILogicNode(Node):

    def __init__(self, hri_logic : "HRILogic"):
        """Initializes the logic node. Subscribes to camera and publishes recognition images"""

        super().__init__('hri_logic')

        self.hri_logic = hri_logic
        self.data_queue = Queue(maxsize = 1) # Queremos que olvide frames antiguos, siempre a por los mas nuevos
        
        self.get_actual_people_service = self.create_service(GetString, 'logic/get/actual_people', self.hri_logic.get_actual_people_service)
        self.get_last_frame_service = self.create_service(GetString, 'logic/get/last_frame', self.hri_logic.get_last_frame_service)

        self.subscription_camera = self.create_subscription(Image, 'camera/color/image_raw', self.frame_callback, 1)

        self.publisher_log = self.create_publisher(Log, 'logs/add', 10)
        self.publisher_session = self.create_publisher(SessionMessage, 'rumi/sessions/process', 10)
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

        self.last_frame = None

        self.node = HRILogicNode(self)
        self.people = PeopleManager(self.node)
    
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
        self.last_frame = frame

        positions_msg, scores_msg = self.detection_request(frame_msg)                   # Detection
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)
        for i in range(len(positions)):
            face_aligned_msg, features_msg, classified_id, classified_name_msg, distance_msg, pos_msg, face_updated = \
                self.recognition_request(frame_msg, positions_msg[i], scores_msg[i])        # Recognition
            face_aligned, features, classified_name, distance, pos = \
                self.node.br.msg_to_recognizer(face_aligned_msg, features_msg, classified_name_msg, distance_msg, pos_msg)
 
            if face_updated:
                self.create_log(CONSTANTS.ACTION.UPDATE_FACE, classified_id)

            if distance < self.LOWER_BOUND: # No sabe quien es (en teoria nunca lo ha visto), pregunta por el nombre
                classified_name = None
                classified_id = None
                if scores[i] >= 1 and self.ask_unknowns: # Si la imagen es buena, pregunta por el nombre, para que no coja una imagen mala
                    self.read_text("¿Cual es tu nombre?")
                    classified_name = get_name(face_aligned) # Poner aqui una lista o si no que meta un nuevo a mano, pero por si ya es que seleccione
                    if classified_name is not None:
                        face_aligned_base64 = self.node.br.cv2_to_base64(face_aligned)
                        result, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                            "class_name": classified_name,
                            "features": features,
                            "face": face_aligned_base64,
                            "score": scores[i]
                        }))) # Añadimos clase (en teoria es alguien nuevo)

                        if result >= 0:
                            distance = 1
                            classified_id = message

                            self.node.get_logger().info(f"Nueva clase con id {classified_id}")
                            self.people.process_detection(classified_id, scores[i], distance)

                            self.read_text("Bienvenido " + classified_name + ", no te conocía")
                            self.create_log(CONSTANTS.ACTION.ADD_CLASS, classified_id)
                        else:
                            self.node.get_logger().info(f">> ERROR: Algo salio mal al agregar una nueva clase: {message}")

            elif distance < self.MIDDLE_BOUND: # Cree que es alguien, pide confirmacion
                if scores[i] > 1 and self.ask_unknowns: # Pero solo si la foto es buena
                    self.read_text("Creo que eres " + classified_name + ", ¿es cierto?")
                    answer = ask_if_name(face_aligned, f"{classified_name} (ID {classified_id})")
                    if answer: # Si dice que si es esa persona
                        self.people.process_detection(classified_id, scores[i], distance)

                        output, message = self.training_request(String(data="add_features"), String(data=json.dumps({
                            "class_id": classified_id,
                            "features": features,
                        }))) # Añadimos otro vector distinto de features a la clase
                        self.node.get_logger().info(message)

                        if output >= 0:
                            self.read_text("Gracias " + classified_name + ", me gusta confirmar que estoy reconociendo bien")
                            self.create_log(CONSTANTS.ACTION.ADD_FEATURES, classified_id)
                        else:
                            self.node.get_logger().info(f">> ERROR: Algo salio mal al agregar features a una clase")
                    else: # Si dice que no, le pregunta el nombre
                        self.read_text("Entonces, ¿Cual es tu nombre?")
                        classified_name = get_name(face_aligned)
                        if classified_name is not None:
                            face_aligned_base64 = self.node.br.cv2_to_base64(face_aligned)
                            result, message = self.training_request(String(data="add_class"), String(data=json.dumps({
                                "class_name": classified_name,
                                "features": features,
                                "face": face_aligned_base64,
                                "score": scores[i]
                            }))) # Añadimos clase (en teoria es alguien nuevo)

                            if result >= 0:                            
                                distance = 1
                                classified_id = message

                                self.node.get_logger().info(f"Nueva clase con id {classified_id}")
                                self.people.process_detection(classified_id, scores[i], distance)

                                self.read_text("Encantando de conocerte " + classified_name + ", perdona por confundirte")
                                self.create_log(CONSTANTS.ACTION.ADD_CLASS, classified_id)
                            else:
                                self.node.get_logger().info(f">> ERROR: Algo salio mal al agregar una nueva clase: {message}")

            elif distance < self.UPPER_BOUND: # Sabe que es alguien pero lo detecta un poco raro
                self.people.process_detection(classified_id, scores[i], distance)
            else: # Reconoce perfectamente
                if self.people.get_last_seen(classified_id) > 60:
                    self.read_text("Bienvenido de vuelta " + classified_name)

                self.people.process_detection(classified_id, scores[i], distance)

                output, message = self.training_request(String(data="refine_class"), String(data=json.dumps({
                    "class_id": classified_id,
                    "features": features,
                    "position": pos
                }))) # Refinamos la clase

                if output < 0:
                    self.node.get_logger().info(F">> ERROR: Al refinar una clase: {message}")

            mark_face(frame, positions[i], distance, self.MIDDLE_BOUND, self.UPPER_BOUND, classified=classified_name, 
                      drawRectangle=self.draw_rectangle, score=scores[i], showDistance=self.show_distance, showScore=self.show_score)

        actual_people_time = self.people.get_all_last_seen()
        actual_people_json = json.dumps(actual_people_time)
        self.node.publisher_people.publish(String(data=actual_people_json))
        self.node.publisher_recognition.publish(self.node.br.cv2_to_imgmsg(frame, "bgr8"))

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

    def recognition_request(self, frame_msg, position_msg, score_msg):
        """Makes a recognition request to the recognition service.

        Args:
            frame_msg (Image-ROS2): The frame in ROS2 format.
            position_msg (int[4]): 4 ints determining the square surronding the face.
            score_msg (float): Score of the detection
        
        Returns:
            face_aligned (Image-ROS2): The face aligned horizontally.
            features (float[]): Features vector of the face.
            classified_id (int): Class id.
            classified_name (str): Class of the recognized face.
            distance (float): Recognition score.
            pos (int): Position of the vector with the best distance.
        """

        recognition_request = Recognition.Request()
        recognition_request.frame = frame_msg
        recognition_request.position = position_msg
        recognition_request.score = score_msg

        future_recognition = self.node.recognition_client.call_async(recognition_request)
        rclpy.spin_until_future_complete(self.node, future_recognition)
        result_recognition = future_recognition.result()

        return (result_recognition.face_aligned, result_recognition.features, result_recognition.classified_id,
                result_recognition.classified_name, result_recognition.distance, result_recognition.pos, 
                result_recognition.face_updated)    

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

        return result_training.result, result_training.message.data

    # Services
    def get_actual_people_service(self, request, response):
        actual_people_time = self.people.get_all_last_seen()
        actual_people_json = json.dumps(actual_people_time)
        response.text = actual_people_json

        return response

    def get_last_frame_service(self, request, response):
        response.text = self.node.br.cv2_to_base64(self.last_frame, quality=100)

        return response

    # Utils
    def read_text(self, text):
        self.node.get_logger().info(f"[SANCHO] {text}")
        self.node.input_tts.publish(String(data=text))
    
    def create_log(self, action, faceprint_id):
        self.node.publisher_log.publish(Log(
            level=CONSTANTS.LEVEL.INFO,
            origin=CONSTANTS.ORIGIN.ROS,
            actor="logic_node",
            action=action,
            target=faceprint_id,
            message="",
            metadata_json=""
        ))

def main(args=None):
    rclpy.init(args=args)

    hri_logic = HRILogic()

    hri_logic.spin()
    rclpy.shutdown()
