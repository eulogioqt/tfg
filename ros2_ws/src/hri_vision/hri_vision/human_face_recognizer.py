import json
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from hri_msgs.srv import Recognition, Training, GetString
from .hri_bridge import HRIBridge
from .aligners.aligner_dlib import align_face
from .encoders.encoder_facenet import encode_face
from .classifiers.complex_classifier import ComplexClassifier

class HumanFaceRecognizer(Node):

    def __init__(self, use_database = True):
        """Initializes the recognizer node.

        Args:
            use_database (str): If True will use database to load and store data.
        """

        super().__init__("human_face_recognizer")

        show_metrics_param = self.declare_parameter("show_metrics", False)
        self.show_metrics = show_metrics_param.get_parameter_value().bool_value
        self.get_logger().info(f"Show Metrics: {self.show_metrics}")

        self.recognition_service = self.create_service(Recognition, "recognition", self.recognition)
        self.training_service = self.create_service(Training, "recognition/training", self.training)
        self.get_faceprint_service = self.create_service(GetString, "recognition/get_faceprint", self.get_people)
        self.faces_publisher = self.create_publisher(Image, "camera/color/aligned_faces", 10)

        self.classifier = ComplexClassifier(use_database)
        self.save_db_timer = self.create_timer(10.0, self.save_data)
        self.get_logger().info(f"Using database: {use_database}")

        self.training_dispatcher = {
            "refine_class": self.classifier.refine_class,
            "add_features": self.classifier.add_features,
            "add_class": self.classifier.add_class, # y este igual realmente, pensarlo bien porque en vd si es train "nueva clase"

            "rename_class": self.classifier.rename_class, # cambiar estos
            "delete_class": self.classifier.delete_class # porque no son training, simplemente por significado
        }

        self.br = HRIBridge()

    def recognition(self, request, response):
        """Recognition service

        Args:
            request (Recognition.srv): Frame and a face position

        Returns:
            response (Recognition.srv): Face aligned, features, class predicted, distance (score) and p
                osition of the vector in the class with highest distance (score).
        """

        start_recognition = time.time()

        frame = self.br.imgmsg_to_cv2(request.frame, "bgr8")
        position = [
            request.position.x,
            request.position.y,
            request.position.w,
            request.position.h,
        ]
        score = request.score
        size = math.sqrt(position[2]**2 + position[3]**2)
        
        face_aligned = align_face(frame, position)
        
        features = encode_face(face_aligned)
        classified, distance, pos = self.classifier.classify_face(features)
        if score >= 1 and distance >= 0.9: # Si la cara es buena y estamos seguro de que es esa persona
            self.classifier.save_face(classified, face_aligned, score) # lo bueno de asi es que siempre tiene una cara reciente
            self.get_logger().info(f"{classified} FACE SIZE: {size}") # hacer que el score guardado sea size / 256 por el score  real o algo o poner un minimo
            self.get_logger().info(f"{classified} FACE SIZE: {size}")
            self.get_logger().info(f"{classified} FACE SIZE: {size}")
            self.get_logger().info(f"{classified} FACE SIZE: {size}")
            self.get_logger().info(f"{classified} FACE SIZE: {size}")
            self.get_logger().info(f"{classified} FACE SIZE: {size}")
            self.get_logger().info(f"{classified} FACE SIZE: {size}")
            self.get_logger().info(f"{classified} FACE SIZE: {size}")

        face_aligned_msg, features_msg, classified_msg, distance_msg, pos_msg = (
            self.br.recognizer_to_msg(face_aligned, features, classified, distance, pos)
        )

        response.face_aligned = face_aligned_msg
        response.features = features_msg
        response.classified = classified_msg
        response.distance = distance_msg
        response.pos = pos_msg

        self.faces_publisher.publish(face_aligned_msg)

        recognition_time = time.time() - start_recognition
        response.recognition_time = recognition_time
        if self.show_metrics:
            self.get_logger().info("Recognition time: " + str(recognition_time))

        return response

    def training(self, request, response):
        """Training service. Takes the command type (training type) and uses the arguments needed to
        perform that kind of training.

        Args:
            request (Training.srv): Command type, and arguments in JSON Object format.

        Returns:
            response (Training.srv): Result. -1 means something went wrong. 0 means everything is okay
                and in case of cmd_type = add_class, also means that the class wasn't already known.
                1 means that the class was already known, and means the same as 0 for cmd_type != add_class.
        """

        cmd_type = request.cmd_type.data
        args = json.loads(request.args.data)

        try:
            function = self.training_dispatcher[cmd_type]
            result, message = function(**args)
        except Exception as e:
            result, message = -1, "Error executing " + cmd_type + ": " + str(e)
            pass

        response.result = result
        response.message = String(data=message)

        return response

    def get_people(self, request, response):
        args = request.args

        if args:
            args = json.loads(args)
            name = args["name"]

            faceprint = self.classifier.db.get_by_name(name)
            response.text = json.dumps(faceprint)
        else:
            faceprints = self.classifier.db.get_all()
            response.text = json.dumps(faceprints)

        return response

    def save_data(self):
        self.classifier.save()

def main(args=None):
    rclpy.init(args=args)

    human_face_recognizer = HumanFaceRecognizer()

    rclpy.spin(human_face_recognizer)
    rclpy.shutdown()
