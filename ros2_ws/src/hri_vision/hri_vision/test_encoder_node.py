import os
import cv2
import json
import random
import numpy as np
from queue import Queue
from scipy.spatial.distance import cdist

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from hri_msgs.srv import Detection, Recognition, Training, GetString

from .api._old_gui import mark_face
from .hri_bridge import HRIBridge


class TestEncoderNode(Node):
    def __init__(self, test_encoder: "TestEncoder"):
        super().__init__('test_encoder_node')
        self.test_encoder = test_encoder
        self.data_queue = Queue(maxsize=1)

        self.publisher_recognition = self.create_publisher(Image, 'camera/color/recognition', 1)

        self.detection_client = self.create_client(Detection, 'detection')
        self.recognition_client = self.create_client(Recognition, 'recognition')
        self.training_client = self.create_client(Training, 'recognition/training')
        self.get_faceprints_client = self.create_client(GetString, 'recognition/get_faceprint')

        for client, name in [
            (self.detection_client, 'Detection'),
            (self.recognition_client, 'Recognition'),
            (self.training_client, 'Training'),
            (self.get_faceprints_client, 'GetFaceprint')
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting again...')

        self.br = HRIBridge()
        self.get_logger().info("Test Encoder Node initialized successfully")


class TestEncoder:
    LOWER_BOUND = 0.75
    MIDDLE_BOUND = 0.80
    UPPER_BOUND = 0.90

    def __init__(self):
        self.node = TestEncoderNode(self)
        self.dataset_path = "/home/ubuntu/tfg/sandbox/mapir_dataset"
        self.samples = self.load_dataset()
        self.current_index = 0

    def load_dataset(self):
        samples = []
        for class_name in sorted(os.listdir(self.dataset_path)):
            class_dir = os.path.join(self.dataset_path, class_name)
            if not os.path.isdir(class_dir):
                continue
            for fname in os.listdir(class_dir):
                if fname.endswith((".png", ".jpg", ".jpeg")):
                    path = os.path.join(class_dir, fname)
                    samples.append((path, class_name))
        random.shuffle(samples)
        return samples

    def spin(self):
        total = 0
        correct = 0
        class_wise = {}

        while rclpy.ok() and self.current_index < len(self.samples):
            path, label = self.samples[self.current_index]
            img = cv2.imread(path)
            self.current_index += 1

            if img is None:
                self.node.get_logger().warn(f"No se pudo cargar la imagen: {path}")
                continue

            predicted = self.process_frame(img, label)

            total += 1
            if predicted == label:
                correct += 1
                class_wise.setdefault(label, {"correct": 0, "total": 0})["correct"] += 1
            class_wise.setdefault(label, {"correct": 0, "total": 0})["total"] += 1

            acc = 100 * correct / total
            self.node.get_logger().info(f"[{self.current_index}/{len(self.samples)}] ACC: {acc:.2f}%")

            rclpy.spin_once(self.node)

        self.node.get_logger().info(f"\n--- RESULTADOS FINALES ---\n")
        self.node.get_logger().info(f"Accuracy global: {100 * correct / total:.2f}%")
        for cls, stats in class_wise.items():
            acc = 100 * stats["correct"] / stats["total"]
            self.node.get_logger().info(f"Clase {cls}: {acc:.2f}% ({stats['correct']}/{stats['total']})")

        self.get_faceprint_metrics()

    def process_frame(self, frame, real_class):
        frame_msg = self.node.br.cv2_to_imgmsg(frame, "bgr8")
        positions_msg, scores_msg = self.detection_request(frame_msg)
        positions, scores = self.node.br.msg_to_detector(positions_msg, scores_msg)

        classified_name = None
        for i in range(len(positions)):
            result = self.recognition_request(frame_msg, positions_msg[i], scores_msg[i])
            face_aligned_msg, features_msg, classified_id, classified_name_msg, distance_msg, pos_msg, face_updated = result
            face_aligned, features, classified_name, distance, pos = self.node.br.msg_to_recognizer(
                face_aligned_msg, features_msg, classified_name_msg, distance_msg, pos_msg
            )

            if distance < self.LOWER_BOUND and scores[i] >= 1:
                face_aligned_base64 = self.node.br.cv2_to_base64(face_aligned)
                classified_name = self.process_face_name_response(real_class, face_aligned_base64, features, scores[i])
            elif distance < self.MIDDLE_BOUND and scores[i] > 1:
                face_aligned_base64 = self.node.br.cv2_to_base64(face_aligned)
                classified_name = self.process_face_question_response(real_class, classified_id, classified_name, face_aligned_base64, features, scores[i])
            elif distance >= self.UPPER_BOUND:
                self.training_request(String(data="refine_class"), String(data=json.dumps({
                    "class_id": classified_id,
                    "features": features,
                    "position": pos
                })))

            mark_face(frame, positions[i], distance, self.MIDDLE_BOUND, self.UPPER_BOUND, classified=classified_name,
                      drawRectangle=True, score=scores[i], showDistance=True, showScore=True)

        self.node.publisher_recognition.publish(self.node.br.cv2_to_imgmsg(frame, "bgr8"))
        return classified_name

    def process_face_name_response(self, real_class, face_base64, features, score):
        self.node.get_logger().info(f"RESPUESTA: {real_class}")
        self.training_request(String(data="add_class"), String(data=json.dumps({
            "class_name": real_class,
            "features": features,
            "face": face_base64,
            "score": score
        })))
        return real_class

    def process_face_question_response(self, real_class, classified_id, classified_name, face_base64, features, score):
        if real_class == classified_name:
            self.training_request(String(data="add_features"), String(data=json.dumps({
                "class_id": classified_id,
                "features": features,
            })))
        else:
            return self.process_face_name_response(real_class, face_base64, features, score)
        return real_class

    def detection_request(self, frame_msg):
        req = Detection.Request()
        req.frame = frame_msg
        future = self.node.detection_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().positions, future.result().scores

    def recognition_request(self, frame_msg, position_msg, score_msg):
        req = Recognition.Request()
        req.frame = frame_msg
        req.position = position_msg
        req.score = score_msg
        future = self.node.recognition_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return (future.result().face_aligned, future.result().features, future.result().classified_id,
                future.result().classified_name, future.result().distance, future.result().pos,
                future.result().face_updated)

    def training_request(self, cmd_type_msg, args_msg):
        req = Training.Request()
        req.cmd_type = cmd_type_msg
        req.args = args_msg
        future = self.node.training_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().result, future.result().message.data

    def get_faceprint_metrics(self):
        req = GetString.Request()
        req.args = json.dumps({})
        future = self.node.get_faceprints_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        data = json.loads(future.result().text)

        metrics = self.compute_faceprint_metrics(data)
        self.node.get_logger().info(f"\n--- MÉTRICAS DE EMBEDDINGS ---")
        for key, val in metrics.items():
            self.node.get_logger().info(f"{key}: {val:.4f}" if isinstance(val, float) else f"{key}: {val}")

    def compute_faceprint_metrics(self, faceprints):
        class_centroids = {}
        intra_variances = []
        total_embeddings = 0

        for fp in faceprints:
            features_list = fp["features"]
            features = np.array(features_list)
            if len(features) == 0:
                continue
            centroid = np.mean(features, axis=0)
            class_centroids[fp["name"]] = centroid
            diffs = features - centroid
            sq_diffs = np.sum(diffs ** 2, axis=1)
            variance = np.mean(sq_diffs)
            intra_variances.append(variance)
            total_embeddings += len(features)

        centroids = np.array(list(class_centroids.values()))
        centroid_distances = cdist(centroids, centroids)
        np.fill_diagonal(centroid_distances, np.nan)

        return {
            "num_classes": len(class_centroids),
            "mean_class_size": total_embeddings / len(class_centroids),
            "mean_intra_variance": float(np.mean(intra_variances)) if intra_variances else 0.0,
            "mean_inter_class_distance": float(np.nanmean(centroid_distances)),
            "min_inter_class_distance": float(np.nanmin(centroid_distances))
        }


def main(args=None):
    rclpy.init(args=args)
    hri_logic = TestEncoder()
    hri_logic.spin()
    rclpy.shutdown()
