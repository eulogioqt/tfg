import rclpy
from rclpy.node import Node
import os
import json
import cv2
import time
import numpy as np
from tqdm import tqdm
from torchvision.datasets import WIDERFace
import torchvision.transforms as T

from .detectors import (
    CV2Detector, DLIBCNNDetector, DLIBFrontalDetector,
    MTCNNDetector, InsightFaceDetector,
    RetinaFaceDetector, YOLOv5FaceDetector, YOLOv8FaceDetector
)

class TestDetectorNode(Node):
    def __init__(self):
        super().__init__('face_evaluator_node')

        self.get_logger().info('Inicializando evaluación de detectores...')

    def compute_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
        yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])
        interW = max(0, xB - xA)
        interH = max(0, yB - yA)
        interArea = interW * interH
        boxAArea = boxA[2] * boxA[3]
        boxBArea = boxB[2] * boxB[3]
        unionArea = boxAArea + boxBArea - interArea
        return interArea / unionArea if unionArea > 0 else 0

    def match_detections(self, preds, gts, iou_thresh=0.5):
        matches = []
        used_pred = set()
        used_gt = set()
        ious = []
        for i, gt in enumerate(gts):
            for j, pred in enumerate(preds):
                if j in used_pred:
                    continue
                iou = self.compute_iou(gt, pred)
                if iou >= iou_thresh:
                    matches.append((i, j))
                    used_gt.add(i)
                    used_pred.add(j)
                    ious.append(iou)
                    break
        return matches, ious

    def evaluate_detector(self, get_faces, dataset, iou_thresh=0.5):
        total_tp, total_fp, total_fn, total_time = 0, 0, 0, 0.0
        iou_total, iou_count = 0.0, 0

        for img_path, gt_boxes in tqdm(dataset, desc="Evaluando"):
            frame = cv2.imread(img_path)
            if frame is None:
                self.get_logger().error(f"No se pudo leer la imagen: {img_path}")
                continue
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            start = time.time()
            pred_boxes, _ = get_faces(frame_rgb)
            total_time += time.time() - start

            matches, ious = self.match_detections(pred_boxes, gt_boxes, iou_thresh)
            matched_pred_idxs = {j for (_, j) in matches}
            matched_gt_idxs = {i for (i, _) in matches}
            total_tp += len(matches)
            total_fp += len(pred_boxes) - len(matched_pred_idxs)
            total_fn += len(gt_boxes) - len(matched_gt_idxs)
            iou_total += sum(ious)
            iou_count += len(ious)

        precision = total_tp / (total_tp + total_fp + 1e-8)
        recall = total_tp / (total_tp + total_fn + 1e-8)
        f1 = 2 * precision * recall / (precision + recall + 1e-8)
        avg_time = total_time / len(dataset) if dataset else 0
        mean_iou = iou_total / iou_count if iou_count > 0 else 0

        return {
            "true_positives": total_tp,
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1_score": round(f1, 4),
            "mean_iou": round(mean_iou, 4),
            "avg_inference_time": round(avg_time, 4)
        }

    def evaluate_all_detectors(self):
        self.get_logger().info("Cargando dataset WIDER FACE (val)...")
        dataset_wider = WIDERFace(root="widerface_data", split="val", download=True)
        dataset = []
        os.makedirs("widerface_data/tmp", exist_ok=True)

        for idx in range(len(dataset_wider)):
            img, target = dataset_wider[idx]
            img_np = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
            boxes = [(int(x), int(y), int(w), int(h)) for x, y, w, h in target["bbox"]]
            if not (0 <= len(boxes) <= 5):
                continue
            path = f"widerface_data/tmp/img_{idx}.jpg"
            cv2.imwrite(path, img_np)
            dataset.append((path, boxes))

        results_dir = os.path.join(os.path.dirname(__file__), "results")
        os.makedirs(results_dir, exist_ok=True)
        results_path = os.path.join(results_dir, "test_detector_results.json")
        results = {}

        detector_classes = {
            "cv2": CV2Detector,
            "dlib_cnn": DLIBCNNDetector,
            "dlib_frontal": DLIBFrontalDetector,
            "mtcnn": MTCNNDetector,
            "yolov5": YOLOv5FaceDetector,
            "yolov8": YOLOv8FaceDetector,
            "retinaface": RetinaFaceDetector,
            "insightface": InsightFaceDetector
        }

        for name, detector_cls in detector_classes.items():
            self.get_logger().info(f"Evaluando detector: {name}")
            try:
                detector = detector_cls()
                metrics = self.evaluate_detector(detector.get_faces, dataset)
                results[name] = metrics
                with open(results_path, "w") as f:
                    json.dump(results, f, indent=4)
                self.get_logger().info(f"Resultados guardados para {name}")
            except Exception as e:
                self.get_logger().error(f"Fallo en detector '{name}': {e}")

        self.get_logger().info(f"Evaluación completada. Resultados: {results_path}")


def main(args=None):
    rclpy.init(args=args)

    node = TestDetectorNode()
    node.evaluate_all_detectors()
    
    rclpy.shutdown()
