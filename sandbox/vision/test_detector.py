import os
import json
import cv2
import time
import numpy as np
from tqdm import tqdm
from typing import List, Tuple, Callable
from torchvision.datasets import WIDERFace
import torchvision.transforms as T

import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"

from .detectors import (
    CV2Detector, DLIBCNNDetector, DLIBFrontalDetector,
    MTCNNDetector, EfficientFaceDetector,
    RetinaFaceDetector, YOLOv5FaceDetector, YOLOv8FaceDetector
)


def compute_iou(boxA, boxB):
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


def match_detections(preds, gts, iou_thresh=0.5):
    matches = []
    used_pred = set()
    used_gt = set()
    ious = []
    for i, gt in enumerate(gts):
        for j, pred in enumerate(preds):
            if j in used_pred:
                continue
            iou = compute_iou(gt, pred)
            if iou >= iou_thresh:
                matches.append((i, j))
                used_gt.add(i)
                used_pred.add(j)
                ious.append(iou)
                break
    return matches, ious


def draw_boxes_on_image(image_path, gt_boxes, pred_boxes, output_path):
    image = cv2.imread(image_path)
    if image is None:
        print(f"[ERROR] No se pudo leer la imagen: {image_path}")
        return

    for (x, y, w, h) in gt_boxes:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Verde (GT)

    for (x, y, w, h) in pred_boxes:
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Azul (Pred)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, image)


def evaluate_detector(get_faces: Callable, dataset: List[Tuple[str, List[Tuple[int, int, int, int]]]], iou_thresh=0.5):
    total_tp = 0
    total_fp = 0
    total_fn = 0
    total_time = 0
    iou_total = 0.0
    iou_count = 0

    for img_path, gt_boxes in tqdm(dataset, desc="Evaluando"):
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"[ERROR] No se pudo leer la imagen: {img_path}")
            continue
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        start = time.time()
        pred_boxes, _ = get_faces(frame_rgb)
        total_time += time.time() - start

        matches, ious = match_detections(pred_boxes, gt_boxes, iou_thresh)
        matched_pred_idxs = {j for (_, j) in matches}
        matched_gt_idxs = {i for (i, _) in matches}
        total_tp += len(matches)
        total_fp += len(pred_boxes) - len(matched_pred_idxs)
        total_fn += len(gt_boxes) - len(matched_gt_idxs)
        iou_total += sum(ious)
        iou_count += len(ious)

        # Guardar imagen con cajas
        img_name = os.path.splitext(os.path.basename(img_path))[0]
        output_img_path = os.path.join(os.path.dirname(__file__), "results", "tests", f"{img_name}_result.jpg")
        #draw_boxes_on_image(img_path, gt_boxes, pred_boxes, output_img_path)

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


if __name__ == "__main__":
    print("[INFO] Cargando WIDER FACE (split: val)...")
    dataset_wider = WIDERFace(root="widerface_data", split="val", download=True)

    dataset = []
    os.makedirs("widerface_data/tmp", exist_ok=True)

    for idx in range(len(dataset_wider)):
        img, target = dataset_wider[idx]
        img_np = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

        boxes_torch = target["bbox"]
        boxes = [
            (int(xmin), int(ymin), int(width), int(height))
            for xmin, ymin, width, height in boxes_torch
        ]

        if not (0 <= len(boxes) <= 5):
            continue

        tmp_path = f"widerface_data/tmp/img_{idx}.jpg"
        cv2.imwrite(tmp_path, img_np)
        dataset.append((tmp_path, boxes))

    print(f"[INFO] Total de imágenes filtradas: {len(dataset)}")

    # Cargar resultados previos si existen
    results_dir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(results_dir, exist_ok=True)
    results_path = os.path.join(results_dir, "test_detector_results.json")

    if os.path.exists(results_path):
        with open(results_path, "r") as f:
            results = json.load(f)
        print(f"[INFO] Resultados previos cargados: {list(results.keys())}")
    else:
        results = {}

    # Diccionario de detectores
    detector_classes = {
        "cv2": CV2Detector,
        "dlib_cnn": DLIBCNNDetector,
        "dlib_frontal": DLIBFrontalDetector,
        "mtcnn": MTCNNDetector,
        "yolov5": YOLOv5FaceDetector,
        "yolov8": YOLOv8FaceDetector,
        "retinaface": RetinaFaceDetector,
        "efficientface": EfficientFaceDetector
    }

    # Evaluar detectores
    for name, detector_cls in detector_classes.items():
        if name in results:
            print(f"[INFO] Saltando '{name}' (ya evaluado)")
            continue

        print(f"\n[INFO] Evaluando detector: {name}")
        try:
            detector = detector_cls()
            metrics = evaluate_detector(detector.get_faces, dataset)
            results[name] = metrics

            with open(results_path, "w") as f:
                json.dump(results, f, indent=4)
            print(f"[INFO] Resultados guardados para {name}")
        except Exception as e:
            print(f"[ERROR] Fallo en detector '{name}': {e}")

    print(f"\n[INFO] Evaluación completada. Resultados totales en: {results_path}")
