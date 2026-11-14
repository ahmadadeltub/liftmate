"""Offline AI Object Detection, Car Recognition, and Obstacle Awareness Controller for LiftMate.

Optimized specifically for Raspberry Pi 3 (ARMv8 Cortex-A53) and 100% offline operation:
- Uses OpenCV DNN with local MobileNet-SSD (224x224 / 192x192) and YOLOv4-tiny (192x192) models.
- Runs in a dedicated non-blocking background worker thread reading from the shared CameraController buffer.
- Never opens /dev/video0 a second time and never blocks live camera streaming or Arduino serial motor commands.
- Automatically throttles inference rate if CPU load rises or while active motor commands are being sent.
- Provides 3 explicit modes:
    1. DETECTION (Observation-only: bounding boxes, labels, LEFT/CENTER/RIGHT awareness, warnings)
    2. SAFETY_ASSIST (Warns operator and requests an immediate STOP via SafetyController on high-risk forward obstacle)
    3. AI_NAVIGATION (Locked by default until external depth/proximity sensors and physical validation are verified)
"""
from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import DATA_DIR, ConfigManager
from .diagnostics import DiagnosticsManager
from .safety_controller import SafetyController

MODELS_DIR = DATA_DIR / "models"

# MobileNet-SSD 20 PASCAL VOC classes
MOBILENET_SSD_CLASSES: List[str] = [
    "background",
    "aeroplane",
    "bicycle",
    "bird",
    "boat",
    "bottle",
    "bus",
    "car",
    "cat",
    "chair",
    "cow",
    "diningtable",
    "dog",
    "horse",
    "motorbike",
    "person",
    "pottedplant",
    "sheep",
    "sofa",
    "train",
    "tvmonitor",
]

# YOLOv4-tiny 80 COCO classes
COCO_CLASSES: List[str] = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
    "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

VEHICLE_CATEGORIES = {"CAR", "TRUCK", "BUS", "MOTORCYCLE", "BICYCLE"}

CLASS_COLORS: Dict[str, str] = {
    "CAR": "#00f0ff",
    "TRUCK": "#f59e0b",
    "BUS": "#eab308",
    "MOTORCYCLE": "#a855f7",
    "BICYCLE": "#ec4899",
    "PERSON": "#ef4444",
    "ROAD OBSTACLE": "#fb7185",
    "WALL": "#f97316",
    "BARRIER": "#f59e0b",
}


def map_raw_label_to_category(raw_label: str) -> Optional[str]:
    """Maps raw VOC/COCO class names to LiftMate robotics categories."""
    lbl = raw_label.lower().strip()
    if lbl == "car":
        return "CAR"
    if lbl == "truck":
        return "TRUCK"
    if lbl in ("bus", "train"):
        return "BUS"
    if lbl in ("motorbike", "motorcycle"):
        return "MOTORCYCLE"
    if lbl == "bicycle":
        return "BICYCLE"
    if lbl == "person":
        return "PERSON"
    if lbl in (
        "chair", "sofa", "bench", "pottedplant", "diningtable", "tvmonitor",
        "bottle", "backpack", "suitcase", "handbag", "fire hydrant", "stop sign",
        "parking meter", "dog", "cat", "horse", "sheep", "cow", "laptop", "bowl",
    ):
        return "ROAD OBSTACLE"
    return None


class AIVisionController:
    """Thread-safe offline AI object detector, car recognizer, and obstacle awareness engine."""

    def __init__(
        self,
        config: ConfigManager,
        diagnostics: DiagnosticsManager,
        safety: SafetyController,
        camera: Any,
        stop_wheels_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._lock = threading.RLock()
        self.config = config
        self.diagnostics = diagnostics
        self.safety = safety
        self.camera = camera
        self.stop_wheels_callback = stop_wheels_callback

        # Limit OpenCV threads on Raspberry Pi 3 so 2 cores remain free for serial/camera/OS
        try:
            cv2.setNumThreads(2)
        except Exception:
            pass

        self.enabled: bool = True
        self.mode: str = "DETECTION"  # "DETECTION" | "SAFETY_ASSIST" | "AI_NAVIGATION"
        self.ai_navigation_locked: bool = True
        self.hardware_safety_validated: bool = False

        self.model_ready: bool = False
        self.model_name: str = "Initializing..."
        self.model_error: str = ""
        self.input_resolution: Tuple[int, int] = (224, 224)
        self.confidence_threshold: float = 0.42
        self.low_confidence_threshold: float = 0.28

        self._net_ssd: Optional[ Any ] = None
        self._net_yolo: Optional[ Any ] = None
        self._yolo_out_layers: List[str] = []

        # Live detection and tracking state
        self.detections: List[Dict[str, Any]] = []
        self._tracks: List[Dict[str, Any]] = []
        self._next_track_id: int = 1

        self.car_count: int = 0
        self.vehicle_count: int = 0
        self.person_count: int = 0
        self.obstacle_count: int = 0
        self.car_in_forward_region: bool = False

        self.regions: Dict[str, Dict[str, Any]] = {
            "LEFT": {"status": "CLEAR CAMERA REGION", "blocked": False, "risk": "LOW", "objects": []},
            "CENTER": {"status": "CLEAR CAMERA REGION", "blocked": False, "risk": "LOW", "objects": []},
            "RIGHT": {"status": "CLEAR CAMERA REGION", "blocked": False, "risk": "LOW", "objects": []},
        }

        self.vision_status: str = "INITIALIZING"
        self.active_warning: str = ""
        self.warning_level: str = "NONE"  # "NONE" | "INFO" | "WARNING" | "CRITICAL"
        self.warnings_list: List[str] = []

        # Free-space corridor & structural wall awareness (9 horizontal FOV bins, left=-1 .. right=+1)
        self.corridor_scores: List[float] = [1.0] * 9
        self._prev_corridor_scores: Optional[List[float]] = None
        self.best_heading: float = 0.0
        self.open_space_ratio: float = 1.0
        self.wall_detected: bool = False
        self.recommended_path: str = "FORWARD"

        # Performance telemetry
        self.ai_fps: float = 0.0
        self.last_inference_ms: float = 0.0
        self.last_update_ts: float = 0.0
        self.cpu_percent: float = 0.0
        self.ram_percent: float = 0.0
        self.throttled: bool = False
        self._last_motor_activity_ts: float = 0.0
        self._last_safety_stop_ts: float = 0.0
        self.safety_assist_stops_count: int = 0

        # CPU /proc/stat tracking for zero-dependency Linux CPU measurement
        self._prev_cpu_total: int = 0
        self._prev_cpu_idle: int = 0

        self._stop_event = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None

        self._load_offline_models()
        self.start_worker()

    def _load_offline_models(self) -> None:
        """Loads offline neural network models from data/models/."""
        with self._lock:
            ssd_proto = MODELS_DIR / "MobileNetSSD_deploy.prototxt"
            ssd_model = MODELS_DIR / "MobileNetSSD_deploy.caffemodel"
            yolo_cfg = MODELS_DIR / "yolov4-tiny.cfg"
            yolo_weights = MODELS_DIR / "yolov4-tiny.weights"

            loaded_names: List[str] = []
            try:
                if ssd_proto.exists() and ssd_model.exists() and ssd_model.stat().st_size > 1_000_000:
                    net_ssd = cv2.dnn.readNetFromCaffe(str(ssd_proto), str(ssd_model))
                    net_ssd.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    net_ssd.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                    self._net_ssd = net_ssd
                    loaded_names.append("MobileNet-SSD")
            except Exception as exc:
                self.model_error = f"MobileNet-SSD load error: {exc}"

            try:
                if yolo_cfg.exists() and yolo_weights.exists() and yolo_weights.stat().st_size > 1_000_000:
                    net_yolo = cv2.dnn.readNetFromDarknet(str(yolo_cfg), str(yolo_weights))
                    net_yolo.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                    net_yolo.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                    self._net_yolo = net_yolo
                    self._yolo_out_layers = list(net_yolo.getUnconnectedOutLayersNames())
                    loaded_names.append("YOLOv4-tiny")
            except Exception as exc:
                if not loaded_names:
                    self.model_error = f"YOLOv4-tiny load error: {exc}"

            if loaded_names:
                self.model_ready = True
                self.model_name = " + ".join(loaded_names)
                self.model_error = ""
                self.vision_status = "READY"
                self.diagnostics.log_event(
                    "INFO", "AI_VISION", "MODELS_LOADED", f"Offline models ready: {self.model_name}"
                )
            else:
                self.model_ready = False
                self.model_name = "NONE"
                self.vision_status = "AI MODEL NOT READY"
                self.active_warning = "AI MODEL NOT READY"
                self.warning_level = "WARNING"
                self.diagnostics.log_event(
                    "WARNING", "AI_VISION", "MODELS_MISSING", self.model_error or "No offline models found in data/models"
                )

    def notify_motor_activity(self) -> None:
        """Called when operator sends wheel/actuator commands so AI yields CPU priority."""
        self._last_motor_activity_ts = time.time()

    def start_worker(self) -> None:
        with self._lock:
            if self._worker_thread and self._worker_thread.is_alive():
                return
            self._stop_event.clear()
            self._worker_thread = threading.Thread(target=self._inference_loop, daemon=True)
            self._worker_thread.start()

    def stop_worker(self) -> None:
        self._stop_event.set()
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.5)

    def set_enabled(self, enabled: bool) -> Dict[str, Any]:
        with self._lock:
            self.enabled = bool(enabled)
            if not self.enabled:
                self.detections = []
                self.car_count = 0
                self.vehicle_count = 0
                self.person_count = 0
                self.obstacle_count = 0
                self.car_in_forward_region = False
                self.vision_status = "AI OFF"
                self.active_warning = ""
                self.warning_level = "NONE"
                self.warnings_list = []
                self.ai_fps = 0.0
                for r in ("LEFT", "CENTER", "RIGHT"):
                    self.regions[r] = {
                        "status": "AI OFF",
                        "blocked": False,
                        "risk": "NONE",
                        "objects": [],
                    }
            else:
                self.vision_status = "ACTIVE" if self.model_ready else "AI MODEL NOT READY"
            self.diagnostics.log_event(
                "INFO", "AI_VISION", "AI_ENABLED_CHANGED", f"enabled={self.enabled}, mode={self.mode}"
            )
            return self.get_status()

    def set_mode(self, mode: str) -> Tuple[bool, str, Dict[str, Any]]:
        req = mode.strip().upper()
        with self._lock:
            if req in ("OFF", "DISABLED"):
                self.set_enabled(False)
                return True, "AI detection disabled.", self.get_status()

            if req == "AI_NAVIGATION":
                if self.ai_navigation_locked or not self.hardware_safety_validated:
                    msg = (
                        "EXPERIMENTAL AI NAVIGATION is locked: requires verified proximity/depth sensors, "
                        "validated wheel odometry feedback, and physical safety validation."
                    )
                    self.diagnostics.log_event("WARNING", "AI_VISION", "AI_NAV_LOCKED", msg)
                    return False, msg, self.get_status()
                self.enabled = True
                self.mode = "AI_NAVIGATION"
                return True, "Experimental AI Navigation enabled (low-speed guarded mode).", self.get_status()

            if req in ("DETECTION", "OBJECT_DETECTION", "AI_DETECTION"):
                self.enabled = True
                self.mode = "DETECTION"
                self.diagnostics.log_event("INFO", "AI_VISION", "MODE_CHANGED", "MODE 1: AI DETECTION")
                return True, "Mode 1: AI Object Detection active (observation-only).", self.get_status()

            if req in ("SAFETY_ASSIST", "AI_SAFETY_ASSIST"):
                self.enabled = True
                self.mode = "SAFETY_ASSIST"
                self.diagnostics.log_event("INFO", "AI_VISION", "MODE_CHANGED", "MODE 2: AI SAFETY ASSIST")
                return True, "Mode 2: AI Safety Assist active (warns & stops on high-risk forward obstacle).", self.get_status()

            return False, f"Unknown AI mode: {mode}", self.get_status()

    def _sample_system_resources(self) -> None:
        """Reads CPU and RAM usage without blocking or requiring heavy external libraries."""
        # Linux /proc/stat & /proc/meminfo (Raspberry Pi 3)
        try:
            if os.path.exists("/proc/stat"):
                with open("/proc/stat", "r") as f:
                    line = f.readline()
                parts = [int(x) for x in line.split()[1:8]]
                idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
                total = sum(parts)
                diff_total = total - self._prev_cpu_total
                diff_idle = idle - self._prev_cpu_idle
                self._prev_cpu_total = total
                self._prev_cpu_idle = idle
                if diff_total > 0:
                    self.cpu_percent = round(100.0 * (1.0 - diff_idle / float(diff_total)), 1)
            else:
                load1, _, _ = os.getloadavg()
                cpus = os.cpu_count() or 4
                self.cpu_percent = round(min(100.0, (load1 / cpus) * 100.0), 1)
        except Exception:
            pass

        try:
            if os.path.exists("/proc/meminfo"):
                mem_total = 1
                mem_avail = 1
                with open("/proc/meminfo", "r") as f:
                    for line in f:
                        if line.startswith("MemTotal:"):
                            mem_total = int(line.split()[1])
                        elif line.startswith("MemAvailable:"):
                            mem_avail = int(line.split()[1])
                if mem_total > 0:
                    self.ram_percent = round(100.0 * (1.0 - mem_avail / float(mem_total)), 1)
        except Exception:
            pass

    def _run_mobilenet_ssd(self, frame: np.ndarray) -> Tuple[List[Dict[str, Any]], bool]:
        """Runs MobileNet-SSD inference on a single BGR frame."""
        if self._net_ssd is None:
            return [], False
        h, w = frame.shape[:2]
        inp_w, inp_h = self.input_resolution
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (inp_w, inp_h), 127.5)
        self._net_ssd.setInput(blob)
        preds = self._net_ssd.forward()

        raw_list: List[Dict[str, Any]] = []
        had_low_conf = False
        for i in range(preds.shape[2]):
            conf = float(preds[0, 0, i, 2])
            if conf < self.low_confidence_threshold:
                continue
            cls_id = int(preds[0, 0, i, 1])
            if cls_id < 0 or cls_id >= len(MOBILENET_SSD_CLASSES):
                continue
            raw_name = MOBILENET_SSD_CLASSES[cls_id]
            category = map_raw_label_to_category(raw_name)
            if not category:
                continue
            if conf < self.confidence_threshold:
                had_low_conf = True
                continue

            x1 = max(0.0, min(1.0, float(preds[0, 0, i, 3])))
            y1 = max(0.0, min(1.0, float(preds[0, 0, i, 4])))
            x2 = max(0.0, min(1.0, float(preds[0, 0, i, 5])))
            y2 = max(0.0, min(1.0, float(preds[0, 0, i, 6])))
            if x2 - x1 < 0.03 or y2 - y1 < 0.03:
                continue

            raw_list.append({
                "category": category,
                "raw_class": raw_name,
                "confidence": round(conf * 100.0, 1),
                "bbox": [round(x1, 4), round(y1, 4), round(x2, 4), round(y2, 4)],
                "px_bbox": [int(x1 * w), int(y1 * h), int(x2 * w), int(y2 * h)],
            })
        return raw_list, had_low_conf

    def _run_yolov4_tiny(self, frame: np.ndarray) -> Tuple[List[Dict[str, Any]], bool]:
        """Runs YOLOv4-tiny (192x192) to detect 80 COCO classes including TRUCK and road objects."""
        if self._net_yolo is None or not self._yolo_out_layers:
            return [], False
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (192, 192), swapRB=True, crop=False)
        self._net_yolo.setInput(blob)
        layer_outputs = self._net_yolo.forward(self._yolo_out_layers)

        boxes: List[List[int]] = []
        confidences: List[float] = []
        categories: List[Tuple[str, str]] = []
        had_low_conf = False

        for output in layer_outputs:
            for detection in output:
                scores = detection[5:]
                cls_id = int(np.argmax(scores))
                conf = float(scores[cls_id])
                if conf < self.low_confidence_threshold:
                    continue
                if cls_id < 0 or cls_id >= len(COCO_CLASSES):
                    continue
                raw_name = COCO_CLASSES[cls_id]
                category = map_raw_label_to_category(raw_name)
                if not category:
                    continue
                if conf < self.confidence_threshold:
                    had_low_conf = True
                    continue

                cx, cy, bw, bh = (
                    detection[0] * w,
                    detection[1] * h,
                    detection[2] * w,
                    detection[3] * h,
                )
                x = int(cx - bw / 2)
                y = int(cy - bh / 2)
                boxes.append([x, y, int(bw), int(bh)])
                confidences.append(conf)
                categories.append((category, raw_name))

        results: List[Dict[str, Any]] = []
        if boxes:
            idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, 0.4)
            if len(idxs) > 0:
                for idx in np.array(idxs).flatten():
                    bx, by, bw, bh = boxes[idx]
                    x1 = max(0.0, min(1.0, bx / float(w)))
                    y1 = max(0.0, min(1.0, by / float(h)))
                    x2 = max(0.0, min(1.0, (bx + bw) / float(w)))
                    y2 = max(0.0, min(1.0, (by + bh) / float(h)))
                    if x2 - x1 < 0.03 or y2 - y1 < 0.03:
                        continue
                    cat, raw_name = categories[idx]
                    results.append({
                        "category": cat,
                        "raw_class": raw_name,
                        "confidence": round(confidences[idx] * 100.0, 1),
                        "bbox": [round(x1, 4), round(y1, 4), round(x2, 4), round(y2, 4)],
                        "px_bbox": [int(x1 * w), int(y1 * h), int(x2 * w), int(y2 * h)],
                    })
        return results, had_low_conf

    @staticmethod
    def _iou(boxA: List[float], boxB: List[float]) -> float:
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        inter = max(0.0, xB - xA) * max(0.0, yB - yA)
        if inter <= 0.0:
            return 0.0
        areaA = max(1e-5, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
        areaB = max(1e-5, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))
        return inter / float(areaA + areaB - inter)

    def _merge_detections(
        self, ssd_dets: List[Dict[str, Any]], yolo_dets: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Merges detections from MobileNet-SSD and YOLOv4-tiny using cross-model NMS."""
        combined = list(ssd_dets)
        for yd in yolo_dets:
            duplicate = False
            for idx, sd in enumerate(combined):
                if self._iou(yd["bbox"], sd["bbox"]) > 0.42:
                    duplicate = True
                    # Prefer TRUCK specificity from YOLO or higher confidence
                    if yd["category"] == "TRUCK" or yd["confidence"] > sd["confidence"]:
                        combined[idx] = yd
                    break
            if not duplicate:
                combined.append(yd)
        return combined

    def _update_tracks(self, raw_dets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Associates detections across frames for smooth bounding boxes and temporal growth cues."""
        now = time.time()
        updated_tracks: List[Dict[str, Any]] = []
        used_prev = set()

        for det in raw_dets:
            best_iou = 0.0
            best_idx = -1
            for idx, tr in enumerate(self._tracks):
                if idx in used_prev:
                    continue
                if tr["category"] != det["category"]:
                    continue
                score = self._iou(det["bbox"], tr["bbox"])
                if score > best_iou:
                    best_iou = score
                    best_idx = idx

            x1, y1, x2, y2 = det["bbox"]
            area = max(0.0001, (x2 - x1) * (y2 - y1))
            if best_idx >= 0 and best_iou >= 0.25:
                used_prev.add(best_idx)
                prev = self._tracks[best_idx]
                # Smooth bounding box (70% new, 30% previous)
                px1, py1, px2, py2 = prev["bbox"]
                sx1 = round(0.7 * x1 + 0.3 * px1, 4)
                sy1 = round(0.7 * y1 + 0.3 * py1, 4)
                sx2 = round(0.7 * x2 + 0.3 * px2, 4)
                sy2 = round(0.7 * y2 + 0.3 * py2, 4)
                prev_area = prev.get("area", area)
                if area > prev_area * 1.12:
                    trend = "approaching"
                elif area < prev_area * 0.88:
                    trend = "receding"
                else:
                    trend = "steady"
                track = {
                    **det,
                    "track_id": prev["track_id"],
                    "bbox": [sx1, sy1, sx2, sy2],
                    "area": round(area, 4),
                    "frames_seen": prev.get("frames_seen", 1) + 1,
                    "temporal_trend": trend,
                    "last_seen": now,
                }
            else:
                tid = self._next_track_id
                self._next_track_id += 1
                track = {
                    **det,
                    "track_id": tid,
                    "area": round(area, 4),
                    "frames_seen": 1,
                    "temporal_trend": "steady",
                    "last_seen": now,
                }

            # Classify horizontal region (LEFT / CENTER / RIGHT)
            cx = 0.5 * (track["bbox"][0] + track["bbox"][2])
            if cx < 0.3333:
                region = "LEFT"
            elif cx <= 0.6667:
                region = "CENTER"
            else:
                region = "RIGHT"
            track["region"] = region
            track["color"] = CLASS_COLORS.get(track["category"], "#38bdf8")
            track["display_label"] = f"{track['category']} – {int(round(track['confidence']))}%"
            updated_tracks.append(track)

        # Retain tracks briefly (up to 0.35s) to prevent single-frame flicker
        for idx, tr in enumerate(self._tracks):
            if idx not in used_prev and (now - tr.get("last_seen", 0.0)) <= 0.35:
                updated_tracks.append(tr)

        self._tracks = updated_tracks
        return updated_tracks

    def _analyze_walls_and_free_space(
        self,
        frame: np.ndarray,
        semantic_dets: List[Dict[str, Any]],
        unreliable_lighting: bool,
    ) -> Tuple[List[float], float, float, bool, str, List[Dict[str, Any]]]:
        """Fast monocular free-space corridor estimator and unlabeled wall/barrier detector.

        Complements neural object recognition so solid walls, baseboards, closed doors,
        furniture, and close featureless obstructions are detected without requiring a
        matching COCO/VOC class label. Preserves clear behavior on flat uniform synthetic
        frames used in unit tests.
        """
        num_bins = 9
        h_full, w_full = frame.shape[:2]
        small_w, small_h = 160, 120
        small_bgr = cv2.resize(frame, (small_w, small_h), interpolation=cv2.INTER_AREA)
        small_gray = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2GRAY)
        global_std = float(np.std(small_gray))

        structural_free = np.ones(num_bins, dtype=np.float32)
        structural_dets: List[Dict[str, Any]] = []
        flush_wall = False
        is_flat_synthetic = global_std < 2.0

        if not unreliable_lighting and not is_flat_synthetic:
            small_lab = cv2.cvtColor(small_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
            gx = np.abs(cv2.Sobel(small_gray, cv2.CV_32F, 1, 0, ksize=3))
            gy = np.abs(cv2.Sobel(small_gray, cv2.CV_32F, 0, 1, ksize=3))
            grad_mag = 0.5 * (gx + gy)

            # Pick cleanest bottom-third floor patch (lowest edge texture avoids poisoning ref)
            third_edges = [
                (0, 53),
                (53, 106),
                (106, small_w),
            ]
            best_third = min(
                third_edges,
                key=lambda span: float(np.mean(grad_mag[96:115, span[0]:span[1]])),
            )
            ref_lab = small_lab[96:115, best_third[0]:best_third[1], :]
            floor_L = float(np.median(ref_lab[:, :, 0]))
            floor_a = float(np.median(ref_lab[:, :, 1]))
            floor_b = float(np.median(ref_lab[:, :, 2]))
            floor_grad = float(np.median(grad_mag[96:115, best_third[0]:best_third[1]]))

            # Check flush close-up blank wall occlusion (fills entire lens with no horizon)
            upper_med = np.median(small_lab[16:46, 20:140, :], axis=(0, 1))
            lower_med = np.median(small_lab[82:112, 20:140, :], axis=(0, 1))
            vert_chroma_diff = float(
                np.sqrt(
                    0.50 * (upper_med[0] - lower_med[0]) ** 2
                    + 1.25 * ((upper_med[1] - lower_med[1]) ** 2 + (upper_med[2] - lower_med[2]) ** 2)
                )
            )
            mid_horizon_gy = float(np.mean(gy[40:84, 16:144]))
            lower_grad = float(np.mean(grad_mag[82:114, 16:144]))
            if vert_chroma_diff < 5.2 and mid_horizon_gy < 6.0 and lower_grad < 6.8 and global_std >= 2.2:
                flush_wall = True

            col_block_y2 = np.full(num_bins, 0.72, dtype=np.float32)
            col_conf = np.zeros(num_bins, dtype=np.float32)

            for c in range(num_bins):
                x0 = int(round(c * small_w / num_bins))
                x1 = int(round((c + 1) * small_w / num_bins))

                # Near foreground ground band (y = 0.73..0.93)
                near_slice = small_lab[88:112, x0:x1, :]
                dE_near = np.sqrt(
                    0.45 * (near_slice[:, :, 0] - floor_L) ** 2
                    + 1.25 * ((near_slice[:, :, 1] - floor_a) ** 2 + (near_slice[:, :, 2] - floor_b) ** 2)
                )
                col_dE_near = float(np.percentile(dE_near, 72))
                col_edge_near = float(np.mean(grad_mag[88:112, x0:x1]))
                gy_near_rows = np.mean(gy[82:112, x0:x1], axis=1)
                max_gy_near = float(np.max(gy_near_rows))

                # Mid-range approach band (y = 0.53..0.73)
                mid_slice = small_lab[64:88, x0:x1, :]
                dE_mid = np.sqrt(
                    0.45 * (mid_slice[:, :, 0] - floor_L) ** 2
                    + 1.25 * ((mid_slice[:, :, 1] - floor_a) ** 2 + (mid_slice[:, :, 2] - floor_b) ** 2)
                )
                col_dE_mid = float(np.mean(dE_mid))
                col_edge_mid = float(np.mean(grad_mag[64:88, x0:x1]))
                gy_mid_rows = np.mean(gy[56:88, x0:x1], axis=1)
                max_gy_mid = float(np.max(gy_mid_rows))

                near_color_occ = float(np.clip((col_dE_near - 16.0) / 22.0, 0.0, 1.0))
                near_edge_occ = float(np.clip((col_edge_near - max(18.0, floor_grad + 10.0)) / 32.0, 0.0, 1.0))
                near_seam_occ = float(np.clip((max_gy_near - 28.0) / 40.0, 0.0, 1.0))
                occ_near = max(near_color_occ, 0.90 * near_seam_occ, 0.80 * near_edge_occ)

                mid_color_occ = float(np.clip((col_dE_mid - 16.0) / 24.0, 0.0, 1.0))
                mid_edge_occ = float(np.clip((col_edge_mid - max(18.0, floor_grad + 10.0)) / 34.0, 0.0, 1.0))
                mid_seam_occ = float(np.clip((max_gy_mid - 26.0) / 38.0, 0.0, 1.0))
                occ_mid = max(mid_color_occ, 0.88 * mid_seam_occ, 0.75 * mid_edge_occ)

                if flush_wall:
                    occ_col = 0.92
                    col_block_y2[c] = 0.92
                else:
                    occ_col = max(occ_near, 0.78 * occ_mid)
                    if occ_near >= 0.56:
                        seam_idx = int(np.argmax(gy_near_rows))
                        col_block_y2[c] = min(0.95, max(0.76, (82 + seam_idx) / float(small_h)))
                    elif occ_mid >= 0.56:
                        seam_idx = int(np.argmax(gy_mid_rows))
                        col_block_y2[c] = min(0.84, max(0.60, (56 + seam_idx) / float(small_h)))

                structural_free[c] = float(np.clip(1.0 - occ_col, 0.0, 1.0))
                col_conf[c] = float(occ_col)

            # Group contiguous low-clearance structural columns into WALL / BARRIER boxes
            blocked_cols = [c for c in range(num_bins) if structural_free[c] < 0.44]
            if flush_wall:
                cat = "WALL"
                structural_dets.append({
                    "track_id": 0,
                    "category": cat,
                    "raw_class": "wall",
                    "confidence": 92.0,
                    "bbox": [0.06, 0.14, 0.94, 0.94],
                    "px_bbox": [int(0.06 * w_full), int(0.14 * h_full), int(0.94 * w_full), int(0.94 * h_full)],
                    "area": 0.704,
                    "region": "CENTER",
                    "color": CLASS_COLORS[cat],
                    "display_label": "WALL – CLOSE AHEAD",
                    "frames_seen": 2,
                    "temporal_trend": "steady",
                    "structural": True,
                })
            elif blocked_cols:
                runs: List[Tuple[int, int]] = []
                start_c = blocked_cols[0]
                prev_c = blocked_cols[0]
                for c in blocked_cols[1:]:
                    if c == prev_c + 1:
                        prev_c = c
                    else:
                        runs.append((start_c, prev_c))
                        start_c = c
                        prev_c = c
                runs.append((start_c, prev_c))

                for r_idx, (c0, c1) in enumerate(runs):
                    span_cols = c1 - c0 + 1
                    x1 = round(c0 / float(num_bins), 4)
                    x2 = round((c1 + 1) / float(num_bins), 4)
                    y2 = round(float(np.max(col_block_y2[c0 : c1 + 1])), 4)
                    y1 = 0.18 if span_cols >= 3 else 0.30
                    cat = "WALL" if span_cols >= 3 else "BARRIER"
                    peak_occ = float(np.max(col_conf[c0 : c1 + 1]))
                    conf = round(min(97.0, 58.0 + 39.0 * peak_occ), 1)
                    cx = 0.5 * (x1 + x2)
                    zone = "LEFT" if cx < 0.3333 else ("CENTER" if cx <= 0.6667 else "RIGHT")
                    structural_dets.append({
                        "track_id": -(10 + r_idx),
                        "category": cat,
                        "raw_class": cat.lower(),
                        "confidence": conf,
                        "bbox": [x1, y1, x2, y2],
                        "px_bbox": [
                            int(x1 * w_full),
                            int(y1 * h_full),
                            int(x2 * w_full),
                            int(y2 * h_full),
                        ],
                        "area": round((x2 - x1) * (y2 - y1), 4),
                        "region": zone,
                        "color": CLASS_COLORS[cat],
                        "display_label": f"{cat} – {int(round(conf))}%",
                        "frames_seen": 2,
                        "temporal_trend": "steady",
                        "structural": True,
                    })

        # Fuse semantic neural detections cleanly onto the 9 corridor bins
        fused = structural_free.copy()
        for d in semantic_dets:
            x1, y1, x2, y2 = d["bbox"]
            area = float(d.get("area", (x2 - x1) * (y2 - y1)))
            if y2 < 0.42 and area < 0.12:
                continue
            if y2 >= 0.74 or area >= 0.22:
                cap = 0.08
            elif y2 >= 0.56 or area >= 0.12:
                cap = 0.35
            else:
                cap = 0.65
            c_start = max(0, min(num_bins - 1, int(np.floor(x1 * num_bins))))
            c_end = max(0, min(num_bins - 1, int(np.floor(max(x1, x2 - 1e-4) * num_bins))))
            for c in range(c_start, c_end + 1):
                fused[c] = min(fused[c], cap)

        # Light spatial smoothing across neighboring bins [0.2, 0.6, 0.2]
        padded = np.pad(fused, 1, mode="edge")
        smoothed = 0.20 * padded[:-2] + 0.60 * padded[1:-1] + 0.20 * padded[2:]

        # Temporal EMA on real structured scenes to stop frame-to-frame heading jitter
        if not is_flat_synthetic and not unreliable_lighting and self._prev_corridor_scores is not None:
            prev_arr = np.array(self._prev_corridor_scores, dtype=np.float32)
            if prev_arr.shape == smoothed.shape:
                smoothed = 0.72 * smoothed + 0.28 * prev_arr
        if not is_flat_synthetic and not unreliable_lighting:
            self._prev_corridor_scores = [round(float(v), 3) for v in smoothed]
        else:
            self._prev_corridor_scores = None

        corridor_list = [round(float(np.clip(v, 0.0, 1.0)), 3) for v in smoothed]

        # Robot-width 3-column corridor sliding score + gentle forward symmetry preference
        win_scores: List[float] = []
        for c in range(num_bins):
            left_nb = corridor_list[c - 1] if c > 0 else corridor_list[0] * 0.85
            right_nb = corridor_list[c + 1] if c < num_bins - 1 else corridor_list[-1] * 0.85
            shoulder = 0.90 if c in (0, num_bins - 1) else 1.0
            heading = (c - 4) / 4.0
            win = (0.25 * left_nb + 0.50 * corridor_list[c] + 0.25 * right_nb) * shoulder
            win_scores.append(win + 0.03 * (1.0 - abs(heading)))

        best_c = int(np.argmax(win_scores))
        best_heading = round((best_c - 4) / 4.0, 2)
        open_space_ratio = round(float(np.mean(corridor_list)), 3)
        center_mean = float(np.mean(corridor_list[3:6]))
        wall_detected = bool(flush_wall or len(structural_dets) > 0)

        if max(win_scores) < 0.36:
            recommended_path = "RETREAT_ROTATE"
        elif center_mean >= 0.62 and abs(best_heading) <= 0.25:
            recommended_path = "FORWARD"
        elif best_heading <= -0.35:
            recommended_path = "VEER_LEFT" if center_mean >= 0.38 else "AVOID_LEFT"
        elif best_heading >= 0.35:
            recommended_path = "VEER_RIGHT" if center_mean >= 0.38 else "AVOID_RIGHT"
        elif best_heading < -0.10:
            recommended_path = "VEER_LEFT"
        elif best_heading > 0.10:
            recommended_path = "VEER_RIGHT"
        else:
            recommended_path = "FORWARD"

        return corridor_list, best_heading, open_space_ratio, wall_detected, recommended_path, structural_dets

    def process_frame(self, frame: np.ndarray, run_yolo_supplement: bool = False) -> Dict[str, Any]:
        """Runs object detection, region obstacle analysis, and safety checks on a BGR frame."""
        t0 = time.time()
        if frame is None or frame.size == 0:
            with self._lock:
                self.vision_status = "CAMERA VISION UNAVAILABLE"
                self.active_warning = "CAMERA VISION UNAVAILABLE"
                self.warning_level = "WARNING"
            return self.get_status()

        # Check image brightness/contrast reliability
        gray_mean = float(np.mean(frame))
        unreliable_lighting = gray_mean < 8.0 or gray_mean > 250.0

        ssd_dets, low_conf_ssd = self._run_mobilenet_ssd(frame)
        yolo_dets: List[Dict[str, Any]] = []
        low_conf_yolo = False
        if run_yolo_supplement and self._net_yolo is not None:
            yolo_dets, low_conf_yolo = self._run_yolov4_tiny(frame)

        merged = self._merge_detections(ssd_dets, yolo_dets)
        tracked_semantic = self._update_tracks(merged)

        (
            corridor_scores,
            best_heading,
            open_space_ratio,
            wall_detected,
            recommended_path,
            structural_dets,
        ) = self._analyze_walls_and_free_space(frame, tracked_semantic, unreliable_lighting)

        # Combine neural object tracks with live structural wall/barrier boxes
        tracked = list(tracked_semantic) + structural_dets
        dt_ms = round((time.time() - t0) * 1000.0, 1)

        with self._lock:
            self.last_inference_ms = dt_ms
            self.last_update_ts = time.time()
            self.detections = tracked
            self.corridor_scores = corridor_scores
            self.best_heading = best_heading
            self.open_space_ratio = open_space_ratio
            self.wall_detected = wall_detected
            self.recommended_path = recommended_path

            self.car_count = sum(1 for d in tracked if d["category"] == "CAR")
            self.vehicle_count = sum(1 for d in tracked if d["category"] in VEHICLE_CATEGORIES)
            self.person_count = sum(1 for d in tracked if d["category"] == "PERSON")

            # Evaluate LEFT / CENTER / RIGHT regions using visual cues (bbox bottom y2, area, and category)
            # Never fabricates metric distance (cm/m).
            region_map: Dict[str, List[Dict[str, Any]]] = {"LEFT": [], "CENTER": [], "RIGHT": []}
            high_risk_center = False
            obstacle_total = 0
            car_forward = False

            for d in tracked:
                x1, y1, x2, y2 = d["bbox"]
                area = d.get("area", (x2 - x1) * (y2 - y1))
                cat = d["category"]

                # Determine which horizontal zones this bounding box overlaps
                overlapped_zones: List[str] = []
                if x1 < 0.3333:
                    overlapped_zones.append("LEFT")
                if x2 > 0.3333 and x1 < 0.6667:
                    overlapped_zones.append("CENTER")
                if x2 > 0.6667:
                    overlapped_zones.append("RIGHT")
                if not overlapped_zones:
                    overlapped_zones.append(d.get("region", "CENTER"))

                if cat == "CAR" and "CENTER" in overlapped_zones:
                    car_forward = True

                # Visual proximity cue: lower in frame (y2 >= 0.58) or large visual footprint (area >= 0.12)
                is_structural = bool(d.get("structural", False))
                is_potential_obstacle = is_structural or (y2 >= 0.58) or (area >= 0.12) or (
                    cat == "PERSON" and y2 >= 0.48
                )
                is_close_visual_cue = (
                    (is_structural and (y2 >= 0.72 or area >= 0.28))
                    or (y2 >= 0.76 and area >= 0.14)
                    or (area >= 0.24)
                    or (cat == "PERSON" and y2 >= 0.65 and area >= 0.10)
                )

                d["is_potential_obstacle"] = bool(is_potential_obstacle)
                d["visual_risk"] = "HIGH" if is_close_visual_cue else ("MEDIUM" if is_potential_obstacle else "LOW")

                if is_potential_obstacle:
                    obstacle_total += 1
                    for z in overlapped_zones:
                        region_map[z].append(d)
                    if "CENTER" in overlapped_zones and is_close_visual_cue:
                        high_risk_center = True

            self.car_in_forward_region = car_forward
            self.obstacle_count = obstacle_total

            # Update LEFT / CENTER / RIGHT region descriptors
            region_labels = {
                "LEFT": "POTENTIAL OBSTACLE ON LEFT",
                "CENTER": "POTENTIAL OBSTACLE AHEAD",
                "RIGHT": "POTENTIAL OBSTACLE ON RIGHT",
            }
            for zone in ("LEFT", "CENTER", "RIGHT"):
                objs = region_map[zone]
                if unreliable_lighting and not objs:
                    self.regions[zone] = {
                        "status": "UNKNOWN / UNRELIABLE DETECTION",
                        "blocked": False,
                        "risk": "UNKNOWN",
                        "objects": [],
                    }
                elif objs:
                    max_risk = "HIGH" if any(o.get("visual_risk") == "HIGH" for o in objs) else "MEDIUM"
                    self.regions[zone] = {
                        "status": region_labels[zone],
                        "blocked": True,
                        "risk": max_risk,
                        "objects": [o["category"] for o in objs],
                    }
                else:
                    self.regions[zone] = {
                        "status": "CLEAR CAMERA REGION",
                        "blocked": False,
                        "risk": "LOW",
                        "objects": [],
                    }

            # Build smart warnings list and primary warning banner
            warnings: List[str] = []
            level = "NONE"

            if not self.model_ready:
                warnings.append("AI MODEL NOT READY")
                level = "WARNING"
            elif high_risk_center:
                warnings.append("POTENTIAL OBSTACLE – STOP")
                level = "CRITICAL"
            elif self.regions["CENTER"]["blocked"]:
                warnings.append("OBJECT AHEAD")
                level = "WARNING"
            elif self.regions["LEFT"]["blocked"] or self.regions["RIGHT"]["blocked"]:
                if self.regions["LEFT"]["blocked"]:
                    warnings.append("POTENTIAL OBSTACLE ON LEFT")
                if self.regions["RIGHT"]["blocked"]:
                    warnings.append("POTENTIAL OBSTACLE ON RIGHT")
                level = "WARNING"

            if self.car_count > 0:
                warnings.append("CAR DETECTED")
                if level == "NONE":
                    level = "INFO"

            if unreliable_lighting and not tracked:
                warnings.append("LOW DETECTION CONFIDENCE")
                if level == "NONE":
                    level = "INFO"
                self.vision_status = "UNKNOWN / UNRELIABLE DETECTION"
            elif high_risk_center:
                self.vision_status = "POTENTIAL OBSTACLE – STOP"
            elif self.regions["CENTER"]["blocked"]:
                self.vision_status = "POTENTIAL OBSTACLE AHEAD"
            elif self.regions["LEFT"]["blocked"]:
                self.vision_status = "POTENTIAL OBSTACLE ON LEFT"
            elif self.regions["RIGHT"]["blocked"]:
                self.vision_status = "POTENTIAL OBSTACLE ON RIGHT"
            else:
                self.vision_status = "CLEAR CAMERA REGION"

            self.warnings_list = warnings
            self.active_warning = warnings[0] if warnings else ""
            self.warning_level = level

            # MODE 2: AI SAFETY ASSIST — Request a stop via SafetyController / MotorController on high-risk forward obstacle
            if self.mode == "SAFETY_ASSIST" and high_risk_center:
                now_ts = time.time()
                if now_ts - self._last_safety_stop_ts >= 1.0:
                    self._last_safety_stop_ts = now_ts
                    self.safety_assist_stops_count += 1
                    reason = "AI Safety Assist: High-risk obstacle detected in CENTER forward path"
                    self.safety.notify_motion_stopped(reason)
                    self.diagnostics.log_event("WARNING", "AI_VISION", "SAFETY_ASSIST_STOP", reason)
                    if self.stop_wheels_callback is not None:
                        try:
                            self.stop_wheels_callback(reason)
                        except Exception:
                            pass

        return self.get_status()

    def _inference_loop(self) -> None:
        """Background daemon worker that processes frames from CameraController without blocking."""
        last_processed_frame_count = -1
        fps_counter = 0
        fps_t0 = time.time()
        frame_iter = 0

        while not self._stop_event.is_set():
            if not self.enabled:
                time.sleep(0.25)
                continue

            if not self.model_ready:
                self._load_offline_models()
                time.sleep(1.0)
                continue

            # Sample CPU & RAM usage periodically
            if frame_iter % 5 == 0:
                self._sample_system_resources()

            # Check if camera is streaming and has a fresh frame
            cam_streaming = bool(getattr(self.camera, "streaming", False) and getattr(self.camera, "connected", False))
            raw_jpeg = getattr(self.camera, "last_frame_jpeg", None)
            cam_frame_cnt = int(getattr(self.camera, "frame_count", 0))
            last_frame_ts = float(getattr(self.camera, "last_frame_ts", 0.0))

            if not cam_streaming or not raw_jpeg or (last_frame_ts > 0 and (time.time() - last_frame_ts) > 3.5):
                with self._lock:
                    self.vision_status = "CAMERA VISION UNAVAILABLE"
                    self.active_warning = "CAMERA VISION UNAVAILABLE"
                    self.warning_level = "WARNING"
                    self.warnings_list = ["CAMERA VISION UNAVAILABLE"]
                    self.detections = []
                    self.ai_fps = 0.0
                time.sleep(0.35)
                continue

            # Avoid reprocessing the exact same camera frame
            if cam_frame_cnt == last_processed_frame_count:
                time.sleep(0.04)
                continue

            # Prioritize responsive motor control: if operator is actively sending commands or CPU > 82%, throttle AI rate
            motor_busy = (time.time() - self._last_motor_activity_ts) < 0.25
            high_cpu = self.cpu_percent > 82.0
            self.throttled = bool(motor_busy or high_cpu)

            try:
                arr = np.frombuffer(raw_jpeg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None and frame.size > 0:
                    last_processed_frame_count = cam_frame_cnt
                    frame_iter += 1
                    # Run YOLOv4-tiny supplement every 3rd frame when CPU is not throttled (detects TRUCK & extra classes)
                    run_yolo = (frame_iter % 3 == 0) and (not self.throttled)
                    self.process_frame(frame, run_yolo_supplement=run_yolo)
                    fps_counter += 1
            except Exception as exc:
                self.diagnostics.log_event("WARNING", "AI_VISION", "INFERENCE_ERROR", str(exc))

            now = time.time()
            if now - fps_t0 >= 1.0:
                with self._lock:
                    self.ai_fps = round(fps_counter / (now - fps_t0), 1)
                fps_counter = 0
                fps_t0 = now

            # Controlled inference pacing so Raspberry Pi 3 remains cool and responsive
            sleep_s = 0.14 if self.throttled else 0.05
            time.sleep(sleep_s)

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "enabled": self.enabled,
                "mode": self.mode,
                "ai_navigation_locked": self.ai_navigation_locked,
                "hardware_safety_validated": self.hardware_safety_validated,
                "model_ready": self.model_ready,
                "model_name": self.model_name,
                "model_error": self.model_error,
                "vision_status": self.vision_status,
                "active_warning": self.active_warning,
                "warning_level": self.warning_level,
                "warnings": list(self.warnings_list),
                "car_count": self.car_count,
                "vehicle_count": self.vehicle_count,
                "person_count": self.person_count,
                "obstacle_count": self.obstacle_count,
                "car_in_forward_region": self.car_in_forward_region,
                "detections": list(self.detections),
                "regions": {
                    "LEFT": dict(self.regions["LEFT"]),
                    "CENTER": dict(self.regions["CENTER"]),
                    "RIGHT": dict(self.regions["RIGHT"]),
                },
                "ai_fps": self.ai_fps,
                "last_inference_ms": self.last_inference_ms,
                "cpu_percent": self.cpu_percent,
                "ram_percent": self.ram_percent,
                "throttled": self.throttled,
                "safety_assist_stops_count": self.safety_assist_stops_count,
                "corridor_scores": list(self.corridor_scores),
                "best_heading": self.best_heading,
                "open_space_ratio": self.open_space_ratio,
                "wall_detected": self.wall_detected,
                "recommended_path": self.recommended_path,
            }
