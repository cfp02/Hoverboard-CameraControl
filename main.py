"""
LeafBot Vision MVP:
- Reads RTSP or MJPEG from --source URL
- Detects a PERSON with YOLO (default) or EfficientDet
- Draws a bounding box and overlays simple control signals (ex, ez) for future use
- Designed to be easily extended into a follow/edge-driving controller

Usage examples:
  python leafbot_detect.py --source "rtsp://user:pass@ip:port/stream"
  python leafbot_detect.py --source "http://ip:port/mjpg"
  python leafbot_detect.py --model yolo --model-name yolov8n.pt
  python leafbot_detect.py --model efficientdet --model-name efficientdet_lite0

Press 'q' to quit.
"""

import argparse
import time
from typing import Optional, Tuple

import cv2
import numpy as np

# --- Detector Abstractions ----------------------------------------------------

class Detection:
    """Simple container for one detection."""
    def __init__(self, xyxy: Tuple[int,int,int,int], conf: float, cls_name: str):
        self.xyxy = xyxy  # (x1,y1,x2,y2) in pixels
        self.conf = conf
        self.cls_name = cls_name

class BaseDetector:
    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        raise NotImplementedError

# ---- YOLO (Ultralytics) ------------------------------------------------------

class YoloDetector(BaseDetector):
    def __init__(self, model_name: str = "yolov8n.pt", device: str = "cpu", conf_thres: float = 0.4):
        from ultralytics import YOLO
        self.model = YOLO(model_name)  # auto-downloads
        self.conf_thres = conf_thres
        self.device = device
        # Restrict to person class id if the model is COCO-trained (class 0 in COCO)
        self.person_names = set(["person"])  # robust to custom class names

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        # Ultralytics expects RGB
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.model.predict(frame_rgb, verbose=False, conf=self.conf_thres, device=self.device)
        dets: list[Detection] = []
        if not results:
            return dets
        r = results[0]
        if r.boxes is None:
            return dets
        names = r.names if hasattr(r, "names") else {}
        for box in r.boxes:
            cls_id = int(box.cls[0]) if box.cls is not None else -1
            cls_name = names.get(cls_id, str(cls_id))
            if cls_name not in self.person_names:
                continue
            conf = float(box.conf[0]) if box.conf is not None else 0.0
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            dets.append(Detection((x1,y1,x2,y2), conf, cls_name))
        return dets

# ---- EfficientDet via TensorFlow Hub ----------------------------------------

class EfficientDetDetector(BaseDetector):
    """
    Uses TF-Hub models like:
      - 'efficientdet_lite0' (fastest)
      - 'efficientdet_lite1', 'efficientdet_lite2' (bigger)
    """
    def __init__(self, model_name: str = "efficientdet_lite0", conf_thres: float = 0.4):
        import tensorflow as tf
        import tensorflow_hub as hub

        # Map simple names to TF-Hub handles
        handles = {
            "efficientdet_lite0": "https://tfhub.dev/tensorflow/efficientdet/lite0/detection/1",
            "efficientdet_lite1": "https://tfhub.dev/tensorflow/efficientdet/lite1/detection/1",
            "efficientdet_lite2": "https://tfhub.dev/tensorflow/efficientdet/lite2/detection/1",
        }
        self.tf = tf
        self.model = hub.load(handles[model_name])
        self.conf_thres = conf_thres

        # COCO person class id is 1 for this TF-Hub model outputs (verify in practice)
        # We'll filter by COCO label map; TF-Hub returns class IDs starting at 1
        self.person_class_ids = {1}

    def detect(self, frame_bgr: np.ndarray) -> list[Detection]:
        tf = self.tf
        # Model expects RGB float32 in [0,1], with batch dim
        img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inp = tf.convert_to_tensor(img_rgb, dtype=tf.uint8)
        inp = tf.expand_dims(inp, 0)  # [1,H,W,3]

        outputs = self.model(inp)
        # Outputs: 'detection_boxes' [1,100,4] (ymin,xmin,ymax,xmax) normalized,
        #          'detection_scores' [1,100], 'detection_classes' [1,100]
        boxes = outputs["detection_boxes"][0].numpy()
        scores = outputs["detection_scores"][0].numpy()
        classes = outputs["detection_classes"][0].numpy().astype(int)

        H, W = frame_bgr.shape[:2]
        dets: list[Detection] = []
        for box, score, cls in zip(boxes, scores, classes):
            if score < self.conf_thres:
                continue
            if cls not in self.person_class_ids:
                continue
            ymin, xmin, ymax, xmax = box  # normalized
            x1, y1 = int(xmin * W), int(ymin * H)
            x2, y2 = int(xmax * W), int(ymax * H)
            dets.append(Detection((x1, y1, x2, y2), float(score), "person"))
        return dets

# --- Utility ------------------------------------------------------------------

def pick_target(detections: list[Detection], strategy: str = "largest") -> Optional[Detection]:
    """
    Choose one person detection to follow/highlight.
    - 'largest': pick the bbox with largest area (proxy for closest person)
    - 'conf': pick highest confidence
    """
    if not detections:
        return None
    if strategy == "conf":
        return max(detections, key=lambda d: d.conf)
    # default: largest area
    return max(detections, key=lambda d: (d.xyxy[2]-d.xyxy[0])*(d.xyxy[3]-d.xyxy[1]))

def compute_errors(bbox: Tuple[int,int,int,int], W: int, H: int, target_h_frac: float = 0.45) -> Tuple[float,float]:
    """
    Compute simple control-friendly errors from a bbox:
      ex: lateral error in [-1,1] (left negative, right positive)
      ez: distance error (positive => too far) based on desired bbox height fraction
    """
    x1, y1, x2, y2 = bbox
    cx = 0.5*(x1 + x2)
    h  = (y2 - y1)
    ex = (cx - (W/2)) / (W/2 + 1e-6)
    desired_h = target_h_frac * H
    ez = (desired_h - h) / (desired_h + 1e-6)
    return float(ex), float(ez)

def draw_bbox(frame: np.ndarray, det: Detection, ex: float, ez: float, fps: float):
    x1,y1,x2,y2 = det.xyxy
    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
    label = f"{det.cls_name} {det.conf:.2f} | ex={ex:+.2f} ez={ez:+.2f}"
    cv2.putText(frame, label, (x1, max(0,y1-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
    cv2.putText(frame, f"FPS: {fps:.1f}", (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)

# --- Main ---------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=str, required=True,
                    help="RTSP or MJPEG URL (e.g., rtsp://..., http://.../mjpg)")
    ap.add_argument("--model", type=str, default="yolo", choices=["yolo", "efficientdet"],
                    help="Detector backend")
    ap.add_argument("--model-name", type=str, default="yolov8n.pt",
                    help="YOLO: .pt name; EfficientDet: efficientdet_lite0/1/2")
    ap.add_argument("--device", type=str, default="cpu",
                    help="YOLO device hint: cpu, mps, cuda (if available)")
    ap.add_argument("--conf", type=float, default=0.4, help="Confidence threshold")
    ap.add_argument("--target-h-frac", type=float, default=0.45,
                    help="Desired bbox height as fraction of frame height (for future distance control)")
    ap.add_argument("--pick", type=str, default="largest", choices=["largest", "conf"],
                    help="How to pick one target among multiple persons")
    ap.add_argument("--show", action="store_true", help="Show UI window")
    args = ap.parse_args()

    # Init detector
    if args.model == "yolo":
        det = YoloDetector(model_name=args.model_name, device=args.device, conf_thres=args.conf)
    else:
        det = EfficientDetDetector(model_name=args.model_name, conf_thres=args.conf)

    # Open stream (OpenCV handles RTSP/MJPEG)
    # Note: You can hint FFMPEG with CAP_FFMPEG if needed.
    cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source: {args.source}")

    prev_t = time.time()
    fps = 0.0

    # Main loop
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            # Try a brief wait/retry
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
            continue

        H, W = frame.shape[:2]

        # Detect
        detections = det.detect(frame)

        # Pick your subject
        target = pick_target(detections, strategy=args.pick)

        # Draw & compute control-friendly values
        if target is not None:
            ex, ez = compute_errors(target.xyxy, W, H, args.target_h_frac)
            # In the future, you'd feed (ex, ez) to your controller → (v, omega)
            draw_bbox(frame, target, ex, ez, fps)
        else:
            cv2.putText(frame, f"FPS: {fps:.1f}", (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)
            cv2.putText(frame, "No person detected", (8,46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)

        if args.show:
            cv2.imshow("LeafBot Vision", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # FPS calc
        now = time.time()
        dt = now - prev_t
        if dt > 0:
            fps = 1.0 / dt
        prev_t = now

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()