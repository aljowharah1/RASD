#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Header

import cv2
from ultralytics import YOLO
import numpy as np
import torch
from datetime import datetime
from typing import List, Dict


class RASDCameraNode(Node):
    def __init__(self):
        super().__init__("rasd_camera_node")

        # ------------------------------------------
        # LOAD YOLO MODEL (CUDA)
        # ------------------------------------------
        self.model = YOLO("/home/rasd/best.pt")

        if torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"

        self.model.to(self.device)
        self.get_logger().info(f"YOLO model loaded on {self.device}")

        # ------------------------------------------
        # CAMERA: 1080p @ 30 FPS
        # ------------------------------------------
        self.gst = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=1920,height=1080,framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true max-buffers=1"
        )

        self.cap = cv2.VideoCapture(self.gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("❌ Could not open camera")

        self.width  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps    = int(self.cap.get(cv2.CAP_PROP_FPS) or 30)

        # ------------------------------------------
        # VIDEO RECORDING
        # ------------------------------------------
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = f"/home/rasd/camera_record_{timestamp}.mp4"

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, (self.width, self.height))

        self.get_logger().info(f"🎥 Recording to: {self.video_path}")

        # ------------------------------------------
        # ROI FOR ZOOM (road only)
        # Change these to adjust zoom
        # ------------------------------------------
        self.roi_x_min = 0.25   # Crop left 25%
        self.roi_x_max = 0.75   # Crop right 25%
        self.roi_y_min = 0.40   # Crop upper 40%
        self.roi_y_max = 1.00   # Keep bottom 60%

        # ------------------------------------------
        # TEMPORAL FUSION (EMA)
        # ------------------------------------------
        self.prev_boxes: List[Dict] = []
        self.ALPHA = 0.5          # smoothing factor
        self.IOU_THR = 0.4        # matching threshold
        self.CLASS_NAMES = {0: "speed_bump", 1: "pothole"}

        # ------------------------------------------
        # ROS2 publishers
        # ------------------------------------------
        self.pub_det = self.create_publisher(String, "/camera/detections", 10)
        self.pub_conf = self.create_publisher(Float32, "/camera/confidence", 10)
        self.pub_stamp = self.create_publisher(Header, "/camera/detections_stamp", 10)

        # ------------------------------------------
        # TIMER LOOP
        # ------------------------------------------
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)



    # ---------------------------------------------------------------
    # IOU helper
    # ---------------------------------------------------------------
    def iou(self, a, b):
        x1 = max(a[0], b[0])
        y1 = max(a[1], b[1])
        x2 = min(a[2], b[2])
        y2 = min(a[3], b[3])

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (a[2] - a[0]) * (a[3] - a[1])
        area_b = (b[2] - b[0]) * (b[3] - b[1])

        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0



    # ---------------------------------------------------------------
    # MAIN LOOP
    # ---------------------------------------------------------------
    def process_frame(self):

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Camera read failed")
            return

        h, w, _ = frame.shape

        # ------------------------------
        # ROI CROP (ZOOM ROAD)
        # ------------------------------
        x0 = int(w * self.roi_x_min)
        x1 = int(w * self.roi_x_max)
        y0 = int(h * self.roi_y_min)
        y1 = int(h * self.roi_y_max)

        roi = frame[y0:y1, x0:x1]

        # ------------------------------
        # YOLO INFERENCE
        # ------------------------------
        results = self.model.predict(roi, device=self.device, verbose=False)[0]

        if len(results.boxes) > 0:
            boxes = results.boxes.xyxy.cpu().numpy()
            confs = results.boxes.conf.cpu().numpy()
            clss  = results.boxes.cls.cpu().numpy().astype(int)
        else:
            boxes = np.zeros((0, 4))
            confs = np.zeros((0,))
            clss  = np.zeros((0,), dtype=int)

        # ------------------------------------------
        # TEMPORAL FUSION (EMA)
        # ------------------------------------------
        new_boxes = []
        used = set()

        for i in range(len(boxes)):
            box_i = boxes[i]
            conf_i = float(confs[i])
            cls_i  = int(clss[i])

            best = -1
            best_iou = 0

            for j, prev in enumerate(self.prev_boxes):
                if prev["cls"] != cls_i or j in used:
                    continue

                iou = self.iou(box_i, prev["box"])
                if iou > best_iou:
                    best_iou = iou
                    best = j

            if best >= 0 and best_iou >= self.IOU_THR:
                prev = self.prev_boxes[best]
                smooth = self.ALPHA * conf_i + (1 - self.ALPHA) * prev["conf"]
                used.add(best)
            else:
                smooth = conf_i

            new_boxes.append({
                "box": box_i.copy(),
                "conf": smooth,
                "cls": cls_i
            })

        self.prev_boxes = new_boxes

        # ------------------------------------------
        # FIND BEST DETECTION
        # ------------------------------------------
        best_conf = 0.0
        best_det  = None

        for det in self.prev_boxes:
            if det["conf"] > best_conf:
                best_conf = det["conf"]
                best_det  = det

        # ------------------------------------------
        # DRAW + RECORD VIDEO
        # ------------------------------------------
        vis = frame.copy()

        # draw crop area
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0,255,255), 2)

        # draw detections
        for det in self.prev_boxes:
            bx = det["box"]
            cls = det["cls"]
            cnf = det["conf"]

            # convert to full-frame coords
            xA = int(bx[0] + x0)
            yA = int(bx[1] + y0)
            xB = int(bx[2] + x0)
            yB = int(bx[3] + y0)

            label = f"{self.CLASS_NAMES.get(cls,'obj')} {cnf:.2f}"
            cv2.rectangle(vis, (xA, yA), (xB, yB), (0,255,0), 2)
            cv2.putText(vis, label, (xA, yA-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        self.writer.write(vis)

        # ------------------------------------------
        # PRINT + PUBLISH DETECTION
        # ------------------------------------------
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        self.pub_stamp.publish(header)

        self.pub_conf.publish(Float32(data=float(best_conf)))

        if best_det:
            cls_name = self.CLASS_NAMES.get(best_det["cls"], "unknown")
            conf_val = best_det["conf"]
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            msg = f"class={cls_name}, conf={conf_val:.2f}"
            self.pub_det.publish(String(data=msg))

            print(f"[CAM] time={ts} {msg}")



def main(args=None):
    rclpy.init(args=args)
    node = RASDCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.writer.release()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

