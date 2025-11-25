#!/usr/bin/env python3
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String


class RASDFusionNode(Node):
    def __init__(self):
        super().__init__("rasd_fusion_node")

        # Sliding-window for LiDAR stability
        self.lidar_window = deque(maxlen=5)

        # Camera state
        self.last_camera_conf = None
        self.last_camera_det = None

        # LED temporal smoothing
        self.decision_window = deque(maxlen=5)
        self.current_led = "OFF"

        # Subscribers
        self.create_subscription(Float32MultiArray, "/rasd/lidar_bump",
                                 self.cb_lidar, 10)
        self.create_subscription(Float32, "/camera/confidence",
                                 self.cb_cam_conf, 10)
        self.create_subscription(String, "/camera/detections",
                                 self.cb_cam_det, 10)

        # Publisher
        self.pub_led = self.create_publisher(String, "/rasd/led_command", 10)

        # Timer 20 Hz
        self.timer = self.create_timer(0.05, self.compute_fusion)

        self.get_logger().info("✅ Fusion Node READY — OFF is default, no temporal needed")


    # ------------------------- Callbacks -------------------------
    def cb_lidar(self, msg):
        if len(msg.data) != 3:
            return

        self.lidar_window.append({
            "x": float(msg.data[0]),
            "conf": float(msg.data[1]),
            "height": float(msg.data[2])
        })


    def cb_cam_conf(self, msg):
        self.last_camera_conf = float(msg.data)


    def cb_cam_det(self, msg):
        self.last_camera_det = msg.data


    # ---------------------- Fusion Logic ------------------------
    def compute_fusion(self):

        # If no LiDAR → OFF immediately
        if len(self.lidar_window) == 0:
            self.set_led_off("No LiDAR window")
            return

        # Aggregate LiDAR
        dists = [x["x"] for x in self.lidar_window]
        confs = [x["conf"] for x in self.lidar_window]
        lidar_dist = min(dists)
        lidar_conf = max(confs)

        # Camera flags
        cam_detect = (self.last_camera_det and "speed_bump" in self.last_camera_det)
        cam_conf = self.last_camera_conf or 0.0

        # ---------------- RAW (per-frame) RULE LOGIC ----------------

        # Rule 1: camera alone strong
        if cam_detect and cam_conf >= 0.85 and lidar_conf < 0.75:
            self.push_temporal("RED", "Camera-alone high conf")
            return

        # Rule 2: LiDAR strong & close
        if lidar_conf >= 0.75 and lidar_dist <= 12:
            self.push_temporal("RED", "LiDAR strong & close")
            return

        # Rule 3: Both LiDAR + camera
        if cam_detect and lidar_dist <= 20:
            self.push_temporal("RED", "LiDAR+Camera confirm bump")
            return

        # Rule 4: Medium → yellow
        if lidar_conf >= 0.50 or lidar_dist <= 25:
            self.push_temporal("YELLOW", "LiDAR medium/conf")
            return

        # Rule 5: Far → green
        if lidar_dist > 25 and lidar_conf < 0.50:
            self.push_temporal("GREEN", "LiDAR far/weak")
            return

        # DEFAULT → immediate OFF
        self.set_led_off("No conditions matched")


    # ---------------------- Temporal Smoothing --------------------
    def push_temporal(self, raw_decision, reason):

        # For OFF → immediate
        if raw_decision == "OFF":
            self.set_led_off(reason)
            return

        # Add to temporal buffer
        self.decision_window.append(raw_decision)

        # Need 3 confirmations
        REQUIRED = 3

        # Count
        if self.decision_window.count(raw_decision) >= REQUIRED:
            if self.current_led != raw_decision:
                self.current_led = raw_decision
                self.pub_led.publish(String(data=raw_decision))
                self.get_logger().info(f"🚦 LED = {raw_decision}   ({reason})")
        # else: keep previous LED (avoid flicker)


    # ---------------------- Immediate OFF ------------------------
    def set_led_off(self, reason):
        if self.current_led != "OFF":
            self.current_led = "OFF"
            self.pub_led.publish(String(data="OFF"))
            self.get_logger().info(f"🚦 LED = OFF   ({reason})")
        # Clear window so next detection is fresh
        self.decision_window.clear()


def main(args=None):
    rclpy.init(args=args)
    node = RASDFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()


