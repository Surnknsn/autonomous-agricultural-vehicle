#!/usr/bin/env python3
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import cv2
    import numpy as np
except Exception:  # pragma: no cover - handled at runtime on robot
    cv2 = None
    np = None

try:
    import requests
except Exception:  # pragma: no cover
    requests = None


class CameraLaneAssistNode(Node):
    """Lightweight orchard lane detector.

    Publishes a small steering bias, never motor commands. Positive bias means
    "turn left"; negative means "turn right". The mower controller treats this
    as a soft assist and ignores it when stale or low-confidence.
    """

    def __init__(self):
        super().__init__("camera_lane_assist_node")
        self.enabled = os.getenv("CAMERA_LANE_ASSIST_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.mode = os.getenv("CAMERA_LANE_MODE", "snapshot").strip().lower()  # snapshot|usb
        self.snapshot_url = os.getenv("CAMERA_LANE_SNAPSHOT_URL", "http://127.0.0.1:8080/video_frame.jpg?fast=1")
        self.device = os.getenv("CAMERA_LANE_DEVICE", "/dev/video0")
        self.width = int(os.getenv("CAMERA_LANE_WIDTH", "416"))
        self.height = int(os.getenv("CAMERA_LANE_HEIGHT", "240"))
        self.fps = max(1.0, min(12.0, float(os.getenv("CAMERA_LANE_FPS", "7.0"))))
        self.max_bias_deg = float(os.getenv("CAMERA_LANE_MAX_BIAS_DEG", "14.0"))
        self.offset_gain_deg = float(os.getenv("CAMERA_LANE_OFFSET_GAIN_DEG", "10.0"))
        self.angle_gain = float(os.getenv("CAMERA_LANE_ANGLE_GAIN", "0.55"))
        self.min_confidence = float(os.getenv("CAMERA_LANE_MIN_CONFIDENCE", "0.25"))
        self.roi_top_frac = float(os.getenv("CAMERA_LANE_ROI_TOP_FRAC", "0.34"))
        self.roi_bottom_frac = float(os.getenv("CAMERA_LANE_ROI_BOTTOM_FRAC", "0.98"))
        self.debug_log_s = float(os.getenv("CAMERA_LANE_DEBUG_LOG_S", "1.5"))
        self._last_debug_ts = 0.0
        self._cap = None
        self._http = requests.Session() if requests is not None else None
        self.pub = self.create_publisher(String, "/camera_lane_assist", 10)
        self.status_pub = self.create_publisher(String, "/camera_lane_status", 10)
        self.timer = self.create_timer(1.0 / self.fps, self.tick)
        self.get_logger().info(
            f"Camera lane assist ready enabled={self.enabled} mode={self.mode} fps={self.fps:.1f}"
        )

    def _publish(self, payload):
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub.publish(msg)

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _read_snapshot(self):
        if self._http is None or cv2 is None or np is None:
            return None
        try:
            r = self._http.get(self.snapshot_url, timeout=(0.12, 0.28), stream=False)
            if r.status_code != 200 or not r.content:
                return None
            arr = np.frombuffer(r.content, dtype=np.uint8)
            if arr.size == 0:
                return None
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return None

    def _open_usb(self):
        if cv2 is None:
            return None
        if self._cap is not None and self._cap.isOpened():
            return self._cap
        dev = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass
        self._cap = cap
        return cap

    def _read_usb(self):
        cap = self._open_usb()
        if cap is None:
            return None
        ok, frame = cap.read()
        if not ok or frame is None or frame.size == 0:
            try:
                cap.release()
            except Exception:
                pass
            self._cap = None
            return None
        return frame

    def _read_frame(self):
        if self.mode == "usb":
            return self._read_usb()
        return self._read_snapshot()

    def _detect_lane(self, frame):
        if cv2 is None or np is None or frame is None or frame.size == 0:
            return None
        h, w = frame.shape[:2]
        if w > self.width > 0:
            nh = max(120, int(h * (self.width / float(w))))
            frame = cv2.resize(frame, (self.width, nh), interpolation=cv2.INTER_AREA)
            h, w = frame.shape[:2]

        y0 = int(max(0.0, min(0.9, self.roi_top_frac)) * h)
        y1 = int(max(self.roi_top_frac + 0.05, min(1.0, self.roi_bottom_frac)) * h)
        roi = frame[y0:y1, :]
        if roi.size == 0:
            return None
        rh, rw = roi.shape[:2]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Orchard rows usually appear as dark/brown vertical trunks and green
        # vegetation boundaries. This mask intentionally stays generic so it is
        # cheap and does not need training to start producing a steering hint.
        dark = cv2.inRange(gray, 0, 95)
        brown = cv2.inRange(hsv, (5, 35, 25), (35, 220, 180))
        green = cv2.inRange(hsv, (35, 35, 30), (90, 255, 230))
        mask = cv2.bitwise_or(dark, brown)
        mask = cv2.bitwise_or(mask, green)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)

        # Estimate safe lane center from left/right obstacle density in the
        # lower image. The closer bottom rows carry more weight.
        rows = np.linspace(0.15, 1.0, rh, dtype=np.float32)[:, None]
        col_density = (mask.astype(np.float32) / 255.0 * rows).sum(axis=0)
        if float(col_density.max(initial=0.0)) > 0.0:
            col_density = col_density / max(1e-6, float(col_density.max()))
        margin = max(8, int(0.06 * rw))
        left_density = col_density[: rw // 2]
        right_density = col_density[rw // 2 :]
        left_peak = int(np.argmax(left_density)) if left_density.size else margin
        right_peak = int(np.argmax(right_density)) + rw // 2 if right_density.size else rw - margin
        left_strength = float(left_density[left_peak]) if left_density.size else 0.0
        right_strength = float(col_density[right_peak]) if right_density.size else 0.0

        if left_strength > 0.16 and right_strength > 0.16 and right_peak > left_peak + margin:
            lane_center_x = 0.5 * (left_peak + right_peak)
            center_conf = min(1.0, 0.45 + 0.55 * min(left_strength, right_strength))
        else:
            # If only one tree row is visible, move away from the denser side.
            side_balance = right_strength - left_strength
            lane_center_x = rw * (0.5 - 0.28 * side_balance)
            center_conf = min(0.55, max(left_strength, right_strength))

        # Cheap orientation estimate: Hough lines on the masked edges.
        edges = cv2.Canny(mask, 60, 150)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180.0,
            threshold=28,
            minLineLength=max(24, int(0.18 * rh)),
            maxLineGap=16,
        )
        angle_samples = []
        if lines is not None:
            for line in lines[:, 0, :]:
                x1, y1, x2, y2 = [float(v) for v in line]
                dx = x2 - x1
                dy = y2 - y1
                length = math.hypot(dx, dy)
                if length < 20.0 or abs(dy) < 8.0:
                    continue
                # angle relative to image vertical; positive means the lane
                # points left in robot frame.
                angle = math.atan2(dx, max(1e-6, abs(dy)))
                if abs(angle) <= math.radians(35.0):
                    angle_samples.append((angle, length))
        if angle_samples:
            total_w = sum(wgt for _, wgt in angle_samples)
            angle_rad = sum(a * wgt for a, wgt in angle_samples) / max(1e-6, total_w)
            angle_conf = min(1.0, total_w / max(80.0, 0.8 * rh))
        else:
            angle_rad = 0.0
            angle_conf = 0.0

        offset_norm = (lane_center_x - (rw * 0.5)) / max(1.0, rw * 0.5)
        # Positive bias = turn left. If lane center is to the right, turn right.
        bias_rad = (-offset_norm * math.radians(self.offset_gain_deg)) + (angle_rad * self.angle_gain)
        max_bias = math.radians(max(1.0, self.max_bias_deg))
        bias_rad = max(-max_bias, min(max_bias, bias_rad))

        confidence = max(0.0, min(1.0, 0.65 * center_conf + 0.35 * angle_conf))
        if confidence < self.min_confidence:
            bias_rad = 0.0

        return {
            "stamp": time.time(),
            "ok": confidence >= self.min_confidence,
            "bias_rad": float(bias_rad),
            "bias_deg": float(math.degrees(bias_rad)),
            "confidence": float(confidence),
            "offset_norm": float(offset_norm),
            "angle_rad": float(angle_rad),
            "angle_deg": float(math.degrees(angle_rad)),
            "left_strength": float(left_strength),
            "right_strength": float(right_strength),
            "width": int(w),
            "height": int(h),
            "source": self.mode,
        }

    def tick(self):
        if not self.enabled:
            self._publish({"stamp": time.time(), "ok": False, "bias_rad": 0.0, "confidence": 0.0, "source": "disabled"})
            return
        if cv2 is None or np is None:
            self._publish_status("CAMERA_LANE missing cv2/numpy")
            self._publish({"stamp": time.time(), "ok": False, "bias_rad": 0.0, "confidence": 0.0, "source": "no_cv"})
            return

        frame = self._read_frame()
        if frame is None:
            self._publish_status("CAMERA_LANE frame_fail")
            self._publish({"stamp": time.time(), "ok": False, "bias_rad": 0.0, "confidence": 0.0, "source": "frame_fail"})
            return

        result = self._detect_lane(frame)
        if result is None:
            self._publish({"stamp": time.time(), "ok": False, "bias_rad": 0.0, "confidence": 0.0, "source": "detect_fail"})
            return
        self._publish(result)
        now = time.time()
        if now - self._last_debug_ts >= self.debug_log_s:
            self._last_debug_ts = now
            self.get_logger().info(
                f"lane ok={result['ok']} conf={result['confidence']:.2f} "
                f"bias={result['bias_deg']:+.1f}deg off={result['offset_norm']:+.2f} "
                f"row=({result['left_strength']:.2f},{result['right_strength']:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraLaneAssistNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            if node._cap is not None:
                node._cap.release()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
