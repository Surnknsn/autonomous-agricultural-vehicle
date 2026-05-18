#!/usr/bin/env python3
"""
follow_tracker_node.py — โหนด ROS2 สำหรับโหมด FOLLOW (ตามคน)

ภาพรวมระบบ:
- ใช้กล้องวิดีโอตรวจจับและติดตามคนที่ต้องการ (owner/เจ้าของ)
- รองรับ detector 2 แบบ: YOLO (แนะนำ) และ MediaPipe pose
- ระบุตัวเจ้าของด้วย ReID model (OSNet ONNX หรือ MobileNet) เทียบกับ profile image ที่บันทึกไว้
- ใช้ LIDAR ร่วมกับ vision เพื่อประมาณระยะทางที่แม่นยำกว่า
- ส่งคำสั่ง FORWARD/BACK/LEFT/RIGHT/STOP ไปยัง lawnmower_node ผ่าน /follow_cmd
- มีระบบ safety: หยุดเมื่อ LIDAR ตรวจพบสิ่งกีดขวาง (multi-sector check)
- รองรับ edge recovery: หมุนเพื่อให้เจ้าของอยู่ในกรอบภาพ

ROS Topics ที่ใช้:
    Subscribe:
        /auto_control          — รับคำสั่ง FOLLOW_START / STOP จาก lawnmower_node
        /follow_target_profile — ชื่อ profile ของเจ้าของที่ต้องการติดตาม
        /scan                  — ข้อมูล LIDAR สำหรับ safety และ range estimation
        /lidar_safety_enable   — เปิด/ปิด LIDAR safety
        /odom                  — yaw rate จาก odometry สำหรับ steering feedback

    Publish:
        /follow_cmd            — คำสั่งขับเคลื่อน (String)
        /follow_vision_status  — สถานะการมองเห็น/ติดตาม (String)
        /follow_debug          — ข้อมูล debug (JSON String)

Environment Variables หลัก:
    FOLLOW_CAMERA_DEVICE   — path ของกล้อง (default /dev/video0)
    FOLLOW_YOLO_MODEL      — path ของ YOLO model file
    FOLLOW_BASE_PWM        — PWM พื้นฐานสำหรับเดินหน้า
    FOLLOW_LIDAR_STOP_M    — ระยะ LIDAR ที่จะหยุด (เมตร)
"""
import math
import os
import time
import glob
import json
import requests
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool

try:
    import onnxruntime as ort
    ONNX_REID_OK = True
except Exception:
    ort = None
    ONNX_REID_OK = False

try:
    import torch
    import torch.nn.functional as torch_f
    from PIL import Image
    from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small
    REID_OK = True
except Exception:
    torch = None
    torch_f = None
    Image = None
    MobileNet_V3_Small_Weights = None
    mobilenet_v3_small = None
    REID_OK = False

try:
    import mediapipe.python.solutions.pose as mp_pose
    MEDIAPIPE_OK = True
except Exception:
    MEDIAPIPE_OK = False
    mp_pose = None

try:
    from ultralytics import YOLO
    YOLO_OK = True
except Exception:
    YOLO_OK = False
    YOLO = None


class FollowTrackerNode(Node):
    """
    โหนดติดตามคน (Follow Tracker) สำหรับโหมด FOLLOW ของหุ่นยนต์ตัดหญ้า

    ทำงาน ~15 Hz ผ่าน timer โดย:
    1. อ่านเฟรมจากกล้อง
    2. ตรวจจับคนด้วย YOLO หรือ MediaPipe
    3. ระบุตัวเจ้าของด้วย ReID model เทียบกับ profile
    4. คำนวณคำสั่งขับเคลื่อน (ซ้าย/ขวา/หน้า/หลัง/หยุด)
    5. ตรวจสอบ LIDAR safety ก่อนส่งคำสั่ง
    6. Publish คำสั่งไปยัง /follow_cmd
    """

    def __init__(self):
        super().__init__("follow_tracker_node")
        # Reduce OpenCV backend noise in console.
        os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
        try:
            cv2.setLogLevel(2)  # ERROR
        except Exception:
            pass

        self.active = False
        self.last_cmd = "STOP"
        self.last_cmd_ts = 0.0
        self.cmd_repeat_s = float(os.getenv("FOLLOW_CMD_REPEAT_S", "0.08"))
        self.lost_timeout_s = float(os.getenv("FOLLOW_LOST_TIMEOUT_S", "1.8"))
        # Hold the last valid follow PWM very briefly across camera frame drops.
        # Long blind drive is unsafe, but zero hold makes the mower stutter.
        self.follow_blind_hold_s = float(os.getenv("FOLLOW_BLIND_HOLD_S", "0.10"))
        self.follow_search_on_lost = os.getenv("FOLLOW_SEARCH_ON_LOST", "1").lower() in ("1", "true", "yes", "on")
        self.follow_search_hold_s = float(os.getenv("FOLLOW_SEARCH_HOLD_S", "1.20"))
        self.follow_search_pwm = int(float(os.getenv("FOLLOW_SEARCH_PWM", "100")))
        self.follow_dim_gamma = float(os.getenv("FOLLOW_DIM_GAMMA", "1.35"))
        self.follow_auto_contrast = os.getenv("FOLLOW_AUTO_CONTRAST", "1").lower() in ("1", "true", "yes", "on")
        self.follow_auto_contrast_clip = float(os.getenv("FOLLOW_AUTO_CONTRAST_CLIP", "2.2"))
        self.follow_auto_contrast_blend = float(os.getenv("FOLLOW_AUTO_CONTRAST_BLEND", "0.72"))
        self.follow_min_visibility = float(os.getenv("FOLLOW_MIN_VISIBILITY", "0.38"))
        self.follow_human_min_visibility = float(os.getenv("FOLLOW_HUMAN_MIN_VISIBILITY", "0.32"))
        self.follow_strict_human_pose = os.getenv("FOLLOW_STRICT_HUMAN_POSE", "0").lower() in ("1", "true", "yes", "on")
        self.follow_low_light_gain = float(os.getenv("FOLLOW_LOW_LIGHT_GAIN", "1.25"))
        self.follow_low_light_beta = float(os.getenv("FOLLOW_LOW_LIGHT_BETA", "18.0"))
        self.last_seen_ts = 0.0
        self.last_person_seen_ts = 0.0
        self.last_drive_cmd = "STOP"
        self.last_drive_status = ""
        self.last_drive_seen_ts = 0.0

        self.deadzone_px = int(os.getenv("FOLLOW_DEADZONE_PX", "18"))
        self.target_shoulder_px = float(os.getenv("FOLLOW_TARGET_SHOULDER_PX", "210"))
        self.shoulder_tol_px = float(os.getenv("FOLLOW_SHOULDER_TOL_PX", "35"))
        self.follow_base_pwm = int(float(os.getenv("FOLLOW_BASE_PWM", "110")))
        self.follow_near_pwm = int(float(os.getenv("FOLLOW_NEAR_PWM", "80")))
        self.follow_far_pwm = int(float(os.getenv("FOLLOW_FAR_PWM", str(self.follow_base_pwm))))
        self.follow_max_pwm = int(float(os.getenv("FOLLOW_MAX_PWM", "140")))
        self.follow_turn_pwm = int(float(os.getenv("FOLLOW_TURN_PWM", "145")))
        self.follow_min_move_pwm = int(float(os.getenv("FOLLOW_MIN_MOVE_PWM", "70")))
        self.follow_center_deadzone_norm = float(os.getenv("FOLLOW_CENTER_DEADZONE_NORM", "0.025"))
        self.follow_steer_zones = max(0, int(os.getenv("FOLLOW_STEER_ZONES", "41")))
        self.follow_turn_exp = float(os.getenv("FOLLOW_TURN_EXP", "0.72"))
        self.follow_min_steer_pwm = float(os.getenv("FOLLOW_MIN_STEER_PWM", "62"))
        self.follow_min_drive_ratio = float(os.getenv("FOLLOW_MIN_DRIVE_RATIO", "0.06"))
        self.follow_steer_gain = float(os.getenv("FOLLOW_STEER_GAIN", "80.0"))
        self.follow_arc_turn_enable = os.getenv("FOLLOW_ARC_TURN_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.follow_arc_outer_pwm = int(float(os.getenv("FOLLOW_ARC_OUTER_PWM", "180")))
        self.follow_arc_inner_min_pwm = int(float(os.getenv("FOLLOW_ARC_INNER_MIN_PWM", "30")))
        self.follow_arc_inner_ratio_min = float(os.getenv("FOLLOW_ARC_INNER_RATIO_MIN", "0.08"))
        self.follow_arc_base_min_pwm = int(float(os.getenv("FOLLOW_ARC_BASE_MIN_PWM", "75")))
        self.stop_dist_m = float(os.getenv("FOLLOW_LIDAR_STOP_M", "2.0"))
        self.follow_lidar_front_sector_deg = float(os.getenv("FOLLOW_LIDAR_FRONT_SECTOR_DEG", "240.0"))
        self.follow_lidar_sector_count = max(5, int(os.getenv("FOLLOW_LIDAR_SECTOR_COUNT", "9")))
        self.follow_lidar_stop_confirm_s = float(os.getenv("FOLLOW_LIDAR_STOP_CONFIRM_S", "0.08"))
        self.follow_lidar_stop_sectors = max(1, int(os.getenv("FOLLOW_LIDAR_STOP_SECTORS", "1")))
        self.follow_lidar_hard_stop_m = float(os.getenv("FOLLOW_LIDAR_HARD_STOP_M", "1.05"))
        # Absolute all-front-sector stop is only for very close physical contact.
        # Normal follow safety below is corridor based; this prevents side dust
        # or a single close speck from freezing the tracker while a person is
        # correctly detected ahead.
        self.follow_lidar_any_hard_stop_m = float(os.getenv("FOLLOW_LIDAR_ANY_HARD_STOP_M", "0.38"))
        self.follow_lidar_cluster_min_points = max(1, int(os.getenv("FOLLOW_LIDAR_CLUSTER_MIN_POINTS", "3")))
        self.follow_lidar_cluster_min_width_m = float(os.getenv("FOLLOW_LIDAR_CLUSTER_MIN_WIDTH_M", "0.055"))
        self.follow_lidar_cluster_max_gap = max(1, int(os.getenv("FOLLOW_LIDAR_CLUSTER_MAX_GAP", "2")))
        self.follow_lidar_cluster_max_jump_m = float(os.getenv("FOLLOW_LIDAR_CLUSTER_MAX_JUMP_M", "0.45"))
        self.follow_lidar_blob_enable = os.getenv("FOLLOW_LIDAR_BLOB_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.follow_lidar_blob_max_m = float(os.getenv("FOLLOW_LIDAR_BLOB_MAX_M", "4.2"))
        self.follow_lidar_blob_min_width_m = float(os.getenv("FOLLOW_LIDAR_BLOB_MIN_WIDTH_M", "0.12"))
        self.follow_lidar_blob_weight_close_m = float(os.getenv("FOLLOW_LIDAR_BLOB_WEIGHT_CLOSE_M", "3.2"))
        self.follow_safety_corridor_sectors = max(0, int(os.getenv("FOLLOW_SAFETY_CORRIDOR_SECTORS", "1")))
        # The followed human is also a LiDAR return. Ignore a wider corridor around
        # the camera/LiDAR-associated target so the mower does not stop on its owner.
        self.follow_lidar_ignore_target_sectors = max(0, int(os.getenv("FOLLOW_LIDAR_IGNORE_TARGET_SECTORS", "1")))
        self.follow_target_lidar_window = max(0, int(os.getenv("FOLLOW_TARGET_LIDAR_WINDOW", "3")))
        self.follow_target_cruise_m = float(os.getenv("FOLLOW_TARGET_CRUISE_M", "3.4"))
        self.follow_target_min_m = float(os.getenv("FOLLOW_TARGET_MIN_M", "0.75"))
        # From your close calibration sample: camera=1.35m -> LiDAR ~=1.435m.
        # Treat this as the comfortable hold distance; closer than this must not
        # keep driving forward.
        self.follow_target_ideal_m = float(os.getenv("FOLLOW_TARGET_IDEAL_M", "1.80"))
        self.follow_close_stop_m = float(os.getenv("FOLLOW_CLOSE_STOP_M", "1.55"))
        self.follow_close_backoff_m = float(os.getenv("FOLLOW_CLOSE_BACKOFF_M", "1.65"))
        self.follow_distance_hold_band_m = float(os.getenv("FOLLOW_DISTANCE_HOLD_BAND_M", "0.22"))
        self.follow_distance_kp_pwm = float(os.getenv("FOLLOW_DISTANCE_KP_PWM", "125.0"))
        self.follow_distance_kd_pwm = float(os.getenv("FOLLOW_DISTANCE_KD_PWM", "42.0"))
        self.follow_distance_alpha = float(os.getenv("FOLLOW_DISTANCE_ALPHA", "0.45"))
        self.follow_distance_rate_alpha = float(os.getenv("FOLLOW_DISTANCE_RATE_ALPHA", "0.55"))
        self.follow_slow_band_near_m = float(os.getenv("FOLLOW_SLOW_BAND_NEAR_M", "2.60"))
        self.follow_slow_band_mid_m = float(os.getenv("FOLLOW_SLOW_BAND_MID_M", "3.20"))
        self.follow_slow_band_far_m = float(os.getenv("FOLLOW_SLOW_BAND_FAR_M", "4.00"))
        self.follow_slow_pwm_near = float(os.getenv("FOLLOW_SLOW_PWM_NEAR", "65"))
        self.follow_slow_pwm_mid = float(os.getenv("FOLLOW_SLOW_PWM_MID", "90"))
        self.follow_slow_pwm_far = float(os.getenv("FOLLOW_SLOW_PWM_FAR", "110"))
        self.follow_camera_to_lidar_offset_m = float(os.getenv("FOLLOW_CAMERA_TO_LIDAR_OFFSET_M", "0.085"))
        # Calibrated from measured samples:
        # sh=122 -> camera distance 1.95m, sh=193 -> camera distance 1.35m.
        # camera_distance_m = A / sh + B
        self.follow_camera_dist_a = float(os.getenv("FOLLOW_CAMERA_DIST_A", "198.98"))
        self.follow_camera_dist_b = float(os.getenv("FOLLOW_CAMERA_DIST_B", "0.319"))
        self.follow_back_pwm = int(float(os.getenv("FOLLOW_BACK_PWM", "82")))
        self.follow_allow_backoff = os.getenv("FOLLOW_ALLOW_BACKOFF", "1").lower() in ("1", "true", "yes", "on")
        self.follow_cross_block_m = float(os.getenv("FOLLOW_CROSS_BLOCK_M", "0.55"))
        self.follow_cross_hold_s = float(os.getenv("FOLLOW_CROSS_HOLD_S", "0.45"))
        self.follow_ex_alpha = float(os.getenv("FOLLOW_EX_ALPHA", "0.30"))
        self.follow_shoulder_alpha = float(os.getenv("FOLLOW_SHOULDER_ALPHA", "0.35"))
        self.follow_turn_in_place_norm = float(os.getenv("FOLLOW_TURN_IN_PLACE_NORM", "0.985"))
        # If the owner is close to the image edge, prioritize rotating to keep
        # them in-frame. This is intentionally stronger than normal steering.
        self.follow_edge_recovery_norm = float(os.getenv("FOLLOW_EDGE_RECOVERY_NORM", "0.36"))
        self.follow_edge_turn_pwm = int(float(os.getenv("FOLLOW_EDGE_TURN_PWM", "180")))
        self.follow_edge_arc_base_pwm = int(float(os.getenv("FOLLOW_EDGE_ARC_BASE_PWM", "0")))
        self.follow_edge_inner_ratio = float(os.getenv("FOLLOW_EDGE_INNER_RATIO", "0.0"))
        self.follow_pwm_slew_up_per_s = float(os.getenv("FOLLOW_PWM_SLEW_UP_PER_S", "420.0"))
        self.follow_pwm_slew_down_per_s = float(os.getenv("FOLLOW_PWM_SLEW_DOWN_PER_S", "620.0"))
        self._cmd_pwm_l = 0.0
        self._cmd_pwm_r = 0.0
        self._cmd_pwm_last_ts = 0.0
        # Yaw feedback for closed-loop steering boost
        self._yaw_cur = 0.0
        self._yaw_rate = 0.0
        self._yaw_ts = 0.0
        self._arc_steer_dir = 0
        self._arc_steer_since = 0.0
        self.follow_yaw_min_rate = float(os.getenv("FOLLOW_YAW_MIN_RATE", "0.10"))
        self.follow_yaw_boost_delay_s = float(os.getenv("FOLLOW_YAW_BOOST_DELAY_S", "0.20"))
        self.follow_yaw_boost_inner_scale = float(os.getenv("FOLLOW_YAW_BOOST_INNER_SCALE", "0.35"))
        self.follow_yaw_boost_outer_scale = float(os.getenv("FOLLOW_YAW_BOOST_OUTER_SCALE", "1.30"))
        self.follow_soft_stop_m = float(os.getenv("FOLLOW_LIDAR_SOFT_STOP_M", str(self.stop_dist_m + 0.25)))
        self.follow_camera_hfov_deg = float(os.getenv("FOLLOW_CAMERA_HFOV_DEG", "70.0"))
        self.follow_lidar_assoc_window_deg = float(os.getenv("FOLLOW_LIDAR_ASSOC_WINDOW_DEG", "28.0"))
        self.follow_lidar_bearing_weight = float(os.getenv("FOLLOW_LIDAR_BEARING_WEIGHT", "0.35"))
        self.follow_lidar_bearing_weight_close_max = float(os.getenv("FOLLOW_LIDAR_BEARING_WEIGHT_CLOSE_MAX", "0.65"))
        self.follow_lidar_range_weight = float(os.getenv("FOLLOW_LIDAR_RANGE_WEIGHT", "0.72"))
        self.follow_lidar_track_hold_s = float(os.getenv("FOLLOW_LIDAR_TRACK_HOLD_S", "0.75"))
        self.follow_lidar_vision_hold_enable = os.getenv("FOLLOW_LIDAR_VISION_HOLD_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.follow_lidar_vision_hold_s = float(os.getenv("FOLLOW_LIDAR_VISION_HOLD_S", "0.25"))
        # Reject LiDAR returns that are physically inconsistent with the camera
        # target size. This prevents a near pole/noise return from becoming
        # "target_lidar=0.6m" and freezing FOLLOW in HOLD_TARGET.
        self.follow_lidar_plausibility = os.getenv("FOLLOW_LIDAR_PLAUSIBILITY", "1").lower() in ("1", "true", "yes", "on")
        self.follow_lidar_plaus_min_ratio = float(os.getenv("FOLLOW_LIDAR_PLAUS_MIN_RATIO", "0.42"))
        self.follow_lidar_plaus_max_ratio = float(os.getenv("FOLLOW_LIDAR_PLAUS_MAX_RATIO", "2.70"))
        self.follow_profile_dir = Path(os.getenv("FOLLOW_PROFILE_DIR", str(Path.home() / "ros2_foxy_ws" / "web" / "dataset")))
        self.shared_ui_state_path = Path(os.getenv("FOLLOW_SHARED_UI_STATE", str(Path.home() / "ros2_foxy_ws" / "web" / "shared_ui_state.json")))
        # Strict owner mode: if the selected profile does not match, do not
        # silently follow another person.
        self.follow_profile_match_max = float(os.getenv("FOLLOW_PROFILE_MATCH_MAX", "0.50"))
        self.follow_profile_accept_hold_s = float(os.getenv("FOLLOW_PROFILE_ACCEPT_HOLD_S", "0.20"))
        self.follow_profile_lock_frames = max(1, int(os.getenv("FOLLOW_PROFILE_LOCK_FRAMES", "2")))
        self.follow_profile_loose_single_max = float(os.getenv("FOLLOW_PROFILE_LOOSE_SINGLE_MAX", "1.05"))
        self.follow_profile_loose_recent_max = float(os.getenv("FOLLOW_PROFILE_LOOSE_RECENT_MAX", "0.95"))
        self.follow_allow_loose_owner_match = os.getenv("FOLLOW_ALLOW_LOOSE_OWNER_MATCH", "0").lower() in ("1", "true", "yes", "on")
        self.follow_allow_weak_owner_match = os.getenv("FOLLOW_ALLOW_WEAK_OWNER_MATCH", "0").lower() in ("1", "true", "yes", "on")
        self.follow_allow_weak_owner_multi = os.getenv("FOLLOW_ALLOW_WEAK_OWNER_MULTI", "0").lower() in ("1", "true", "yes", "on")
        self.follow_owner_ambiguity_margin = float(os.getenv("FOLLOW_OWNER_AMBIGUITY_MARGIN", "0.12"))
        self.follow_reacquire_window_norm = float(os.getenv("FOLLOW_REACQUIRE_WINDOW_NORM", "0.30"))
        self.follow_reacquire_track_bonus = float(os.getenv("FOLLOW_REACQUIRE_TRACK_BONUS", "0.26"))
        self.follow_reacquire_continuity_weight = float(os.getenv("FOLLOW_REACQUIRE_CONTINUITY_WEIGHT", "0.36"))
        self.selected_profile = ""
        self.profile_refs = []
        self._profile_accept_until = 0.0
        self._profile_lock_count = 0
        self._last_profile_score = float("inf")
        self._last_profile_match_mode = "none"
        self.reid_enabled = os.getenv("FOLLOW_REID_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.reid_weight = float(os.getenv("FOLLOW_REID_WEIGHT", "0.58"))
        self.reid_score_scale = float(os.getenv("FOLLOW_REID_SCORE_SCALE", "1.85"))
        self.reid_model = None
        self.reid_preprocess = None
        self.reid_device = None
        self.reid_ready = False
        self.osnet_reid_enabled = os.getenv("FOLLOW_OSNET_REID_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.osnet_reid_path = Path(os.getenv(
            "FOLLOW_OSNET_REID_ONNX",
            str(Path.home() / "ros2_foxy_ws" / "models" / "reid" / "osnet_x0_25_msmt17_b1.onnx")
        ))
        self.osnet_reid_weight = float(os.getenv("FOLLOW_OSNET_REID_WEIGHT", "0.78"))
        self.osnet_reid_score_scale = float(os.getenv("FOLLOW_OSNET_REID_SCORE_SCALE", "1.00"))
        self.osnet_reid_session = None
        self.osnet_reid_input_name = None
        self.osnet_reid_ready = False
        self.lidar_safety_enabled = True

        self.lidar_min_front_m = float("inf")
        self.lidar_last_ts = 0.0
        self.lidar_sector_min = [float("inf")] * self.follow_lidar_sector_count
        self.lidar_sector_angle = [float("nan")] * self.follow_lidar_sector_count
        self.lidar_clusters = []
        self._lidar_stop_since = 0.0
        self._lidar_stop_sector_count = 0
        self._cross_block_until = 0.0
        self._ex_filt = None
        self._shoulder_filt = None
        self._target_bearing_rad = 0.0
        self._target_bearing_ts = 0.0
        self._target_lidar_angle = float("nan")
        self._target_lidar_range = float("inf")
        self._follow_range_filt = float("inf")
        self._follow_range_rate_mps = 0.0
        self._follow_range_prev = float("inf")
        self._follow_range_prev_ts = 0.0

        self.camera_device = os.getenv("FOLLOW_CAMERA_DEVICE", "/dev/video0")
        self.camera_candidates_env = os.getenv("FOLLOW_CAMERA_CANDIDATES", "").strip()
        self.camera_mode = os.getenv("FOLLOW_CAMERA_MODE", "auto").strip().lower()  # auto|snapshot|mjpeg|usb
        self.camera_mjpeg_url = os.getenv(
            "FOLLOW_CAMERA_MJPEG_URL",
            "http://127.0.0.1:8080/video_feed?stabilize=0&follow_ai=0"
        ).strip()
        self.camera_snapshot_url = os.getenv(
            "FOLLOW_CAMERA_SNAPSHOT_URL",
            "http://127.0.0.1:8080/video_frame.jpg?fast=1"
        ).strip()
        self.camera_width = int(os.getenv("FOLLOW_CAMERA_WIDTH", "640"))
        self.camera_height = int(os.getenv("FOLLOW_CAMERA_HEIGHT", "360"))
        self.follow_infer_width = int(os.getenv("FOLLOW_INFER_WIDTH", "320"))
        self.detector_mode = os.getenv("FOLLOW_DETECTOR", "yolo").strip().lower()  # auto|yolo|mediapipe
        self.yolo_model_path = os.getenv("FOLLOW_YOLO_MODEL", "/home/iai/foolme/yolo11n.pt").strip()
        self.yolo_conf = float(os.getenv("FOLLOW_YOLO_CONF", "0.34"))
        self.yolo_imgsz = int(os.getenv("FOLLOW_YOLO_IMGSZ", "416"))
        self.yolo_track = os.getenv("FOLLOW_YOLO_TRACK", "1").lower() in ("1", "true", "yes", "on")
        self.yolo_tracker_cfg = os.getenv("FOLLOW_YOLO_TRACKER", "bytetrack.yaml").strip()
        self.yolo_min_box_h = int(os.getenv("FOLLOW_YOLO_MIN_BOX_H", "70"))
        self.yolo_min_box_w = int(os.getenv("FOLLOW_YOLO_MIN_BOX_W", "28"))
        self.follow_target_bbox_width_px = float(os.getenv("FOLLOW_TARGET_BBOX_WIDTH_PX", "230"))
        self.follow_max_camera_target_m = float(os.getenv("FOLLOW_MAX_CAMERA_TARGET_M", "4.20"))
        self.follow_min_drive_apparent_px = float(os.getenv("FOLLOW_MIN_DRIVE_APPARENT_PX", "52.0"))
        self.follow_track_hold_s = float(os.getenv("FOLLOW_TRACK_HOLD_S", "1.25"))
        self.freeze_diff_thresh = float(os.getenv("FOLLOW_FREEZE_DIFF_THRESH", "0.04"))
        self.freeze_trigger_s = float(os.getenv("FOLLOW_FREEZE_TRIGGER_S", "2.5"))
        self.cap = None
        self._cam_source = None
        self._snapshot_fallback = False
        self._last_cam_open_try_ts = 0.0
        self._last_cam_err_log_ts = 0.0
        self._last_frame_ok_ts = 0.0
        self._prev_gray_small = None
        self._freeze_accum_s = 0.0
        self._last_frame_check_ts = 0.0
        self._last_idle_status_ts = 0.0
        self._last_target_bbox = None
        self._last_target_cx = None
        self._last_track_id = None
        self._last_target_ts = 0.0
        self._last_target_ex = 0.0
        self._target_cx_vel_px_s = 0.0
        self._last_target_memory_ts = 0.0
        self._last_people_count = 0
        self.follow_state = "IDLE"
        self._http = requests.Session()

        if not self._init_osnet_reid_model():
            self._init_reid_model()

        self.yolo = None
        if self.detector_mode in ("auto", "yolo"):
            if YOLO_OK:
                try:
                    model_path = self.yolo_model_path
                    if not model_path or not Path(model_path).exists():
                        model_path = "yolo11n.pt"
                    self.yolo = YOLO(model_path)
                    self.get_logger().warn(f"FOLLOW YOLO detector ready: {model_path}")
                except Exception as e:
                    self.yolo = None
                    self.get_logger().warn(f"FOLLOW YOLO unavailable, fallback to MediaPipe: {e}")
            else:
                self.get_logger().warn("FOLLOW YOLO library unavailable, fallback to MediaPipe")

        self.pose = None
        if MEDIAPIPE_OK:
            self.pose = mp_pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                min_detection_confidence=float(os.getenv("FOLLOW_DETECT_CONF", "0.45")),
                min_tracking_confidence=float(os.getenv("FOLLOW_TRACK_CONF", "0.42")),
            )
        else:
            self.get_logger().error("MediaPipe not available. Install mediapipe before running follow tracker.")

        self.cmd_pub = self.create_publisher(String, "/follow_cmd", 10)
        self.status_pub = self.create_publisher(String, "/follow_vision_status", 10)
        self.debug_pub = self.create_publisher(String, "/follow_debug", 10)
        self.create_subscription(String, "/auto_control", self.auto_control_cb, 10)
        self.create_subscription(String, "/follow_target_profile", self.follow_target_profile_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.lidar_cb, 10)
        self.create_subscription(Bool, "/lidar_safety_enable", self.lidar_safety_enable_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        self.timer = self.create_timer(0.067, self.tick)  # ~15 Hz
        self.get_logger().info("FOLLOW tracker ready")

    def auto_control_cb(self, msg):
        """
        Callback รับคำสั่งเปิด/ปิด FOLLOW mode จาก lawnmower_node (/auto_control)

        FOLLOW_START — เริ่มติดตาม (ต้องมี profile เจ้าของก่อน)
        AUTO_START/START — หยุด follow และตั้ง state เป็น IDLE
        FOLLOW_PAUSE/PAUSE/STOP — หยุด follow ชั่วคราว
        """
        cmd = msg.data.strip().upper()
        if cmd in ("AUTO_START", "START"):
            self.active = False
            self.follow_state = "IDLE"
            self._reset_follow_range_state()
            self._send_cmd("STOP", force=True)
            self._publish_status("FOLLOW IDLE")
            return
        if cmd == "FOLLOW_START":
            if not self.selected_profile or not self.profile_refs:
                self._load_profile_from_shared_ui_state()
            if not self.selected_profile or not self.profile_refs:
                self.active = False
                self._send_cmd("STOP", force=True)
                self._publish_status("SELECT OWNER FIRST (tracker has no owner profile)")
                return
            self.active = True
            self._ex_filt = None
            self._shoulder_filt = None
            self._cross_block_until = 0.0
            self._target_bearing_ts = 0.0
            self._target_lidar_angle = float("nan")
            self._target_lidar_range = float("inf")
            self._profile_accept_until = 0.0
            self._profile_lock_count = 0
            self._last_profile_score = float("inf")
            self.last_seen_ts = 0.0
            self.last_person_seen_ts = 0.0
            self._last_target_bbox = None
            self._last_target_cx = None
            self._last_track_id = None
            self._last_target_ts = 0.0
            self._last_target_ex = 0.0
            self._target_cx_vel_px_s = 0.0
            self._last_target_memory_ts = 0.0
            self._reset_follow_range_state()
            self.follow_state = "LOCKING"
            self._open_camera_if_needed()
            self._publish_status(f"FOLLOW ACTIVE [{self.selected_profile}]")
        elif cmd in ("FOLLOW_PAUSE", "PAUSE", "STOP"):
            self.active = False
            self.follow_state = "IDLE"
            self._reset_follow_range_state()
            self._send_cmd("STOP", force=True)
            self._publish_status("FOLLOW PAUSED")

    def _reset_follow_range_state(self):
        """รีเซ็ต state ทั้งหมดที่ใช้คำนวณระยะทางเจ้าของ (range filter และ rate) — เรียกเมื่อเริ่ม follow ใหม่"""
        self._follow_range_filt = float("inf")
        self._follow_range_rate_mps = 0.0
        self._follow_range_prev = float("inf")
        self._follow_range_prev_ts = 0.0

    def lidar_cb(self, msg: LaserScan):
        """
        Callback รับข้อมูล LaserScan จาก LIDAR (/scan)

        ทำงาน:
        1. แบ่ง scan เป็น N sector (follow_lidar_sector_count)
        2. หาค่าระยะ minimum ในแต่ละ sector
        3. จัดกลุ่ม cluster จากจุดที่ใกล้กัน (สำหรับระบุตำแหน่งเจ้าของ)
        4. ข้อมูลนี้ใช้ใน _front_lidar_stop_confirmed(), _pick_lidar_target()
           และ safety check ใน tick()
        """
        if not msg.ranges:
            return
        n = len(msg.ranges)
        if n == 0:
            return

        front_half = math.radians(self.follow_lidar_front_sector_deg * 0.5)  # default front 240 deg
        min_front = float("inf")
        sector_min = [float("inf")] * self.follow_lidar_sector_count
        sector_angle = [float("nan")] * self.follow_lidar_sector_count
        lidar_clusters = []

        front_points = []
        angle = msg.angle_min
        for idx, r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if abs(angle) <= front_half:
                    front_points.append((idx, float(r), float(angle)))
            angle += msg.angle_increment

        clusters = []
        cluster = []
        prev_idx = None
        prev_r = None
        for pt in front_points:
            idx, r, _ang = pt
            if (
                cluster
                and prev_idx is not None
                and (
                    idx - prev_idx > self.follow_lidar_cluster_max_gap
                    or abs(r - prev_r) > self.follow_lidar_cluster_max_jump_m
                )
            ):
                clusters.append(cluster)
                cluster = []
            cluster.append(pt)
            prev_idx = idx
            prev_r = r
        if cluster:
            clusters.append(cluster)

        for cl in clusters:
            if not cl:
                continue
            min_r = min(p[1] for p in cl)
            span = abs(cl[-1][2] - cl[0][2])
            width_m = 2.0 * min_r * math.sin(min(span, math.pi) * 0.5)
            if len(cl) < self.follow_lidar_cluster_min_points and width_m < self.follow_lidar_cluster_min_width_m:
                continue

            if self.follow_lidar_blob_enable and min_r <= self.follow_lidar_blob_max_m:
                weight_sum = 0.0
                sin_sum = 0.0
                cos_sum = 0.0
                for _idx, r, ang in cl:
                    w = 1.0 / max(0.15, r)
                    weight_sum += w
                    sin_sum += math.sin(ang) * w
                    cos_sum += math.cos(ang) * w
                center_ang = math.atan2(sin_sum, cos_sum) if weight_sum > 0.0 else 0.5 * (cl[0][2] + cl[-1][2])
                mean_r = sum(p[1] for p in cl) / max(1, len(cl))
                lidar_clusters.append({
                    "range": min_r,
                    "mean_range": mean_r,
                    "angle": center_ang,
                    "width": width_m,
                    "points": len(cl),
                })

            for _idx, r, ang in cl:
                min_front = min(min_front, r)
                sector_pos = (ang + front_half) / max(1e-6, 2.0 * front_half)
                sector_idx = int(sector_pos * self.follow_lidar_sector_count)
                sector_idx = max(0, min(self.follow_lidar_sector_count - 1, sector_idx))
                if r < sector_min[sector_idx]:
                    sector_min[sector_idx] = r
                    sector_angle[sector_idx] = ang

        self.lidar_min_front_m = min_front
        self.lidar_last_ts = time.time()
        self.lidar_sector_min = sector_min
        self.lidar_sector_angle = sector_angle
        self.lidar_clusters = lidar_clusters

    def lidar_safety_enable_cb(self, msg: Bool):
        """เปิด/ปิด LIDAR safety check — เมื่อ False หุ่นยนต์จะไม่หยุดแม้ LIDAR ตรวจพบสิ่งกีดขวาง"""
        self.lidar_safety_enabled = bool(msg.data)

    def follow_target_profile_cb(self, msg: String):
        """
        Callback รับชื่อ profile ของเจ้าของที่ต้องการติดตาม (/follow_target_profile)

        โหลด reference images จาก FOLLOW_PROFILE_DIR/<profile_name>/
        แล้วคำนวณ ReID embedding เพื่อใช้เปรียบเทียบในการระบุตัวเจ้าของ
        """
        name = (msg.data or "").strip()
        if not name or name.upper() == "NOT SELECTED":
            self.selected_profile = ""
            self.profile_refs = []
            self._profile_accept_until = 0.0
            self._profile_lock_count = 0
            self._last_profile_score = float("inf")
            self.follow_state = "IDLE"
            self._publish_status("SELECT OWNER FIRST")
            return
        self.selected_profile = name
        n = self._load_profile_refs(name)
        self._profile_accept_until = 0.0
        self._profile_lock_count = 0
        self._last_profile_score = float("inf")
        if n > 0:
            self.get_logger().info(f"FOLLOW target profile loaded: {name} ({n} refs)")
            self._publish_status(f"OWNER READY: {name}")
        else:
            self.get_logger().warn(f"FOLLOW target profile has no usable refs: {name}")
            self._publish_status(f"OWNER EMPTY: {name}")

    def _load_profile_from_shared_ui_state(self):
        """
        โหลด follow target profile จากไฟล์ shared_ui_state.json

        อ่านชื่อ follow_target จาก JSON แล้วโหลด reference embeddings ของบุคคลนั้น
        ใช้เมื่อ Web UI เลือกเจ้าของไว้แล้ว และต้องการให้ node รู้จักเจ้าของ
        คืนค่า: จำนวน reference ที่โหลดได้ (0 = ไม่พบหรือ error)
        """
        try:
            with open(self.shared_ui_state_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            name = str(data.get("follow_target") or "").strip()
        except Exception:
            return 0
        if not name or name.upper() == "NOT SELECTED":
            return 0
        self.selected_profile = name
        n = self._load_profile_refs(name)
        self._profile_accept_until = 0.0
        self._profile_lock_count = 0
        self._last_profile_score = float("inf")
        if n > 0:
            self.get_logger().warn(f"FOLLOW target loaded from shared UI: {name} ({n} refs)")
            self._publish_status(f"OWNER READY: {name}")
        else:
            self.get_logger().warn(f"FOLLOW shared UI owner has no refs: {name}")
            self._publish_status(f"OWNER EMPTY: {name}")
        return n

    def _init_osnet_reid_model(self):
        """
        โหลด OSNet ReID model จากไฟล์ ONNX

        OSNet เป็น model ระบุตัวบุคคล (re-identification) ที่แม่นยำกว่า MobileNet
        ถ้าโหลดสำเร็จ → return True, ถ้าไม่มี ONNX runtime หรือไฟล์ไม่อยู่ → return False
        """
        if not self.reid_enabled or not self.osnet_reid_enabled:
            return False
        if not ONNX_REID_OK:
            self.get_logger().warn("FOLLOW OSNet ReID unavailable: onnxruntime not importable")
            return False
        if not self.osnet_reid_path.exists():
            self.get_logger().warn(f"FOLLOW OSNet ReID model missing: {self.osnet_reid_path}")
            return False
        try:
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = max(1, int(os.getenv("FOLLOW_OSNET_THREADS", "2")))
            opts.inter_op_num_threads = 1
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.osnet_reid_session = ort.InferenceSession(
                str(self.osnet_reid_path),
                sess_options=opts,
                providers=["CPUExecutionProvider"],
            )
            self.osnet_reid_input_name = self.osnet_reid_session.get_inputs()[0].name
            self.osnet_reid_ready = True
            self.get_logger().warn(f"FOLLOW ReID ready: OSNet MSMT17 ONNX {self.osnet_reid_path}")
            return True
        except Exception as e:
            self.osnet_reid_session = None
            self.osnet_reid_input_name = None
            self.osnet_reid_ready = False
            self.get_logger().warn(f"FOLLOW OSNet ReID disabled, fallback to MobileNet/color: {e}")
            return False

    def _init_reid_model(self):
        """
        โหลด MobileNet ReID model (fallback เมื่อไม่มี OSNet)

        ใช้ MobileNetV3-Small ผ่าน PyTorch/torchvision
        ถ้าไม่มี torch/torchvision หรือ PIL → return False, ใช้ HSV color matching แทน
        """
        if not self.reid_enabled:
            return False
        if not REID_OK:
            self.get_logger().warn("FOLLOW ReID unavailable: torch/torchvision/PIL not importable; using color matcher")
            return False
        try:
            try:
                torch.set_num_threads(max(1, int(os.getenv("FOLLOW_REID_TORCH_THREADS", "2"))))
            except Exception:
                pass
            use_cuda = os.getenv("FOLLOW_REID_CUDA", "0").lower() in ("1", "true", "yes", "on")
            self.reid_device = torch.device("cuda:0" if (use_cuda and torch.cuda.is_available()) else "cpu")
            weights = MobileNet_V3_Small_Weights.DEFAULT
            model = mobilenet_v3_small(weights=weights)
            model.classifier = torch.nn.Identity()
            model.eval().to(self.reid_device)
            self.reid_model = model
            self.reid_preprocess = weights.transforms()
            self.reid_ready = True
            self.get_logger().warn(f"FOLLOW ReID ready: MobileNetV3-Small on {self.reid_device}")
            return True
        except Exception as e:
            self.reid_model = None
            self.reid_preprocess = None
            self.reid_ready = False
            self.get_logger().warn(f"FOLLOW ReID disabled, fallback to color matcher: {e}")
            return False

    def _compute_osnet_reid_embedding(self, image):
        """
        คำนวณ ReID feature vector ของ image โดยใช้ OSNet ONNX model

        image — crop ของคนที่ตรวจจับได้ (numpy BGR array)
        return — normalized embedding vector (float32) หรือ None ถ้าไม่สำเร็จ
        """
        if not self.osnet_reid_ready or self.osnet_reid_session is None or image is None or image.size == 0:
            return None
        try:
            h, w = image.shape[:2]
            if h < 48 or w < 24:
                return None
            resized = cv2.resize(image, (128, 256), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
            std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
            rgb = (rgb - mean) / std
            blob = np.transpose(rgb, (2, 0, 1))[None].astype(np.float32)
            emb = self.osnet_reid_session.run(None, {self.osnet_reid_input_name: blob})[0][0]
            emb = emb.astype(np.float32).reshape(-1)
            norm = float(np.linalg.norm(emb))
            if norm <= 1e-6:
                return None
            return emb / norm
        except Exception:
            return None

    def _compute_hsv_hist(self, image):
        """
        คำนวณ HSV color histogram เป็น feature fallback เมื่อไม่มี ReID model

        ใช้เฉพาะ H (hue) และ S (saturation) channel เพื่อให้ทน lighting ได้ดีกว่า
        normalize ค่าสว่างก่อนเพื่อลดผลจากแสงกลางแจ้ง
        """
        if image is None or image.size == 0:
            return None
        # Light normalization makes profile matching less fragile outdoors.
        img = cv2.resize(image, (96, 160), interpolation=cv2.INTER_AREA)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_chan, a_chan, b_chan = cv2.split(lab)
        l_chan = cv2.equalizeHist(l_chan)
        norm = cv2.cvtColor(cv2.merge((l_chan, a_chan, b_chan)), cv2.COLOR_LAB2BGR)
        hsv = cv2.cvtColor(norm, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [24, 20], [0, 180, 0, 256])
        if hist is None:
            return None
        cv2.normalize(hist, hist, alpha=0.0, beta=1.0, norm_type=cv2.NORM_L1)
        return hist

    def _compute_reid_embedding(self, image):
        """
        คำนวณ ReID feature vector โดยใช้ MobileNetV3 (PyTorch)

        ใช้ global average pooling output ก่อน FC layer เพื่อให้ได้ embedding
        แล้ว normalize เป็น unit vector สำหรับเปรียบเทียบด้วย cosine distance
        """
        if not self.reid_ready or self.reid_model is None or image is None or image.size == 0:
            return None
        try:
            h, w = image.shape[:2]
            if h < 48 or w < 24:
                return None
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil = Image.fromarray(rgb)
            tensor = self.reid_preprocess(pil).unsqueeze(0).to(self.reid_device)
            with torch.no_grad():
                emb = self.reid_model(tensor)
                emb = torch_f.normalize(emb, p=2, dim=1)
            return emb.squeeze(0).detach().cpu().numpy().astype(np.float32)
        except Exception:
            return None

    def _compute_appearance_feature(self, image):
        """
        คำนวณ appearance feature ของ image โดยเลือก method ที่ดีที่สุดที่มี

        ลำดับความสำคัญ: OSNet ONNX → MobileNet → HSV histogram
        ตัดขอบรูปออก 12% ทุกด้านเพื่อให้ focus เฉพาะเสื้อผ้า ไม่รวม background
        """
        if image is None or image.size == 0:
            return None
        h, w = image.shape[:2]
        if h < 20 or w < 12:
            return None
        # Ignore background edges; person center is more stable for clothes.
        x1 = int(w * 0.12)
        x2 = int(w * 0.88)
        y1 = int(h * 0.05)
        y2 = int(h * 0.96)
        crop = image[y1:y2, x1:x2]
        if crop.size == 0:
            return None
        hist = self._compute_hsv_hist(crop)
        if hist is None:
            return None
        small = cv2.resize(crop, (32, 48), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
        # Upper/lower body color descriptors. Low dimensional but robust enough
        # for "select owner from a few photos" without adding a re-id model.
        upper_hsv = np.mean(hsv[:24, :, :], axis=(0, 1)).astype(np.float32)
        lower_hsv = np.mean(hsv[24:, :, :], axis=(0, 1)).astype(np.float32)
        upper_lab = np.mean(lab[:24, :, :], axis=(0, 1)).astype(np.float32)
        lower_lab = np.mean(lab[24:, :, :], axis=(0, 1)).astype(np.float32)
        return {
            "hist": hist,
            "upper_hsv": upper_hsv,
            "lower_hsv": lower_hsv,
            "upper_lab": upper_lab,
            "lower_lab": lower_lab,
            "osnet": self._compute_osnet_reid_embedding(crop),
            "reid": None if self.osnet_reid_ready else self._compute_reid_embedding(crop),
        }

    def _load_profile_refs(self, name):
        """
        โหลด reference images ของ profile เจ้าของจากโฟลเดอร์

        path: FOLLOW_PROFILE_DIR/<name>/*.jpg|png
        คำนวณ feature embedding ของแต่ละ reference image
        เก็บไว้ใน self.profile_refs เพื่อใช้เปรียบเทียบในการระบุตัวเจ้าของ
        """
        self.profile_refs = []
        folder = self.follow_profile_dir / name
        if not folder.is_dir():
            return 0
        exts = {'.jpg', '.jpeg', '.png', '.webp', '.bmp'}
        count = 0
        for fp in sorted(folder.iterdir()):
            if fp.suffix.lower() not in exts:
                continue
            img = cv2.imread(str(fp))
            if img is None or img.size == 0:
                continue
            img = self._auto_contrast_frame(img)
            h, w = img.shape[:2]
            crops = [img]
            x1 = int(w * 0.18)
            x2 = int(w * 0.82)
            y1 = int(h * 0.10)
            y2 = int(h * 0.88)
            if (x2 - x1) > 20 and (y2 - y1) > 20:
                crops.append(img[y1:y2, x1:x2])
            for crop in crops:
                feat = self._compute_appearance_feature(crop)
                if feat is not None:
                    self.profile_refs.append(feat)
            count += 1
            if count >= 12:
                break
        return len(self.profile_refs)

    def _pose_bbox(self, lm, w, h):
        """
        คำนวณ bounding box ของคนจาก MediaPipe pose landmarks

        ใช้จุด landmark สำคัญ: หัว (0), ไหล่ (11,12), สะโพก (23,24)
        เพิ่ม margin รอบๆ เพื่อให้ crop ครอบคลุมร่างกายพอ
        return: (x1, y1, x2, y2) หรือ None ถ้าจุดไม่ครบ/กล่องเล็กเกินไป
        """
        pts = []
        for idx in (0, 11, 12, 23, 24):
            if idx >= len(lm):
                continue
            p = lm[idx]
            if p.visibility < self.follow_human_min_visibility:
                continue
            pts.append((p.x * w, p.y * h))
        if len(pts) < 3:
            return None
        xs = [x for x, _ in pts]
        ys = [y for _, y in pts]
        x1 = max(0, int(min(xs) - 0.35 * (max(xs) - min(xs) + 1)))
        x2 = min(w - 1, int(max(xs) + 0.35 * (max(xs) - min(xs) + 1)))
        y1 = max(0, int(min(ys) - 0.28 * (max(ys) - min(ys) + 1)))
        y2 = min(h - 1, int(max(ys) + 0.22 * (max(ys) - min(ys) + 1)))
        if (x2 - x1) < 24 or (y2 - y1) < 40:
            return None
        return x1, y1, x2, y2

    def _pose_is_human_like(self, lm, w, h):
        """
        ตรวจสอบว่า pose landmarks มีสัดส่วนคล้ายมนุษย์จริงๆ

        ป้องกัน false positive จากวัตถุอื่น (ต้นไม้, สัตว์)
        โดยตรวจสอบ torso height, shoulder width, และ head position
        return: True ถ้ามีลักษณะเหมือนคน
        """
        try:
            l_sh, r_sh = lm[11], lm[12]
            l_hp, r_hp = lm[23], lm[24]
        except Exception:
            return False
        if min(l_sh.visibility, r_sh.visibility, l_hp.visibility, r_hp.visibility) < self.follow_human_min_visibility:
            return False
        vis_count = sum(1 for p in lm if getattr(p, "visibility", 0.0) >= self.follow_human_min_visibility)
        if vis_count < 6:
            return False
        shoulder_y = ((l_sh.y + r_sh.y) * 0.5) * h
        hip_y = ((l_hp.y + r_hp.y) * 0.5) * h
        shoulder_w = abs(l_sh.x - r_sh.x) * w
        torso_h = abs(hip_y - shoulder_y)
        if shoulder_w < 34 or torso_h < 52:
            return False
        if torso_h < (0.65 * shoulder_w) or torso_h > (4.1 * shoulder_w):
            return False
        if abs((l_sh.y - r_sh.y) * h) > max(35.0, shoulder_w * 0.55):
            return False
        nose = lm[0]
        if nose.visibility > 0.30 and (nose.y * h) > (shoulder_y + 8.0):
            return False
        return True

    def _match_selected_profile(self, frame, bbox):
        """
        เปรียบเทียบ crop ของคนที่ตรวจจับได้กับ reference images ของเจ้าของ

        คำนวณ distance ระหว่าง feature vector ของ crop และ references
        return: distance score (ต่ำ = ตรงกันมาก) หรือ None ถ้าไม่สำเร็จ
        """
        if not self.profile_refs or frame is None or bbox is None:
            return None
        x1, y1, x2, y2 = bbox
        crop = frame[y1:y2, x1:x2]
        feat = self._compute_appearance_feature(crop)
        if feat is None:
            return None
        best = None
        best_mode = "color"
        for ref in self.profile_refs:
            try:
                if isinstance(ref, dict):
                    hist_score = cv2.compareHist(feat["hist"], ref["hist"], cv2.HISTCMP_BHATTACHARYYA)
                    hsv_score = (
                        np.linalg.norm(feat["upper_hsv"] - ref["upper_hsv"]) +
                        0.65 * np.linalg.norm(feat["lower_hsv"] - ref["lower_hsv"])
                    ) / 360.0
                    lab_score = (
                        np.linalg.norm(feat["upper_lab"] - ref["upper_lab"]) +
                        0.65 * np.linalg.norm(feat["lower_lab"] - ref["lower_lab"])
                    ) / 255.0
                    color_score = 0.62 * hist_score + 0.23 * hsv_score + 0.15 * lab_score
                    mode = "color"
                    if feat.get("osnet") is not None and ref.get("osnet") is not None:
                        cosine_sim = float(np.dot(feat["osnet"], ref["osnet"]))
                        cosine_dist = max(0.0, min(2.0, 1.0 - cosine_sim))
                        osnet_score = min(1.50, cosine_dist * self.osnet_reid_score_scale)
                        w = max(0.0, min(0.95, self.osnet_reid_weight))
                        score = (1.0 - w) * color_score + w * osnet_score
                        mode = "osnet"
                    elif feat.get("reid") is not None and ref.get("reid") is not None:
                        cosine_sim = float(np.dot(feat["reid"], ref["reid"]))
                        cosine_dist = max(0.0, min(2.0, 1.0 - cosine_sim))
                        reid_score = min(1.50, cosine_dist * self.reid_score_scale)
                        w = max(0.0, min(0.95, self.reid_weight))
                        score = (1.0 - w) * color_score + w * reid_score
                        mode = "reid"
                    else:
                        score = color_score
                else:
                    score = cv2.compareHist(feat["hist"], ref, cv2.HISTCMP_BHATTACHARYYA)
                    mode = "hist"
            except Exception:
                continue
            if best is None or score < best:
                best = score
                best_mode = mode
        self._last_profile_match_mode = best_mode if best is not None else "none"
        return best

    def _predict_target_cx(self, frame_w, now):
        """
        ทำนาย cx ของเจ้าของในเฟรมปัจจุบันโดยใช้ velocity ของ target

        ใช้ linear prediction จาก cx ล่าสุด + ความเร็ว (_target_cx_vel_px_s)
        เพื่อประมาณตำแหน่งเมื่อกล้องไม่เจอ target ชั่วคราว
        คืนค่า None ถ้าไม่มีข้อมูลอ้างอิง
        """
        if self._last_target_cx is None or self._last_target_ts <= 0.0:
            return None
        dt = max(0.0, min(self.follow_track_hold_s, now - self._last_target_ts))
        pred = self._last_target_cx + self._target_cx_vel_px_s * dt
        return max(0.0, min(float(frame_w - 1), pred))

    def _remember_yolo_target(self, target, now):
        """
        บันทึกตำแหน่ง cx ของ target ที่ YOLO เจอ และอัปเดต velocity

        คำนวณ velocity (px/s) ของ cx เพื่อใช้ใน _predict_target_cx()
        ทำให้ติดตามเจ้าของได้ต่อเนื่องแม้กล้องเจอบ้างไม่เจอบ้าง
        """
        cx = float(target["cx"])
        if self._last_target_cx is not None and self._last_target_memory_ts > 0.0:
            dt = max(0.02, min(0.50, now - self._last_target_memory_ts))
            raw_vel = max(-900.0, min(900.0, (cx - self._last_target_cx) / dt))
            self._target_cx_vel_px_s = (0.70 * self._target_cx_vel_px_s) + (0.30 * raw_vel)
        else:
            self._target_cx_vel_px_s = 0.0
        self._last_target_bbox = target["bbox"]
        self._last_target_cx = cx
        self._last_track_id = target.get("track_id")
        self._last_target_ts = now
        self._last_target_memory_ts = now

    def _camera_candidates(self):
        """
        สร้างรายการ camera device/URL ที่จะลองเปิดตามลำดับ

        รวม device จาก camera_mode (usb/mjpeg/snapshot/auto) และ
        FOLLOW_CAMERA_CANDIDATES_ENV กับ /dev/video* fallback
        คืนค่า: list ของ string (path หรือ URL)
        """
        cands = []
        seen = set()

        def add(x):
            if x in seen:
                return
            seen.add(x)
            cands.append(x)

        if self.camera_mode == "auto":
            add(self.camera_device)
            if self.camera_mjpeg_url:
                add(self.camera_mjpeg_url)
        elif self.camera_mode == "snapshot" and self.camera_snapshot_url:
            add(self.camera_snapshot_url)
        elif self.camera_mode == "mjpeg" and self.camera_mjpeg_url:
            add(self.camera_mjpeg_url)
        elif self.camera_mode == "usb":
            add(self.camera_device)
        if self.camera_candidates_env:
            for p in self.camera_candidates_env.split(","):
                p = p.strip()
                if p:
                    add(p)
        # fallback candidates (USB mode only)
        if self.camera_mode in ("auto", "usb"):
            for p in sorted(glob.glob("/dev/video*")):
                add(p)

        # Some OpenCV builds on Jetson cannot capture by '/dev/videoX' name,
        # but can open the same node by numeric index.
        if self.camera_mode in ("auto", "usb"):
            for p in list(cands):
                if isinstance(p, str) and p.startswith("/dev/video"):
                    suf = p.replace("/dev/video", "", 1)
                    if suf.isdigit():
                        add(int(suf))
        return cands

    def _open_camera_if_needed(self):
        """
        เปิด camera capture ถ้ายังไม่ได้เปิดหรือเปิดล้มเหลว

        รองรับหลาย mode:
        - snapshot: ดึง JPEG จาก HTTP URL ทีละเฟรม (ไม่ใช้ cv2 persistent stream)
        - mjpeg: stream จาก HTTP MJPEG URL
        - usb: กล้อง USB ผ่าน cv2.VideoCapture
        - auto: ลองทีละ candidate ตามลำดับ
        """
        # Snapshot mode doesn't hold a persistent cv2 capture.
        if self.camera_mode == "snapshot":
            self._cam_source = self.camera_snapshot_url
            self._snapshot_fallback = True
            return

        if self.cap is not None and self.cap.isOpened():
            self._snapshot_fallback = False
            return
        now = time.time()
        if now - self._last_cam_open_try_ts < 0.35:
            return
        self._last_cam_open_try_ts = now

        def try_open(dev, backend=None):
            cap = None
            try:
                if backend is None:
                    cap = cv2.VideoCapture(dev)
                else:
                    cap = cv2.VideoCapture(dev, backend)
                if cap is None or not cap.isOpened():
                    return None
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                except Exception:
                    pass
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                cap.set(cv2.CAP_PROP_FPS, 30)
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                # Verify real frame read before accepting this device.
                ok_any = False
                for _ in range(4):
                    try:
                        cap.grab()
                    except Exception:
                        pass
                    ok, frame = cap.read()
                    if ok and frame is not None and frame.size > 0:
                        ok_any = True
                        break
                    time.sleep(0.03)
                if not ok_any:
                    cap.release()
                    return None
                return cap
            except Exception:
                try:
                    if cap is not None:
                        cap.release()
                except Exception:
                    pass
                return None

        for cand in self._camera_candidates():
            dev = int(cand) if str(cand).isdigit() else cand
            is_mjpeg = isinstance(dev, str) and dev.startswith(("http://", "https://"))
            if is_mjpeg:
                cap = try_open(dev, None)
            else:
                # Prefer pure V4L2 path first to avoid FFmpeg ioctl spam on Linux.
                cap = try_open(dev, cv2.CAP_V4L2)
                if cap is None:
                    cap = try_open(dev, None)
            if cap is not None:
                self.cap = cap
                self._cam_source = str(cand)
                self._snapshot_fallback = False
                self._prev_gray_small = None
                self._freeze_accum_s = 0.0
                self._last_frame_check_ts = time.time()
                self.get_logger().warn(f"FOLLOW camera opened: {self._cam_source}")
                return

        if self.camera_mode == "auto" and self.camera_snapshot_url:
            self._cam_source = self.camera_snapshot_url
            self._snapshot_fallback = True
            if now - self._last_cam_err_log_ts > 1.0:
                self._last_cam_err_log_ts = now
                self.get_logger().warn("FOLLOW USB camera busy/unavailable, using web snapshot fallback")
            return

        if now - self._last_cam_err_log_ts > 1.0:
            self._last_cam_err_log_ts = now
            self.get_logger().error(f"Cannot open follow camera (tried: {self._camera_candidates()})")

    def _odom_cb(self, msg):
        """
        Callback รับ Odometry จาก /odom เพื่อคำนวณ yaw rate

        แปลง quaternion orientation → yaw (radian)
        คำนวณ yaw rate โดย low-pass filter เพื่อใช้เป็น feedback
        สำหรับ steering boost ขณะหมุน (ดู _send_cmd)
        """
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_new = math.atan2(siny, cosy)
        now = time.time()
        if self._yaw_ts > 0.0:
            dt = max(0.01, min(0.5, now - self._yaw_ts))
            dyaw = math.atan2(math.sin(yaw_new - self._yaw_cur), math.cos(yaw_new - self._yaw_cur))
            raw_rate = max(-6.0, min(6.0, dyaw / dt))
            self._yaw_rate = 0.7 * self._yaw_rate + 0.3 * raw_rate
        self._yaw_cur = yaw_new
        self._yaw_ts = now

    def _send_cmd(self, cmd, force=False):
        """
        ส่งคำสั่งขับเคลื่อน (FORWARD/BACK/LEFT/RIGHT/STOP) ไปยัง /follow_cmd

        มี rate limiting เพื่อไม่ให้ publish ซ้ำถี่เกินไป (cmd_repeat_s)
        force=True: bypass rate limiter เพื่อส่ง STOP ทันทีในกรณีฉุกเฉิน
        รองรับ PWM command โดยตรง ("PWM,L,R") สำหรับการควบคุมละเอียด
        """
        now = time.time()
        if cmd.startswith("PWM,"):
            try:
                parts = cmd.split(",")
                target_l = max(-255.0, min(255.0, float(parts[1])))
                target_r = max(-255.0, min(255.0, float(parts[2])))
                dt = max(0.02, min(0.20, now - self._cmd_pwm_last_ts)) if self._cmd_pwm_last_ts > 0.0 else 0.067
                self._cmd_pwm_last_ts = now

                def slew(target, prev):
                    if target != 0.0 and prev != 0.0 and ((target > 0.0) != (prev > 0.0)):
                        prev = 0.0
                    rate = self.follow_pwm_slew_up_per_s if abs(target) >= abs(prev) else self.follow_pwm_slew_down_per_s
                    step = max(1.0, rate * dt)
                    if target > prev:
                        return min(target, prev + step)
                    return max(target, prev - step)

                self._cmd_pwm_l = slew(target_l, self._cmd_pwm_l)
                self._cmd_pwm_r = slew(target_r, self._cmd_pwm_r)
                cmd = f"PWM,{int(round(self._cmd_pwm_l))},{int(round(self._cmd_pwm_r))}"
            except Exception:
                pass
        elif cmd == "STOP":
            self._cmd_pwm_l = 0.0
            self._cmd_pwm_r = 0.0
            self._cmd_pwm_last_ts = now

        if not force and cmd == self.last_cmd and (now - self.last_cmd_ts) < self.cmd_repeat_s:
            return
        self.last_cmd = cmd
        self.last_cmd_ts = now
        self.cmd_pub.publish(String(data=cmd))

    def _publish_status(self, text):
        """Publish สถานะ Follow Tracker ไปยัง topic /follow_status (String)"""
        self.status_pub.publish(String(data=text))

    def _publish_debug(self, cmd, status, people_count=0):
        """
        Publish ข้อมูล debug แบบ JSON ไปยัง topic /follow_debug

        รวมข้อมูลสำคัญทั้งหมด: คำสั่ง PWM, สถานะ, ReID score, LIDAR range,
        จำนวนคนที่เจอ, track ID, และค่า tuning parameter ต่าง ๆ
        """
        payload = {
            "cmd": cmd,
            "status": status,
            "detector": "yolo" if self.yolo is not None and self.detector_mode in ("auto", "yolo") else "mediapipe",
            "state": self.follow_state,
            "owner": self.selected_profile or "NOT SELECTED",
            "score": None if not math.isfinite(self._last_profile_score) else round(float(self._last_profile_score), 3),
            "match_max": round(float(self.follow_profile_match_max), 3),
            "refs": len(self.profile_refs),
            "match_mode": self._last_profile_match_mode,
            "steer_zones": int(self.follow_steer_zones),
            "people": int(people_count),
            "track_id": self._last_track_id,
            "target_ex": round(float(self._last_target_ex), 3),
            "target_cx_vel_px_s": round(float(self._target_cx_vel_px_s), 1),
            "target_age_s": round(max(0.0, time.time() - self._last_target_ts), 2) if self._last_target_ts > 0 else None,
            "lidar_min_m": None if not math.isfinite(self.lidar_min_front_m) else round(float(self.lidar_min_front_m), 2),
            "target_lidar_m": None if not math.isfinite(self._target_lidar_range) else round(float(self._target_lidar_range), 2),
            "target_lidar_deg": None if not math.isfinite(self._target_lidar_angle) else round(math.degrees(self._target_lidar_angle), 1),
            "lidar_blobs": len(self.lidar_clusters),
            "camera": self._cam_source or "",
        }
        try:
            self.debug_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception:
            pass

    def _is_follow_lost_or_untrusted(self, status):
        """
        ตรวจสอบว่า follow status ปัจจุบัน หมายถึง "สูญหาย" หรือ "ไม่น่าเชื่อถือ"

        ใช้เพื่อตัดสินใจว่าควรเริ่มนับ follow_lost_timeout หรือไม่
        Status ที่ขึ้นต้นด้วย prefix ใน prefixes จะถือว่า lost หรือ untrusted
        """
        prefixes = (
            "NO_TARGET",
            "LOW_VIS",
            "NO_BBOX",
            "REJECT_NON_HUMAN",
            "REACQUIRE_WAIT",
            "WRONG_TARGET",
            "AMBIGUOUS_OWNER",
            "LOCKING",
            "SELECT OWNER FIRST",
            "FAR_TARGET",
        )
        return any(str(status).startswith(p) for p in prefixes)

    def _clear_follow_drive_memory(self):
        """รีเซ็ต state การขับเคลื่อนทั้งหมดของ follow mode (cmd, status, PWM, timestamp)"""
        self.last_drive_cmd = "STOP"
        self.last_drive_status = ""
        self.last_drive_seen_ts = 0.0
        self._cmd_pwm_l = 0.0
        self._cmd_pwm_r = 0.0

    def _idle_status(self):
        """Publish สถานะ IDLE ทุก 1 วินาที เพื่อให้ Web UI รู้ว่า follow tracker ยังทำงานอยู่"""
        now = time.time()
        if now - self._last_idle_status_ts < 1.0:
            return
        self._last_idle_status_ts = now
        if not self.selected_profile or not self.profile_refs:
            self._publish_status("SELECT OWNER FIRST")
        elif not self.active:
            self._publish_status(f"READY owner={self.selected_profile} refs={len(self.profile_refs)}")
            self._publish_debug("STOP", "READY", 0)

    def _read_snapshot_frame(self):
        """
        ดึงเฟรมภาพโดย HTTP GET จาก snapshot URL (สำหรับ camera_mode="snapshot")

        ใช้กับกล้องที่เปิด HTTP server เช่น mjpeg-streamer
        return: numpy BGR image หรือ None ถ้าดึงไม่สำเร็จ
        """
        try:
            r = self._http.get(self.camera_snapshot_url, timeout=(0.20, 0.35), stream=False)
            if r.status_code != 200 or not r.content:
                return None
            arr = np.frombuffer(r.content, dtype=np.uint8)
            if arr.size == 0:
                return None
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return frame
        except Exception:
            return None

    def _pick_lidar_target(self, desired_bearing_rad):
        """
        เลือก LIDAR cluster ที่น่าจะเป็นเจ้าของตาม bearing จากกล้อง

        desired_bearing_rad — มุมที่กล้องบอกว่าเจ้าของอยู่ (radian จาก center)
        ใช้ weighted score จากระยะห่าง, ความกว้าง, และ bearing match
        return: tuple (range_m, angle_rad) ของ cluster ที่ดีที่สุด หรือ (inf, nan)
        """
        now = time.time()
        assoc_half = math.radians(max(4.0, self.follow_lidar_assoc_window_deg))
        best = None
        if self.follow_lidar_blob_enable:
            for cl in self.lidar_clusters:
                rng = float(cl.get("range", float("inf")))
                ang = float(cl.get("angle", float("nan")))
                width_m = float(cl.get("width", 0.0))
                points = int(cl.get("points", 0))
                if not math.isfinite(rng) or not math.isfinite(ang):
                    continue
                if rng > self.follow_lidar_blob_max_m:
                    continue
                d_ang = math.atan2(math.sin(ang - desired_bearing_rad), math.cos(ang - desired_bearing_rad))
                # When the camera has confirmed an owner, a nearby human blob may
                # be offset from the visual center. Allow a wider association than
                # the coarse sector tracker, then prefer large close blobs.
                if abs(d_ang) > max(assoc_half, math.radians(42.0)):
                    continue
                width_bonus = min(0.35, max(0.0, width_m - self.follow_lidar_blob_min_width_m) * 0.9)
                point_bonus = min(0.20, points * 0.015)
                score = abs(d_ang) * 1.2 + min(rng, 6.0) * 0.08 - width_bonus - point_bonus
                cand = (score, rng, ang)
                if best is None or cand < best:
                    best = cand

        for rng, ang in zip(self.lidar_sector_min, self.lidar_sector_angle):
            if not math.isfinite(rng) or not math.isfinite(ang):
                continue
            d_ang = math.atan2(math.sin(ang - desired_bearing_rad), math.cos(ang - desired_bearing_rad))
            if abs(d_ang) > assoc_half:
                continue
            score = abs(d_ang) * 3.2 + min(rng, 6.0) * 0.14
            cand = (score, rng, ang)
            if best is None or cand < best:
                best = cand

        if best is not None:
            _, rng, ang = best
            self._target_lidar_angle = ang
            self._target_lidar_range = rng
            self._target_bearing_rad = ang
            self._target_bearing_ts = now
            return ang, rng, True

        if (
            math.isfinite(self._target_lidar_angle)
            and (now - self._target_bearing_ts) <= self.follow_lidar_track_hold_s
        ):
            return self._target_lidar_angle, self._target_lidar_range, False

        return float("nan"), float("inf"), False

    def _is_follow_safety_sector(self, sector_idx):
        """
        ตรวจสอบว่า LIDAR sector นี้อยู่ใน safety corridor (แนวหน้าตรง) หรือไม่

        เฉพาะ sector ที่อยู่ตรงกลาง ±follow_safety_corridor_sectors เท่านั้น
        ที่จะทริกเกอร์ safety stop เพื่อไม่ให้สิ่งกีดขวางด้านข้างทำให้หยุดไม่จำเป็น
        """
        # Only a narrow forward corridor should trigger FOLLOW safety stop.
        # Side/edge LiDAR returns are useful for awareness, but should not make
        # follow mode freeze unless they are in the immediate driving corridor.
        center = self.follow_lidar_sector_count // 2
        return abs(sector_idx - center) <= self.follow_safety_corridor_sectors

    def _dangerous_lidar_sector_count(self, ignore_sector=None):
        """
        นับจำนวน sector ใน safety corridor ที่มีสิ่งกีดขวางใกล้กว่า stop_dist_m

        ignore_sector — sector ที่จะข้ามไม่นับ (เพราะนั่นอาจเป็น sector ของเจ้าของ)
        return: จำนวน sector ที่อันตราย
        """
        count = 0
        for i, rng in enumerate(self.lidar_sector_min):
            if not self._is_follow_safety_sector(i):
                continue
            if ignore_sector is not None and abs(i - ignore_sector) <= self.follow_lidar_ignore_target_sectors:
                continue
            if math.isfinite(rng) and rng <= self.stop_dist_m:
                count += 1
        return count

    def _nearest_lidar_range(self, ignore_sector=None):
        """
        หาระยะสิ่งกีดขวางที่ใกล้ที่สุดในบริเวณ safety sector ด้านหน้า

        ถ้า ignore_sector ระบุ จะข้าม sector นั้น (และ sector ใกล้เคียง
        ตาม follow_lidar_ignore_target_sectors) เพื่อไม่นับเจ้าของเป็นสิ่งกีดขวาง
        คืนค่า: ระยะ (เมตร) หรือ inf ถ้าไม่มีสิ่งกีดขวาง
        """
        nearest = float("inf")
        for i, rng in enumerate(self.lidar_sector_min):
            if not self._is_follow_safety_sector(i):
                continue
            if ignore_sector is not None and abs(i - ignore_sector) <= self.follow_lidar_ignore_target_sectors:
                continue
            if math.isfinite(rng):
                nearest = min(nearest, rng)
        return nearest

    def _hard_lidar_stop_range(self):
        """
        คำนวณระยะสิ่งกีดขวางที่ใกล้ที่สุดสำหรับ hard stop (ไม่มี exception)

        ใช้เป็น last-resort: ถ้ามีอะไรใกล้มากกว่า follow_lidar_any_hard_stop_m
        ต้องหยุดทันทีโดยไม่ยกเว้น sector ของเจ้าของ
        """
        # Absolute last-resort collision guard. Do not ignore target sector here:
        # a hand/object directly in front of the LiDAR must stop the mower even
        # if it lies in the same sector as the followed person.
        any_nearest = min(
            (rng for rng in self.lidar_sector_min if math.isfinite(rng)),
            default=float("inf")
        )
        if math.isfinite(any_nearest) and any_nearest <= self.follow_lidar_any_hard_stop_m:
            return any_nearest
        nearest = self._nearest_lidar_range(ignore_sector=None)
        if math.isfinite(nearest) and nearest <= self.follow_lidar_hard_stop_m:
            return nearest
        return float("inf")

    def _auto_contrast_frame(self, frame):
        """
        ปรับ contrast/brightness ของภาพอัตโนมัติสำหรับสภาพแสงที่เปลี่ยนแปลง

        ตรวจสอบ exposure/contrast ก่อน ถ้าดีพออยู่แล้วจะไม่แก้ไข
        ปรับด้วย CLAHE และ gamma correction เพื่อให้ตรวจจับคนในที่มืดได้ดีขึ้น
        """
        if cv2 is None or np is None:
            return frame
        if not self.follow_auto_contrast or frame is None or frame.size == 0:
            return frame
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mean = float(np.mean(gray))
            p05, p50, p95 = np.percentile(gray, [5, 50, 95])
            contrast = float(p95 - p05)

            # Good exposure/contrast: keep raw image to avoid unnecessary color shift.
            if 85.0 <= mean <= 170.0 and contrast >= 78.0 and p95 < 238.0:
                return frame

            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l_chan, a_chan, b_chan = cv2.split(lab)
            clahe = cv2.createCLAHE(
                clipLimit=max(1.0, min(5.0, self.follow_auto_contrast_clip)),
                tileGridSize=(8, 8),
            )
            l_eq = clahe.apply(l_chan)

            # Lift shadows in backlight; tame highlights when the mower light
            # blows the face/shirt out. Gamma is deliberately bounded.
            if mean < 95.0 or p50 < 85.0:
                gamma = min(1.85, max(1.10, self.follow_dim_gamma + (100.0 - min(mean, 100.0)) / 120.0))
            elif p95 > 235.0:
                gamma = 0.82
            else:
                gamma = 1.0
            inv_gamma = 1.0 / max(0.45, min(2.2, gamma))
            table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8)
            l_eq = cv2.LUT(l_eq, table)

            enhanced = cv2.cvtColor(cv2.merge((l_eq, a_chan, b_chan)), cv2.COLOR_LAB2BGR)
            blend = max(0.0, min(1.0, self.follow_auto_contrast_blend))
            return cv2.addWeighted(enhanced, blend, frame, 1.0 - blend, 0.0)
        except Exception:
            return frame

    def _prepare_pose_frame(self, frame):
        """เตรียมเฟรมสำหรับ MediaPipe Pose โดยปรับ contrast อัตโนมัติก่อนส่ง"""
        return self._auto_contrast_frame(frame)

    def _estimate_lidar_distance_from_camera_size(self, apparent_px):
        """
        ประมาณระยะห่างของเจ้าของจากขนาดในภาพกล้อง (apparent size)

        สูตร: camera_distance = A / apparent_px + B (calibrated constants)
        แปลงเป็นระยะทาง LIDAR โดยบวก camera-to-lidar offset
        ใช้เมื่อ LIDAR ไม่เจอ target เพื่อประมาณระยะจาก vision แทน

        Estimate target range at the LiDAR origin from camera target size.

        The camera is mounted ahead of the LiDAR, so a measured camera-to-human
        distance must be shifted by FOLLOW_CAMERA_TO_LIDAR_OFFSET_M before it is
        compared against LiDAR ranges.
        """
        if self.follow_camera_dist_a > 0.0:
            cam_est_m = (self.follow_camera_dist_a / max(1.0, apparent_px)) + self.follow_camera_dist_b
        else:
            cam_ideal_m = max(0.10, self.follow_target_ideal_m - self.follow_camera_to_lidar_offset_m)
            cam_est_m = cam_ideal_m * (self.target_shoulder_px / max(1.0, apparent_px))
        cam_est_m = max(0.20, min(8.0, cam_est_m))
        lidar_est_m = cam_est_m + self.follow_camera_to_lidar_offset_m
        return cam_est_m, lidar_est_m

    def _follow_distance_pwm(self, range_m, fallback_dist_norm):
        """
        คำนวณ PWM สำหรับควบคุมระยะห่างจากเจ้าของ (follow distance control)

        แบ่ง band ระยะทาง: ideal (หยุด), slow near/mid/far (เดินตาม), backoff (ถอย)
        รวม rate feedback (ความเร็วเปลี่ยนระยะ) เพื่อลดการสั่น
        PWM บวก = เดินหน้า, PWM ลบ = ถอยหลัง
        คืนค่า: (pwm: float, debug_str: str)
        """
        now = time.time()
        if not math.isfinite(range_m):
            range_m = self.follow_target_ideal_m + max(0.0, min(1.0, fallback_dist_norm)) * (
                self.follow_target_cruise_m - self.follow_target_ideal_m
            )

        a = max(0.01, min(1.0, self.follow_distance_alpha))
        if not math.isfinite(self._follow_range_filt):
            self._follow_range_filt = range_m
        else:
            self._follow_range_filt = (1.0 - a) * self._follow_range_filt + a * range_m

        rate = 0.0
        if math.isfinite(self._follow_range_prev) and self._follow_range_prev_ts > 0.0:
            dt = max(0.02, min(0.50, now - self._follow_range_prev_ts))
            # Positive rate = target is moving away; negative = target is approaching.
            raw_rate = max(-2.0, min(2.0, (self._follow_range_filt - self._follow_range_prev) / dt))
            ar = max(0.01, min(1.0, self.follow_distance_rate_alpha))
            self._follow_range_rate_mps = (1.0 - ar) * self._follow_range_rate_mps + ar * raw_rate
            rate = self._follow_range_rate_mps

        self._follow_range_prev = self._follow_range_filt
        self._follow_range_prev_ts = now

        err = self._follow_range_filt - self.follow_target_ideal_m
        hold = max(0.02, self.follow_distance_hold_band_m)
        if abs(err) <= hold and abs(rate) < 0.10:
            return 0.0, f"range={self._follow_range_filt:.2f}m err={err:+.2f}m rate={rate:+.2f}m/s"

        if err >= 0.0:
            r = self._follow_range_filt
            start_m = self.follow_target_ideal_m + hold
            near_m = max(start_m + 0.05, self.follow_slow_band_near_m)
            mid_m = max(near_m + 0.05, self.follow_slow_band_mid_m)
            far_m = max(mid_m + 0.05, self.follow_slow_band_far_m)

            if r <= near_m:
                t = max(0.0, min(1.0, (r - start_m) / max(0.05, near_m - start_m)))
                pwm = 70.0 + (self.follow_slow_pwm_near - 70.0) * t
            elif r <= mid_m:
                t = (r - near_m) / max(0.05, mid_m - near_m)
                pwm = self.follow_slow_pwm_near + (self.follow_slow_pwm_mid - self.follow_slow_pwm_near) * t
            elif r <= far_m:
                t = (r - mid_m) / max(0.05, far_m - mid_m)
                pwm = self.follow_slow_pwm_mid + (self.follow_slow_pwm_far - self.follow_slow_pwm_mid) * t
            else:
                extra = min(15.0, (r - far_m) * 18.0)
                pwm = self.follow_slow_pwm_far + extra

            # Only a tiny catch-up boost; keep follow at walking speed.
            pwm += max(0.0, min(10.0, rate * 8.0))
            pwm = max(0.0, min(float(self.follow_max_pwm), pwm))
            if pwm > 0.0:
                pwm = max(float(self.follow_min_move_pwm), pwm)
        else:
            pwm = -float(self.follow_back_pwm)
            if pwm < 0.0 and self.follow_allow_backoff:
                pwm = max(-float(max(self.follow_back_pwm, self.follow_min_move_pwm)), pwm)
            elif pwm < 0.0:
                pwm = 0.0

        return pwm, f"range={self._follow_range_filt:.2f}m err={err:+.2f}m rate={rate:+.2f}m/s"

    def _front_lidar_stop_confirmed(self, ignore_sector=None):
        """
        ตรวจสอบว่าต้องหยุดด้านหน้าเพราะ LIDAR หรือไม่ พร้อม debounce

        นับจำนวน sector ที่มีสิ่งกีดขวาง ถ้าเกิน follow_lidar_stop_sectors
        ต้องยืนยันนานอย่างน้อย follow_lidar_stop_confirm_s วินาที
        คืนค่า True เมื่อยืนยันแล้วว่าควร stop
        """
        count = self._dangerous_lidar_sector_count(ignore_sector)
        self._lidar_stop_sector_count = count
        now = time.time()
        if count < self.follow_lidar_stop_sectors:
            self._lidar_stop_since = 0.0
            return False
        if self._lidar_stop_since <= 0.0:
            self._lidar_stop_since = now
            return self.follow_lidar_stop_confirm_s <= 0.0
        return (now - self._lidar_stop_since) >= self.follow_lidar_stop_confirm_s

    def _lidar_vision_hold_cmd(self, reason):
        """
        ติดตามเจ้าของต่อจาก LIDAR bearing ล่าสุด เมื่อกล้องเสียหาย target ชั่วคราว

        ใช้ angle/range ของ target LIDAR ที่บันทึกไว้เพื่อขับต่อไปสั้น ๆ
        (ไม่เกิน follow_lidar_vision_hold_s วินาที)
        คืนค่า None ถ้าไม่สามารถใช้ได้หรือฟีเจอร์ถูกปิด
        """
        if not self.follow_lidar_vision_hold_enable:
            return None

        now = time.time()
        if (
            not self.lidar_safety_enabled
            or not math.isfinite(self._target_lidar_angle)
            or not math.isfinite(self._target_lidar_range)
            or (now - self._target_bearing_ts) > self.follow_lidar_vision_hold_s
        ):
            return None

        range_m = float(self._target_lidar_range)
        angle = float(self._target_lidar_angle)
        if range_m <= self.follow_close_stop_m:
            self.follow_state = "LIDAR_HOLD_CLOSE"
            return "STOP", f"LIDAR_HOLD_CLOSE {range_m:.2f}m ({reason})"

        cam_half_rad = math.radians(max(20.0, min(110.0, self.follow_camera_hfov_deg * 0.5)))
        ex = max(-1.0, min(1.0, -angle / max(1e-6, cam_half_rad)))
        deadzone_norm = max(self.follow_center_deadzone_norm, self.deadzone_px / max(1.0, self.camera_width * 0.5))

        if range_m <= self.follow_close_backoff_m and self.follow_allow_backoff:
            back = int(max(self.follow_min_move_pwm, min(self.follow_max_pwm, self.follow_back_pwm)))
            steer_back = int(max(0.0, min(self.follow_turn_pwm * 0.35, abs(ex) * self.follow_turn_pwm)))
            self.follow_state = "LIDAR_BACKOFF"
            if ex > deadzone_norm:
                return f"PWM,{-back - steer_back},{-back + steer_back}", f"LIDAR_BACKOFF_RIGHT {range_m:.2f}m ({reason})"
            if ex < -deadzone_norm:
                return f"PWM,{-back + steer_back},{-back - steer_back}", f"LIDAR_BACKOFF_LEFT {range_m:.2f}m ({reason})"
            return f"PWM,{-back},{-back}", f"LIDAR_BACKOFF {range_m:.2f}m ({reason})"

        dist_norm = (range_m - self.follow_target_ideal_m) / max(0.2, self.follow_target_cruise_m - self.follow_target_ideal_m)
        dist_norm = max(0.0, min(1.0, dist_norm))
        base, range_dbg = self._follow_distance_pwm(range_m, dist_norm)
        if abs(ex) <= deadzone_norm:
            steer_norm = 0.0
        else:
            steer_norm = max(0.0, min(1.0, (abs(ex) - deadzone_norm) / max(1e-6, 1.0 - deadzone_norm)))
        base *= max(0.62, 1.0 - 0.28 * steer_norm)
        steer_mag = 0.0
        if steer_norm > 0.0:
            steer_mag = self.follow_min_steer_pwm + (self.follow_turn_pwm - self.follow_min_steer_pwm) * (steer_norm ** self.follow_turn_exp)
        steer = math.copysign(steer_mag, ex)
        l = int(max(-self.follow_max_pwm, min(self.follow_max_pwm, base + steer)))
        r = int(max(-self.follow_max_pwm, min(self.follow_max_pwm, base - steer)))
        if abs(l) < 8 and abs(r) < 8:
            self.follow_state = "LIDAR_HOLD_DISTANCE"
            return "STOP", f"LIDAR_HOLD_DISTANCE {range_m:.2f}m {range_dbg} ({reason})"
        l, r = self._apply_follow_min_pwm(l, r)
        self.follow_state = "LIDAR_VISION_HOLD"
        return (
            f"PWM,{l},{r}",
            f"LIDAR_VISION_HOLD range={range_m:.2f}m angle={math.degrees(angle):+.0f}deg "
            f"ex={ex:+.2f} steer={steer:+.0f} {range_dbg} ({reason}) pwm=({l},{r})"
        )

    def _apply_follow_min_pwm(self, l, r):
        """
        ปรับ PWM ซ้าย/ขวาให้ไม่ต่ำกว่า deadzone ของมอเตอร์

        ถ้าค่า PWM ที่ส่งมาต่ำเกินไปจนมอเตอร์ไม่หมุน จะยกขึ้นเป็น follow_min_move_pwm
        กรณี arc turn ด้านใน (ล้อด้านในหมุนน้อย) จะใช้ follow_arc_inner_min_pwm แทน
        """
        if l == 0 and r == 0:
            return 0, 0
        arc_inner_mode = (
            l > 0
            and r > 0
            and abs(l - r) >= 55
            and min(l, r) <= max(self.follow_min_move_pwm, self.follow_arc_inner_min_pwm + 45)
        )
        min_pwm = max(0, min(self.follow_max_pwm, self.follow_min_move_pwm))
        if arc_inner_mode:
            min_pwm = max(0, min(self.follow_max_pwm, self.follow_arc_inner_min_pwm))
        if l != 0 and abs(l) < min_pwm:
            l = min_pwm if l > 0 else -min_pwm
        if r != 0 and abs(r) < min_pwm:
            r = min_pwm if r > 0 else -min_pwm
        return l, r

    def _edge_recovery_cmd(self, ex, ex_raw, range_for_drive, dist_src, label=""):
        """
        สร้างคำสั่ง PWM สำหรับ edge recovery (หมุนเพื่อให้เจ้าของกลับเข้าเฟรม)

        เรียกเมื่อเจ้าของอยู่ใกล้ขอบภาพมาก (|ex_raw| > edge_recovery_norm)
        หมุนด้วย follow_edge_turn_pwm โดยไม่คำนึงถึงระยะทาง
        เพราะถ้าไม่หมุนเร็วพอ เจ้าของจะหายออกนอกเฟรม
        """
        """Return a smooth arc command when the target is about to leave frame."""
        edge = max(0.20, min(0.95, self.follow_edge_recovery_norm))
        edge_ex = ex_raw if abs(ex_raw) >= abs(ex) else ex
        if abs(edge_ex) < edge:
            return None

        outer = int(max(self.follow_min_move_pwm, min(255, self.follow_edge_turn_pwm)))
        inner = int(max(0.0, min(outer * 0.85, outer * max(0.0, min(0.90, self.follow_edge_inner_ratio)))))
        # If already too close, rotate without forward creep; otherwise arc
        # forward so the camera keeps the person in frame instead of snapping.
        if math.isfinite(range_for_drive) and range_for_drive <= self.follow_close_backoff_m:
            inner = -inner
        else:
            inner = max(inner, int(self.follow_edge_arc_base_pwm))
        self.follow_state = "EDGE_RECOVERY"
        if edge_ex > 0:
            return (
                f"PWM,{outer},{inner}",
                f"EDGE_RECOVERY_RIGHT {label} ex={ex:+.2f} raw={ex_raw:+.2f} "
                f"range={range_for_drive:.2f}m {dist_src}"
            )
        return (
            f"PWM,{inner},{outer}",
            f"EDGE_RECOVERY_LEFT {label} ex={ex:+.2f} raw={ex_raw:+.2f} "
            f"range={range_for_drive:.2f}m {dist_src}"
        )

    def _follow_arc_pwm(self, base, steer_norm, ex):
        """
        คำนวณ PWM arc turn สำหรับเลี้ยวตามเจ้าของแบบ smooth

        ใช้ steer_norm (0-1) เพื่อกำหนดความโค้งของวงเลี้ยว
        มี yaw feedback boost: ถ้าหมุนนานแต่ yaw ไม่ตอบสนอง จะเพิ่ม differential
        คืนค่า (left_pwm, right_pwm) หรือ None ถ้าไม่ใช้ arc mode
        """
        if not self.follow_arc_turn_enable or steer_norm <= 0.0:
            self._arc_steer_dir = 0
            self._arc_steer_since = 0.0
            return None

        now = time.time()
        cur_dir = 1 if ex > 0 else -1
        if self._arc_steer_dir != cur_dir:
            self._arc_steer_dir = cur_dir
            self._arc_steer_since = now

        outer_cap = max(self.follow_min_move_pwm, min(255, self.follow_arc_outer_pwm))
        outer = max(
            float(self.follow_arc_base_min_pwm),
            abs(float(base)),
            float(self.follow_min_move_pwm),
        )
        outer += (outer_cap - outer) * max(0.0, min(1.0, steer_norm))
        outer = max(float(self.follow_min_move_pwm), min(float(outer_cap), outer))

        inner_ratio = 1.0 - (1.0 - self.follow_arc_inner_ratio_min) * max(0.0, min(1.0, steer_norm))
        inner = max(float(self.follow_arc_inner_min_pwm), outer * inner_ratio)
        inner = min(outer * 0.92, inner)

        # Yaw feedback boost: if we've been steering this direction but the robot
        # isn't actually rotating (PWM difference not enough to overcome friction),
        # increase outer and reduce inner until yaw responds.
        if (self._yaw_ts > 0.0
                and self._arc_steer_since > 0.0
                and (now - self._arc_steer_since) > self.follow_yaw_boost_delay_s):
            # In ROS: positive yaw = CCW = left. ex>0 means person is right → expect yaw_rate < 0.
            expected_yaw_sign = -1 if ex > 0 else 1
            yaw_turning = (
                abs(self._yaw_rate) >= self.follow_yaw_min_rate * steer_norm
                and (self._yaw_rate * expected_yaw_sign) > 0
            )
            if not yaw_turning:
                outer = min(float(outer_cap), outer * self.follow_yaw_boost_outer_scale)
                inner = max(0.0, inner * self.follow_yaw_boost_inner_scale)

        if ex > 0.0:
            return int(round(outer)), int(round(inner))
        return int(round(inner)), int(round(outer))

    def _zone_follow_ex(self, ex):
        """
        แปลงค่า ex ต่อเนื่อง (-1 ถึง +1) ให้เป็นค่าแบบ discrete zone

        ถ้า follow_steer_zones >= 3 จะแบ่ง ex ออกเป็น N zones แบบ step function
        เพื่อลด jitter เมื่อเจ้าของอยู่ใกล้ center zone (ตรงกลางถือว่า "ตรง")
        """
        zones = int(self.follow_steer_zones)
        if zones < 3:
            return max(-1.0, min(1.0, ex))
        # Force an odd zone count so one exact center zone can be straight.
        if zones % 2 == 0:
            zones += 1
        ex = max(-1.0, min(1.0, ex))
        idx = int(round((ex + 1.0) * 0.5 * (zones - 1)))
        return max(-1.0, min(1.0, (idx / max(1, zones - 1)) * 2.0 - 1.0))

    def _fused_follow_range_m(self, lidar_m, camera_lidar_m):
        """
        รวมระยะจาก LIDAR และจากการประมาณกล้อง (camera-based range)

        ถ้ามีทั้งสองค่า จะ weighted average ด้วย follow_lidar_range_weight
        ถ้าฝ่ายใดฝ่ายหนึ่งใกล้มาก (≤ follow_close_stop_m) จะใช้ค่าน้อยกว่าแทน
        คืนค่า: ระยะที่ fused แล้ว (เมตร)
        """
        lidar_ok = math.isfinite(lidar_m)
        cam_ok = math.isfinite(camera_lidar_m)
        if lidar_ok and cam_ok:
            if lidar_m <= self.follow_close_stop_m or camera_lidar_m <= self.follow_close_stop_m:
                return min(lidar_m, camera_lidar_m)
            w = max(0.0, min(1.0, self.follow_lidar_range_weight))
            return w * lidar_m + (1.0 - w) * camera_lidar_m
        if lidar_ok:
            return lidar_m
        return camera_lidar_m

    def _detect_people_yolo(self, frame):
        """
        ตรวจจับคนในภาพด้วย YOLO model

        ส่ง frame ขนาดเล็ก (follow_infer_width) เข้า YOLO เพื่อลด latency
        กรองเฉพาะ class "person" (class_id=0) และ confidence > yolo_conf
        กรองกล่องที่เล็กเกินไป (< yolo_min_box_h × yolo_min_box_w)
        return: list ของ dict {cx, cy, bbox, track_id, conf}
        """
        if self.yolo is None:
            return []
        try:
            kwargs = {
                "imgsz": self.yolo_imgsz,
                "conf": self.yolo_conf,
                "classes": [0],
                "verbose": False,
            }
            device = os.getenv("FOLLOW_YOLO_DEVICE", "").strip()
            if device:
                kwargs["device"] = device
            if self.yolo_track:
                results = self.yolo.track(frame, persist=True, tracker=self.yolo_tracker_cfg, **kwargs)
            else:
                results = self.yolo.predict(frame, **kwargs)
        except Exception as e:
            if time.time() - self._last_cam_err_log_ts > 2.0:
                self._last_cam_err_log_ts = time.time()
                self.get_logger().warn(f"YOLO predict failed: {e}")
            return []

        people = []
        if not results:
            return people
        h, w = frame.shape[:2]
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return people
        for b in boxes:
            try:
                xyxy = b.xyxy[0].detach().cpu().numpy().astype(float)
                conf = float(b.conf[0].detach().cpu().item()) if b.conf is not None else 0.0
                track_id = None
                if getattr(b, "id", None) is not None:
                    track_id = int(b.id[0].detach().cpu().item())
            except Exception:
                continue
            x1, y1, x2, y2 = xyxy
            x1 = max(0, min(w - 1, int(x1)))
            y1 = max(0, min(h - 1, int(y1)))
            x2 = max(0, min(w - 1, int(x2)))
            y2 = max(0, min(h - 1, int(y2)))
            bw = x2 - x1
            bh = y2 - y1
            if bw < self.yolo_min_box_w or bh < self.yolo_min_box_h:
                continue
            people.append({
                "bbox": (x1, y1, x2, y2),
                "conf": conf,
                "cx": 0.5 * (x1 + x2),
                "cy": 0.5 * (y1 + y2),
                "bw": bw,
                "bh": bh,
                "track_id": track_id,
            })
        return people

    def _select_yolo_target(self, frame, people):
        """
        เลือก target เจ้าของจากรายการคนที่ YOLO ตรวจจับได้

        ลำดับการเลือก:
        1. ถ้ามีเพียงคนเดียว → เปรียบเทียบกับ profile
        2. ถ้ามีหลายคน → ใช้ ReID score + track continuity เลือกที่ดีที่สุด
        3. ถ้า score เกิน threshold → LOST (ไม่ตรงกับ profile เลย)
        return: (target_dict, status_label)
        """
        if not people:
            self.follow_state = "LOST"
            return None, "NO_TARGET"
        self.last_person_seen_ts = time.time()
        if not self.selected_profile or not self.profile_refs:
            self.follow_state = "NO_OWNER"
            return None, "SELECT OWNER FIRST"

        h, w = frame.shape[:2]
        now = time.time()
        recent_target = (
            self._last_target_cx is not None
            and (now - self._last_target_ts) <= self.follow_track_hold_s
        )
        predicted_cx = self._predict_target_cx(w, now) if recent_target else None
        reacquire_window_px = max(50.0, self.follow_reacquire_window_norm * w)
        scored = []
        for p in people:
            bbox = p["bbox"]
            profile_score = self._match_selected_profile(frame, bbox)
            if profile_score is None:
                profile_score = 9.99
            center_penalty = 0.05 * abs((p["cx"] - (w * 0.5)) / max(1.0, w * 0.5))
            continuity_penalty = 0.0
            if predicted_cx is not None:
                continuity_penalty = self.follow_reacquire_continuity_weight * abs(p["cx"] - predicted_cx) / max(1.0, w)
            track_bonus = 0.0
            if self._last_track_id is not None and p.get("track_id") == self._last_track_id:
                track_bonus = self.follow_reacquire_track_bonus
            # Lower score is better. Owner color matters most, then continuity,
            # then detection confidence/center as tie breakers.
            total = profile_score + center_penalty + continuity_penalty - (0.10 * p["conf"]) - track_bonus
            scored.append((total, profile_score, p, track_bonus))

        scored.sort(key=lambda x: x[0])
        best_total, profile_score, target, track_bonus = scored[0]
        self._last_profile_score = profile_score

        owner_ok = profile_score <= self.follow_profile_match_max
        strong_owner_ok = profile_score <= max(0.05, self.follow_profile_match_max - self.follow_owner_ambiguity_margin)
        if len(scored) > 1 and not strong_owner_ok:
            second_total = scored[1][0]
            if (second_total - best_total) < self.follow_owner_ambiguity_margin:
                self._profile_lock_count = 0
                self.follow_state = "AMBIGUOUS_OWNER"
                return None, (
                    f"AMBIGUOUS_OWNER {self.selected_profile} best={profile_score:.2f} "
                    f"gap={second_total - best_total:.2f} min_gap={self.follow_owner_ambiguity_margin:.2f}"
                )
        if recent_target and predicted_cx is not None and not strong_owner_ok and track_bonus <= 0.0:
            if abs(target["cx"] - predicted_cx) > reacquire_window_px:
                self._profile_lock_count = 0
                self.follow_state = "REACQUIRE_WAIT"
                return None, (
                    f"REACQUIRE_WAIT {self.selected_profile} d={profile_score:.2f} "
                    f"jump={abs(target['cx'] - predicted_cx):.0f}px max={reacquire_window_px:.0f}px"
                )
        loose_owner_ok = self.follow_allow_loose_owner_match and (
            (len(people) == 1 and profile_score <= self.follow_profile_loose_single_max)
            or (recent_target and profile_score <= self.follow_profile_loose_recent_max)
        )
        weak_ok = self.follow_allow_weak_owner_match and (
            len(people) == 1 or self.follow_allow_weak_owner_multi or recent_target
        )
        if owner_ok:
            self._profile_accept_until = now + self.follow_profile_accept_hold_s
            self._profile_lock_count = min(self.follow_profile_lock_frames + 1, self._profile_lock_count + 1)
        elif now <= self._profile_accept_until or weak_ok or loose_owner_ok:
            self._profile_lock_count = self.follow_profile_lock_frames
        else:
            self._profile_lock_count = 0
            self.follow_state = "WRONG_OWNER"
            return None, (
                f"WRONG_TARGET {self.selected_profile} d={profile_score:.2f} "
                f"max={self.follow_profile_match_max:.2f} via={self._last_profile_match_mode} people={len(people)}"
            )

        if self._profile_lock_count < self.follow_profile_lock_frames:
            self.follow_state = "LOCKING"
            return None, (
                f"LOCKING {self.selected_profile} d={profile_score:.2f} "
                f"via={self._last_profile_match_mode} n={self._profile_lock_count}/{self.follow_profile_lock_frames}"
            )

        self._remember_yolo_target(target, now)
        return (target, profile_score, owner_ok or loose_owner_ok), ""

    def _drive_from_target(self, w, cx, apparent_px, profile_score, src_label):
        """
        คำนวณและส่งคำสั่ง PWM จากตำแหน่งเจ้าของในภาพ

        ขั้นตอน:
        1. คำนวณ ex (normalized horizontal error: -1=ซ้าย, +1=ขวา)
        2. ตรวจสอบ edge recovery ถ้าเจ้าของใกล้ขอบ
        3. ตรวจสอบ LIDAR safety ถ้าอยู่ใกล้เกินไป
        4. คำนวณ drive PWM จากระยะทาง (LIDAR หรือ camera estimate)
        5. คำนวณ steer PWM จาก ex และส่งคำสั่ง arc turn หรือ pivot turn
        6. Apply slew rate limiter แล้วส่งผ่าน _send_cmd
        """
        self.last_seen_ts = time.time()

        ex_raw = ((cx - (w * 0.5)) / max(1.0, w * 0.5))
        ex_raw = max(-1.0, min(1.0, ex_raw))
        self._last_target_ex = ex_raw

        if self._ex_filt is None:
            self._ex_filt = ex_raw
        else:
            a = max(0.01, min(1.0, self.follow_ex_alpha))
            self._ex_filt = (1.0 - a) * self._ex_filt + a * ex_raw
        if self._shoulder_filt is None:
            self._shoulder_filt = apparent_px
        else:
            a = max(0.01, min(1.0, self.follow_shoulder_alpha))
            self._shoulder_filt = (1.0 - a) * self._shoulder_filt + a * apparent_px

        ex = self._ex_filt
        shoulder_px = self._shoulder_filt
        cam_est_m, cam_est_lidar_m = self._estimate_lidar_distance_from_camera_size(shoulder_px)
        if shoulder_px < self.follow_min_drive_apparent_px or cam_est_lidar_m > self.follow_max_camera_target_m:
            self.follow_state = "FAR_TARGET"
            return (
                "STOP",
                f"FAR_TARGET {src_label} size={shoulder_px:.0f}px "
                f"cam_est={cam_est_m:.2f}m lidar_est={cam_est_lidar_m:.2f}m"
            )

        cam_half_rad = math.radians(max(20.0, min(110.0, self.follow_camera_hfov_deg * 0.5)))
        camera_bearing_rad = -ex * cam_half_rad
        lidar_target_angle, lidar_target_m, lidar_locked = self._pick_lidar_target(camera_bearing_rad)
        lidar_reject_reason = ""
        if self.follow_lidar_plausibility and math.isfinite(lidar_target_m):
            ratio = lidar_target_m / max(0.1, cam_est_lidar_m)
            if ratio < self.follow_lidar_plaus_min_ratio or ratio > self.follow_lidar_plaus_max_ratio:
                lidar_reject_reason = (
                    f" lidar_reject={lidar_target_m:.2f}m/cam{cam_est_m:.2f}m/lidar_est{cam_est_lidar_m:.2f}m"
                    f" ratio={ratio:.2f}"
                )
                lidar_target_angle = float("nan")
                lidar_target_m = float("inf")
                lidar_locked = False
                self._target_lidar_angle = float("nan")
                self._target_lidar_range = float("inf")
        if not lidar_locked:
            self._target_bearing_rad = camera_bearing_rad
            self._target_bearing_ts = time.time()

        follow_bearing_rad = camera_bearing_rad
        if math.isfinite(lidar_target_angle):
            lidar_ex = max(-1.0, min(1.0, -lidar_target_angle / max(1e-6, cam_half_rad)))
            w_lidar = max(0.0, min(1.0, self.follow_lidar_bearing_weight))
            if math.isfinite(lidar_target_m) and lidar_target_m <= self.follow_lidar_blob_weight_close_m:
                w_lidar = max(w_lidar, max(0.0, min(1.0, self.follow_lidar_bearing_weight_close_max)))
            ex = max(-1.0, min(1.0, (1.0 - w_lidar) * ex + w_lidar * lidar_ex))
            follow_bearing_rad = lidar_target_angle
        ex = self._zone_follow_ex(ex)

        front_half = math.radians(self.follow_lidar_front_sector_deg * 0.5)
        sector_pos = (follow_bearing_rad + front_half) / max(1e-6, 2.0 * front_half)
        target_sector = int(sector_pos * self.follow_lidar_sector_count)
        target_sector = max(0, min(self.follow_lidar_sector_count - 1, target_sector))
        target_lidar_m = lidar_target_m if math.isfinite(lidar_target_m) else float("inf")

        center_mid = self.follow_lidar_sector_count // 2
        center_half = max(1, self.follow_lidar_sector_count // 6)
        center_lo = max(0, center_mid - center_half)
        center_hi = min(self.follow_lidar_sector_count - 1, center_mid + center_half)
        center_sector_ranges = [
            self.lidar_sector_min[i]
            for i in range(center_lo, center_hi + 1)
            if math.isfinite(self.lidar_sector_min[i])
        ]
        center_lidar_m = min(center_sector_ranges) if center_sector_ranges else float("inf")

        near_limit = self.target_shoulder_px + self.shoulder_tol_px
        if shoulder_px >= near_limit and not math.isfinite(target_lidar_m):
            self.follow_state = "HOLD_DISTANCE"
            return "STOP", f"HOLD {src_label} size={shoulder_px:.0f}px"

        far_ref = max(40.0, self.target_shoulder_px * 0.35)
        dist_norm_cam = (near_limit - shoulder_px) / max(1.0, near_limit - far_ref)
        dist_norm_cam = max(0.0, min(1.0, dist_norm_cam))

        if math.isfinite(target_lidar_m):
            lock_tag = 'L' if lidar_locked else 'H'
            dist_src = f"target_lidar={target_lidar_m:.2f}m@{math.degrees(lidar_target_angle):+.0f}deg[{lock_tag}]"
            camera_says_far = cam_est_lidar_m > (self.follow_target_ideal_m * 1.25)
            # A LiDAR return in the target sector is the owner, not an obstacle.
            # Use it as following distance: too close -> back up, ideal -> hold,
            # far -> move forward. Obstacle stopping is checked later outside the
            # ignored target corridor.
            if target_lidar_m <= self.follow_target_ideal_m and camera_says_far:
                # If the camera box is still small, a very near LiDAR return is
                # probably a handlebar/pole/noise in the same sector. Keep moving
                # gently from camera distance instead of freezing in HOLD_TARGET.
                dist_norm = dist_norm_cam
                dist_src += f" camera_override cam_est={cam_est_m:.2f}m lidar_est={cam_est_lidar_m:.2f}m"
            else:
                dist_norm = (target_lidar_m - self.follow_target_ideal_m) / max(0.2, self.follow_target_cruise_m - self.follow_target_ideal_m)
            dist_norm = max(0.0, min(1.0, dist_norm))
        else:
            dist_norm = dist_norm_cam
            dist_src = f"{src_label}_size={shoulder_px:.0f}px cam_est={cam_est_m:.2f}m lidar_est={cam_est_lidar_m:.2f}m{lidar_reject_reason}"

        range_for_drive = self._fused_follow_range_m(target_lidar_m, cam_est_lidar_m)
        deadzone_norm = max(self.follow_center_deadzone_norm, self.deadzone_px / max(1.0, w * 0.5))

        if range_for_drive <= self.follow_close_stop_m:
            self.follow_state = "CLOSE_STOP"
            return "STOP", f"CLOSE_STOP range={range_for_drive:.2f}m limit={self.follow_close_stop_m:.2f}m {dist_src}"

        if range_for_drive <= self.follow_close_backoff_m and self.follow_allow_backoff:
            self.follow_state = "BACKOFF"
            back = int(max(self.follow_min_move_pwm, min(self.follow_max_pwm, self.follow_back_pwm)))
            steer_back = int(max(0.0, min(self.follow_turn_pwm * 0.45, abs(ex) * self.follow_turn_pwm)))
            if ex > deadzone_norm:
                return f"PWM,{-back - steer_back},{-back + steer_back}", f"BACKOFF_RIGHT range={range_for_drive:.2f}m {dist_src}"
            if ex < -deadzone_norm:
                return f"PWM,{-back + steer_back},{-back - steer_back}", f"BACKOFF_LEFT range={range_for_drive:.2f}m {dist_src}"
            return f"PWM,{-back},{-back}", f"BACKOFF range={range_for_drive:.2f}m {dist_src}"

        edge_cmd = self._edge_recovery_cmd(ex, ex_raw, range_for_drive, dist_src, src_label)
        if edge_cmd is not None:
            return edge_cmd

        base, range_dbg = self._follow_distance_pwm(range_for_drive, dist_norm)

        if abs(ex) <= deadzone_norm:
            steer_norm = 0.0
        else:
            steer_norm = (abs(ex) - deadzone_norm) / max(1e-6, 1.0 - deadzone_norm)
            steer_norm = max(0.0, min(1.0, steer_norm))

        base *= max(0.58, 1.0 - 0.35 * steer_norm)
        if steer_norm > 0.0:
            steer_mag = self.follow_min_steer_pwm + (self.follow_turn_pwm - self.follow_min_steer_pwm) * (steer_norm ** self.follow_turn_exp)
        else:
            steer_mag = 0.0
        steer = math.copysign(steer_mag, ex)

        cross_block = (
            self.lidar_safety_enabled
            and math.isfinite(center_lidar_m)
            and center_lidar_m < self.follow_cross_block_m
            and abs(target_sector - center_mid) > (self.follow_target_lidar_window + 1)
        )
        now = time.time()
        if cross_block:
            self._cross_block_until = now + self.follow_cross_hold_s
        if now < self._cross_block_until:
            self.follow_state = "OBSTACLE_WAIT"
            return "STOP", f"CROSS_BLOCK center={center_lidar_m:.2f}m target_sec={target_sector}"

        hard_stop_m = self._hard_lidar_stop_range()
        if self.lidar_safety_enabled and math.isfinite(hard_stop_m):
            self.follow_state = "OBSTACLE_WAIT"
            return "STOP", f"LIDAR_HARD_STOP {hard_stop_m:.2f}m limit={self.follow_lidar_hard_stop_m:.2f}m"

        if self.lidar_safety_enabled and self._front_lidar_stop_confirmed(target_sector):
            self.follow_state = "OBSTACLE_WAIT"
            return "STOP", f"LIDAR_STOP {self.lidar_min_front_m:.2f}m sectors={self._lidar_stop_sector_count}"

        if steer_norm >= self.follow_turn_in_place_norm and not self.follow_arc_turn_enable:
            self.follow_state = "ALIGNING"
            turn = int(max(self.follow_turn_pwm, min(self.follow_max_pwm, abs(steer_mag))))
            if ex > 0:
                return f"PWM,{turn},{-turn}", f"TURN_ALIGN {src_label} ex={ex:+.2f} raw={ex_raw:+.2f} steer_norm={steer_norm:.2f} sec={target_sector} {dist_src}"
            return f"PWM,{-turn},{turn}", f"TURN_ALIGN {src_label} ex={ex:+.2f} raw={ex_raw:+.2f} steer_norm={steer_norm:.2f} sec={target_sector} {dist_src}"

        nearest_obstacle_m = self._nearest_lidar_range(target_sector)
        if base > 0.0 and self.lidar_safety_enabled and math.isfinite(nearest_obstacle_m) and nearest_obstacle_m <= self.follow_soft_stop_m:
            slow_scale = max(0.0, min(1.0, (nearest_obstacle_m - self.stop_dist_m) / max(0.10, self.follow_soft_stop_m - self.stop_dist_m)))
            base *= slow_scale

        if abs(base) < 8.0 and dist_norm < self.follow_min_drive_ratio and steer_norm < 0.12:
            self.follow_state = "HOLD_DISTANCE"
            if math.isfinite(target_lidar_m):
                return "STOP", f"HOLD_TARGET {dist_src} ideal={self.follow_target_ideal_m:.2f}m"
            return "STOP", f"HOLD {src_label} size={shoulder_px:.0f}px"

        arc_pwm = self._follow_arc_pwm(base, steer_norm, ex)
        if arc_pwm is not None:
            l, r = arc_pwm
            self.follow_state = "ARC_FOLLOW"
        else:
            l = int(max(-self.follow_max_pwm, min(self.follow_max_pwm, base + steer)))
            r = int(max(-self.follow_max_pwm, min(self.follow_max_pwm, base - steer)))
        if abs(l) < 8 and abs(r) < 8:
            self.follow_state = "HOLD_DISTANCE"
            if math.isfinite(target_lidar_m):
                return "STOP", f"HOLD_TARGET {dist_src} ideal={self.follow_target_ideal_m:.2f}m"
            return "STOP", f"HOLD {src_label} size={shoulder_px:.0f}px"
        l, r = self._apply_follow_min_pwm(l, r)
        self.follow_state = "FOLLOWING"
        return (
            f"PWM,{l},{r}",
            f"TRACK/{src_label} owner={self.selected_profile} d={profile_score:.2f} "
            f"via={self._last_profile_match_mode} "
            f"ex={ex:+.2f} raw={ex_raw:+.2f} steer_norm={steer_norm:.2f} state={self.follow_state} "
            f"bear={math.degrees(follow_bearing_rad):+.1f}deg steer={steer:+.0f} "
            f"sec={target_sector} {dist_src} {range_dbg} dist={dist_norm:.2f} pwm=({l},{r})"
        )

    def _decide_cmd_yolo(self, frame):
        """
        ตัดสินใจคำสั่งขับเคลื่อนโดยใช้ YOLO detector

        ตรวจจับคน → เลือก target → คำนวณ PWM ผ่าน _drive_from_target
        return: (cmd_string, status_label)
        """
        people = self._detect_people_yolo(frame)
        self._last_people_count = len(people)
        selected, reason = self._select_yolo_target(frame, people)
        if selected is None:
            return "STOP", reason
        target, profile_score, owner_ok = selected
        x1, y1, x2, y2 = target["bbox"]
        apparent_px = max(
            24.0,
            min(self.follow_target_bbox_width_px * 1.8, (x2 - x1) * 0.72 + (y2 - y1) * 0.18)
        )
        tag = "YOLO" if owner_ok else "YOLO_WEAK_OWNER"
        return self._drive_from_target(frame.shape[1], target["cx"], apparent_px, profile_score, tag)

    def _decide_cmd(self, frame):
        """
        ตัดสินใจคำสั่งขับเคลื่อนหลัก — เลือก detector และรวมผลลัพธ์

        ลำดับ:
        1. ปรับ contrast ของเฟรม
        2. ลอง YOLO ก่อน (ถ้ามี)
        3. Fallback ไป MediaPipe ถ้า YOLO ไม่เจอคน
        4. return (cmd, status)
        """
        vision_frame = self._auto_contrast_frame(frame)
        if self.yolo is not None and self.detector_mode in ("auto", "yolo"):
            cmd, status = self._decide_cmd_yolo(vision_frame)
            # In auto mode, only fallback to MediaPipe when YOLO sees no person.
            # If YOLO sees a wrong owner/obstacle/hold state, keep that decision.
            if self.detector_mode == "yolo" or status not in ("NO_TARGET",):
                return cmd, status

        if self.pose is None:
            return "STOP", "NO_MEDIAPIPE"

        h, w = vision_frame.shape[:2]
        pose_frame = vision_frame
        proc = pose_frame
        scale = 1.0
        if self.follow_infer_width > 0 and w > self.follow_infer_width:
            scale = self.follow_infer_width / float(w)
            proc = cv2.resize(pose_frame, (self.follow_infer_width, max(96, int(h * scale))), interpolation=cv2.INTER_AREA)
        ph, pw = proc.shape[:2]
        rgb = cv2.cvtColor(proc, cv2.COLOR_BGR2RGB)
        result = self.pose.process(rgb)
        if not result.pose_landmarks:
            return "STOP", "NO_TARGET"

        lm = result.pose_landmarks.landmark
        l_sh, r_sh = lm[11], lm[12]
        if l_sh.visibility < self.follow_min_visibility or r_sh.visibility < self.follow_min_visibility:
            return "STOP", "LOW_VIS"
        if self.follow_strict_human_pose and not self._pose_is_human_like(lm, pw, ph):
            return "STOP", "REJECT_NON_HUMAN"
        bbox_small = self._pose_bbox(lm, pw, ph)
        bbox = None
        if bbox_small is not None:
            x1, y1, x2, y2 = bbox_small
            inv = 1.0 / max(scale, 1e-6)
            bbox = (
                max(0, int(x1 * inv)),
                max(0, int(y1 * inv)),
                min(w - 1, int(x2 * inv)),
                min(h - 1, int(y2 * inv)),
            )
        if bbox is None:
            return "STOP", "NO_BBOX"
        # A human pose is visible. Keep this separate from owner-confirmed target,
        # otherwise owner-lock failures get mislabeled as TARGET LOST.
        self.last_person_seen_ts = time.time()
        if not self.selected_profile or not self.profile_refs:
            return "STOP", "SELECT OWNER FIRST"
        profile_score = self._match_selected_profile(vision_frame, bbox)
        now = time.time()
        if profile_score is None:
            if self.follow_allow_weak_owner_match:
                profile_score = 9.99
            else:
                return "STOP", f"NO_PROFILE_REF {self.selected_profile}"
        self._last_profile_score = profile_score
        loose_owner_ok = self.follow_allow_loose_owner_match and profile_score <= self.follow_profile_loose_single_max
        if profile_score <= self.follow_profile_match_max:
            self._profile_accept_until = now + self.follow_profile_accept_hold_s
            self._profile_lock_count = min(self.follow_profile_lock_frames + 1, self._profile_lock_count + 1)
        elif now > self._profile_accept_until:
            if self.follow_allow_weak_owner_match or loose_owner_ok:
                self._profile_lock_count = self.follow_profile_lock_frames
            else:
                self._profile_lock_count = 0
                return "STOP", f"WRONG_TARGET {self.selected_profile} d={profile_score:.2f} max={self.follow_profile_match_max:.2f} via={self._last_profile_match_mode}"
        else:
            self._profile_lock_count = max(0, self._profile_lock_count - 1)

        if self._profile_lock_count < self.follow_profile_lock_frames:
            return "STOP", f"LOCKING {self.selected_profile} d={profile_score:.2f} via={self._last_profile_match_mode} n={self._profile_lock_count}/{self.follow_profile_lock_frames}"

        cx = int(((l_sh.x + r_sh.x) * 0.5) * pw / max(scale, 1e-6))
        shoulder_px_raw = abs(l_sh.x - r_sh.x) * pw / max(scale, 1e-6)
        self.last_seen_ts = time.time()

        # Normalized horizontal error from image center.
        ex_raw = ((cx - (w * 0.5)) / max(1.0, w * 0.5))
        ex_raw = max(-1.0, min(1.0, ex_raw))

        if self._ex_filt is None:
            self._ex_filt = ex_raw
        else:
            a = max(0.01, min(1.0, self.follow_ex_alpha))
            self._ex_filt = (1.0 - a) * self._ex_filt + a * ex_raw
        if self._shoulder_filt is None:
            self._shoulder_filt = shoulder_px_raw
        else:
            a = max(0.01, min(1.0, self.follow_shoulder_alpha))
            self._shoulder_filt = (1.0 - a) * self._shoulder_filt + a * shoulder_px_raw

        ex = self._ex_filt
        shoulder_px = self._shoulder_filt
        cam_est_m, cam_est_lidar_m = self._estimate_lidar_distance_from_camera_size(shoulder_px)
        if shoulder_px < self.follow_min_drive_apparent_px or cam_est_lidar_m > self.follow_max_camera_target_m:
            return (
                "STOP",
                f"FAR_TARGET pose size={shoulder_px:.0f}px "
                f"cam_est={cam_est_m:.2f}m lidar_est={cam_est_lidar_m:.2f}m"
            )

        cam_half_rad = math.radians(max(20.0, min(110.0, self.follow_camera_hfov_deg * 0.5)))
        camera_bearing_rad = -ex * cam_half_rad
        lidar_target_angle, lidar_target_m, lidar_locked = self._pick_lidar_target(camera_bearing_rad)
        lidar_reject_reason = ""
        if self.follow_lidar_plausibility and math.isfinite(lidar_target_m):
            ratio = lidar_target_m / max(0.1, cam_est_lidar_m)
            if ratio < self.follow_lidar_plaus_min_ratio or ratio > self.follow_lidar_plaus_max_ratio:
                lidar_reject_reason = (
                    f" lidar_reject={lidar_target_m:.2f}m/cam{cam_est_m:.2f}m/lidar_est{cam_est_lidar_m:.2f}m"
                    f" ratio={ratio:.2f}"
                )
                lidar_target_angle = float("nan")
                lidar_target_m = float("inf")
                lidar_locked = False
                self._target_lidar_angle = float("nan")
                self._target_lidar_range = float("inf")
        if not lidar_locked:
            self._target_bearing_rad = camera_bearing_rad
            self._target_bearing_ts = time.time()

        follow_bearing_rad = camera_bearing_rad
        if math.isfinite(lidar_target_angle):
            lidar_ex = max(-1.0, min(1.0, -lidar_target_angle / max(1e-6, cam_half_rad)))
            w_lidar = max(0.0, min(1.0, self.follow_lidar_bearing_weight))
            if math.isfinite(lidar_target_m) and lidar_target_m <= self.follow_lidar_blob_weight_close_m:
                w_lidar = max(w_lidar, max(0.0, min(1.0, self.follow_lidar_bearing_weight_close_max)))
            ex = max(-1.0, min(1.0, (1.0 - w_lidar) * ex + w_lidar * lidar_ex))
            follow_bearing_rad = lidar_target_angle
        ex = self._zone_follow_ex(ex)

        front_half = math.radians(self.follow_lidar_front_sector_deg * 0.5)
        sector_pos = (follow_bearing_rad + front_half) / max(1e-6, 2.0 * front_half)
        target_sector = int(sector_pos * self.follow_lidar_sector_count)
        target_sector = max(0, min(self.follow_lidar_sector_count - 1, target_sector))
        target_lidar_m = lidar_target_m if math.isfinite(lidar_target_m) else float("inf")

        center_mid = self.follow_lidar_sector_count // 2
        center_half = max(1, self.follow_lidar_sector_count // 6)
        center_lo = max(0, center_mid - center_half)
        center_hi = min(self.follow_lidar_sector_count - 1, center_mid + center_half)
        center_sector_ranges = [
            self.lidar_sector_min[i]
            for i in range(center_lo, center_hi + 1)
            if math.isfinite(self.lidar_sector_min[i])
        ]
        center_lidar_m = min(center_sector_ranges) if center_sector_ranges else float("inf")

        near_limit = self.target_shoulder_px + self.shoulder_tol_px
        if shoulder_px >= near_limit and not math.isfinite(target_lidar_m):
            return "STOP", f"HOLD sh={shoulder_px:.0f}px"

        far_ref = max(40.0, self.target_shoulder_px * 0.35)
        dist_norm_cam = (near_limit - shoulder_px) / max(1.0, near_limit - far_ref)
        dist_norm_cam = max(0.0, min(1.0, dist_norm_cam))

        if math.isfinite(target_lidar_m):
            lock_tag = 'L' if lidar_locked else 'H'
            dist_src = f"target_lidar={target_lidar_m:.2f}m@{math.degrees(lidar_target_angle):+.0f}deg[{lock_tag}]"
            camera_says_far = cam_est_lidar_m > (self.follow_target_ideal_m * 1.25)
            if target_lidar_m <= self.follow_target_ideal_m and camera_says_far:
                dist_norm = dist_norm_cam
                dist_src += f" camera_override cam_est={cam_est_m:.2f}m lidar_est={cam_est_lidar_m:.2f}m"
            else:
                dist_norm = (target_lidar_m - self.follow_target_ideal_m) / max(0.2, self.follow_target_cruise_m - self.follow_target_ideal_m)
            dist_norm = max(0.0, min(1.0, dist_norm))
        else:
            dist_norm = dist_norm_cam
            dist_src = f"cam_sh={shoulder_px:.0f}px cam_est={cam_est_m:.2f}m lidar_est={cam_est_lidar_m:.2f}m{lidar_reject_reason}"

        range_for_drive = self._fused_follow_range_m(target_lidar_m, cam_est_lidar_m)
        deadzone_norm = max(self.follow_center_deadzone_norm, self.deadzone_px / max(1.0, w * 0.5))

        if range_for_drive <= self.follow_close_stop_m:
            return "STOP", f"CLOSE_STOP range={range_for_drive:.2f}m limit={self.follow_close_stop_m:.2f}m {dist_src}"

        if range_for_drive <= self.follow_close_backoff_m and self.follow_allow_backoff:
            back = int(max(self.follow_min_move_pwm, min(self.follow_max_pwm, self.follow_back_pwm)))
            steer_back = int(max(0.0, min(self.follow_turn_pwm * 0.45, abs(ex) * self.follow_turn_pwm)))
            if ex > deadzone_norm:
                return f"PWM,{-back - steer_back},{-back + steer_back}", f"BACKOFF_RIGHT range={range_for_drive:.2f}m {dist_src}"
            if ex < -deadzone_norm:
                return f"PWM,{-back + steer_back},{-back - steer_back}", f"BACKOFF_LEFT range={range_for_drive:.2f}m {dist_src}"
            return f"PWM,{-back},{-back}", f"BACKOFF range={range_for_drive:.2f}m {dist_src}"

        edge_cmd = self._edge_recovery_cmd(ex, ex_raw, range_for_drive, dist_src, "pose")
        if edge_cmd is not None:
            return edge_cmd

        base, range_dbg = self._follow_distance_pwm(range_for_drive, dist_norm)

        if abs(ex) <= deadzone_norm:
            steer_norm = 0.0
        else:
            steer_norm = (abs(ex) - deadzone_norm) / max(1e-6, 1.0 - deadzone_norm)
            steer_norm = max(0.0, min(1.0, steer_norm))

        # Continuous steering: small image offset gives small differential PWM,
        # large offset gives strong turn. Keep some forward drive while steering.
        base *= max(0.58, 1.0 - 0.35 * steer_norm)
        if steer_norm > 0.0:
            steer_mag = self.follow_min_steer_pwm + (self.follow_turn_pwm - self.follow_min_steer_pwm) * (steer_norm ** self.follow_turn_exp)
        else:
            steer_mag = 0.0
        steer = math.copysign(steer_mag, ex)

        cross_block = (
            self.lidar_safety_enabled
            and math.isfinite(center_lidar_m)
            and center_lidar_m < self.follow_cross_block_m
            and abs(target_sector - center_mid) > (self.follow_target_lidar_window + 1)
        )
        now = time.time()
        if cross_block:
            self._cross_block_until = now + self.follow_cross_hold_s
        if now < self._cross_block_until:
            return "STOP", f"CROSS_BLOCK center={center_lidar_m:.2f}m target_sec={target_sector}"

        hard_stop_m = self._hard_lidar_stop_range()
        if self.lidar_safety_enabled and math.isfinite(hard_stop_m):
            return "STOP", f"LIDAR_HARD_STOP {hard_stop_m:.2f}m limit={self.follow_lidar_hard_stop_m:.2f}m"

        if self.lidar_safety_enabled and self._front_lidar_stop_confirmed(target_sector):
            return "STOP", f"LIDAR_STOP {self.lidar_min_front_m:.2f}m sectors={self._lidar_stop_sector_count}"

        if steer_norm >= self.follow_turn_in_place_norm and not self.follow_arc_turn_enable:
            turn = int(max(self.follow_turn_pwm, min(self.follow_max_pwm, abs(steer_mag))))
            if ex > 0:
                return f"PWM,{turn},{-turn}", f"TURN_ALIGN ex={ex:+.2f} raw={ex_raw:+.2f} steer_norm={steer_norm:.2f} sec={target_sector} {dist_src}"
            return f"PWM,{-turn},{turn}", f"TURN_ALIGN ex={ex:+.2f} raw={ex_raw:+.2f} steer_norm={steer_norm:.2f} sec={target_sector} {dist_src}"

        nearest_obstacle_m = self._nearest_lidar_range(target_sector)
        if base > 0.0 and self.lidar_safety_enabled and math.isfinite(nearest_obstacle_m) and nearest_obstacle_m <= self.follow_soft_stop_m:
            slow_scale = max(0.0, min(1.0, (nearest_obstacle_m - self.stop_dist_m) / max(0.10, self.follow_soft_stop_m - self.stop_dist_m)))
            base *= slow_scale

        if abs(base) < 8.0 and dist_norm < self.follow_min_drive_ratio and steer_norm < 0.12:
            if math.isfinite(target_lidar_m):
                return "STOP", f"HOLD_TARGET {dist_src} ideal={self.follow_target_ideal_m:.2f}m"
            return "STOP", f"HOLD sh={shoulder_px:.0f}px"

        arc_pwm = self._follow_arc_pwm(base, steer_norm, ex)
        if arc_pwm is not None:
            l, r = arc_pwm
        else:
            l = int(max(-self.follow_max_pwm, min(self.follow_max_pwm, base + steer)))
            r = int(max(-self.follow_max_pwm, min(self.follow_max_pwm, base - steer)))
        if abs(l) < 8 and abs(r) < 8:
            if math.isfinite(target_lidar_m):
                return "STOP", f"HOLD_TARGET {dist_src} ideal={self.follow_target_ideal_m:.2f}m"
            return "STOP", f"HOLD sh={shoulder_px:.0f}px"
        l, r = self._apply_follow_min_pwm(l, r)
        return f"PWM,{l},{r}", f"TRACK owner={self.selected_profile} d={profile_score:.2f} via={self._last_profile_match_mode} ex={ex:+.2f} raw={ex_raw:+.2f} steer_norm={steer_norm:.2f} bear={math.degrees(follow_bearing_rad):+.1f}deg steer={steer:+.0f} sec={target_sector} {dist_src} {range_dbg} dist={dist_norm:.2f} pwm=({l},{r})"

    def tick(self):
        """
        Loop หลักของ Follow Tracker ทำงาน ~15 Hz

        ลำดับการทำงาน:
        1. ถ้าไม่ active → publish IDLE status และ return
        2. เปิดกล้องถ้าจำเป็น
        3. อ่านเฟรมจากกล้อง (USB, MJPEG, หรือ snapshot)
        4. ตรวจจับภาพแช่แข็ง (frozen camera detection)
        5. เรียก _decide_cmd() เพื่อตัดสินใจคำสั่งขับเคลื่อน
        6. จัดการ timeout เมื่อ lost target เกิน follow_lost_timeout_s
        7. ส่งคำสั่งผ่าน _send_cmd() และ publish status/debug
        """
        if not self.active:
            self._idle_status()
            return

        self._open_camera_if_needed()
        frame = None
        if self.camera_mode == "snapshot" or self._snapshot_fallback:
            frame = self._read_snapshot_frame()
            if frame is None or frame.size == 0:
                self._send_cmd("STOP")
                self._publish_status("SNAPSHOT FAIL")
                self._prev_gray_small = None
                self._freeze_accum_s = 0.0
                return
            self._last_frame_ok_ts = time.time()
        else:
            if self.cap is None or not self.cap.isOpened():
                self._send_cmd("STOP")
                self._publish_status("CAMERA ERROR")
                return

            ok = False
            for _ in range(3):
                ok, frame = self.cap.read()
                if ok and frame is not None and frame.size > 0:
                    self._last_frame_ok_ts = time.time()
                    break
                time.sleep(0.02)
            if not ok or frame is None or frame.size == 0:
                self._send_cmd("STOP")
                self._publish_status("CAMERA FRAME FAIL -> RECONNECT")
                # Drop stale stream and reopen on next ticks.
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                self._cam_source = None
                self._prev_gray_small = None
                self._freeze_accum_s = 0.0
                return

        # Detect frozen/repeated frame (common on MJPEG stall):
        # if frame difference remains near-zero for too long, force reconnect + STOP.
        now = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_small = cv2.resize(gray, (160, 90), interpolation=cv2.INTER_AREA)
        if self._prev_gray_small is not None:
            diff = cv2.absdiff(gray_small, self._prev_gray_small)
            mad = cv2.mean(diff)[0]  # mean absolute pixel difference
            dt = max(0.001, now - self._last_frame_check_ts) if self._last_frame_check_ts > 0 else 0.1
            if mad < self.freeze_diff_thresh:
                self._freeze_accum_s += dt
            else:
                self._freeze_accum_s = 0.0
            if self._freeze_accum_s >= self.freeze_trigger_s:
                self._send_cmd("STOP", force=True)
                self._publish_status("FRAME STALE -> RECONNECT")
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                self._cam_source = None
                self._prev_gray_small = None
                self._freeze_accum_s = 0.0
                self._last_frame_check_ts = 0.0
                return
        self._prev_gray_small = gray_small
        self._last_frame_check_ts = now

        cmd, status = self._decide_cmd(frame)

        # Hard LiDAR stop is handled inside _decide_cmd with target-sector
        # filtering. A single noisy point should not override a valid owner lock.

        # If no human pose has been visible for long enough -> target truly lost.
        # Do not hide useful reasons like WRONG_TARGET, LOCKING, or LIDAR_STOP.
        lost_candidates = ("NO_TARGET", "LOW_VIS", "NO_BBOX", "REJECT_NON_HUMAN")
        untrusted_target = self._is_follow_lost_or_untrusted(status)
        if cmd == "STOP" and status in lost_candidates:
            lidar_hold = self._lidar_vision_hold_cmd(status)
            if lidar_hold is not None:
                cmd, status = lidar_hold
                untrusted_target = False

        if (
            cmd == "STOP"
            and status in lost_candidates
            and (time.time() - self.last_person_seen_ts) > self.lost_timeout_s
        ):
            status = "TARGET LOST"
            untrusted_target = True

        if cmd != "STOP":
            self.last_drive_cmd = cmd
            self.last_drive_status = status
            self.last_drive_seen_ts = time.time()
        elif (
            (not untrusted_target)
            and status in lost_candidates
            and self.last_drive_cmd != "STOP"
            and self.follow_blind_hold_s > 0.0
            and (time.time() - self.last_drive_seen_ts) <= self.follow_blind_hold_s
        ):
            cmd = self.last_drive_cmd
            status = f"VISION HOLD {time.time() - self.last_drive_seen_ts:.1f}s ({status}) | {self.last_drive_status}"
            self.follow_state = "VISION_HOLD"
        elif (
            self.follow_search_on_lost
            and status in lost_candidates
            and self._last_target_ts > 0.0
            and (time.time() - self._last_target_ts) <= self.follow_search_hold_s
        ):
            turn = max(0, min(self.follow_max_pwm, self.follow_search_pwm))
            if abs(self._last_target_ex) < 0.05:
                cmd = "STOP"
                status = f"SEARCH_WAIT ({status})"
                self.follow_state = "LOST_SHORT"
            elif self._last_target_ex > 0:
                cmd = f"PWM,{turn},{-turn}"
                status = f"SEARCH_RIGHT {time.time() - self._last_target_ts:.1f}s ({status})"
                self.follow_state = "SEARCHING"
            else:
                cmd = f"PWM,{-turn},{turn}"
                status = f"SEARCH_LEFT {time.time() - self._last_target_ts:.1f}s ({status})"
                self.follow_state = "SEARCHING"
        elif untrusted_target:
            self.follow_state = "LOST"
            self._clear_follow_drive_memory()

        self._send_cmd(cmd, force=(cmd == "STOP" and untrusted_target))
        self._publish_status(status)
        self._publish_debug(cmd, status, getattr(self, "_last_people_count", 0))

    def destroy_node(self):
        """
        ปิด node อย่างปลอดภัย: ส่ง STOP ก่อน แล้ว release กล้องและ MediaPipe Pose

        เรียกโดย ROS2 lifecycle เมื่อ node ถูก shutdown
        """
        try:
            self._send_cmd("STOP", force=True)
        except Exception:
            pass
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.pose is not None:
            try:
                self.pose.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    """
    Entry point ของ follow_tracker_node

    เริ่ม rclpy, สร้าง FollowTrackerNode, และ spin จนกว่าจะถูก shutdown
    เมื่อออก จะ destroy_node() และ rclpy.shutdown() อัตโนมัติ
    """
    rclpy.init(args=args)
    node = FollowTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
