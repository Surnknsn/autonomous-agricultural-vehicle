from flask import Flask, render_template, request, jsonify, send_from_directory, send_file, abort, Response, stream_with_context, make_response
from flask_sock import Sock
import websocket
import threading
import os
import time
import json
import requests
import shutil
import csv
import math
import zipfile
from datetime import datetime
from werkzeug.utils import secure_filename
try:
    import cv2
except Exception:
    cv2 = None
try:
    import numpy as np
except Exception:
    np = None
try:
    import mediapipe.python.solutions.pose as mp_pose
    import mediapipe.python.solutions.drawing_utils as mp_drawing
    _MEDIAPIPE_OK = True
except Exception:
    mp_pose = None
    mp_drawing = None
    _MEDIAPIPE_OK = False

app = Flask(__name__)
sock = Sock(app)

ROSBRIDGE_URL = "ws://127.0.0.1:9090"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.abspath(os.path.join(BASE_DIR, os.pardir))
AMR_DESCRIPTION_DIR = os.path.join(WS_DIR, "src", "amr_description")
DATASET_DIR = os.path.join(BASE_DIR, "dataset")
SHARED_MAP_STATE_PATH = os.path.join(BASE_DIR, "shared_map_state.json")
SHARED_UI_STATE_PATH = os.path.join(BASE_DIR, "shared_ui_state.json")
PATH_LIBRARY_DIR = os.path.join(BASE_DIR, "saved_paths")
DATALOG_DIR = os.getenv("DATALOG_DIR", os.path.join(BASE_DIR, "datalog"))
CAMERA_MJPEG_URL = os.getenv("CAMERA_MJPEG_URL", "http://127.0.0.1:8081/video_feed")
CAMERA_MODE = os.getenv("CAMERA_MODE", "auto").lower()  # auto | mjpeg | usb
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "360"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "20"))
CAMERA_QUALITY = int(os.getenv("CAMERA_QUALITY", "75"))
CAMERA_STABILIZE_DEFAULT = os.getenv("CAMERA_STABILIZE_DEFAULT", "0").lower() in ("1", "true", "yes", "on")
CAMERA_FAST_FPS = int(os.getenv("CAMERA_FAST_FPS", "5"))
CAMERA_FAST_QUALITY = int(os.getenv("CAMERA_FAST_QUALITY", "48"))
CAMERA_FAST_WIDTH = int(os.getenv("CAMERA_FAST_WIDTH", "480"))
CAMERA_FOLLOW_OVERLAY_WIDTH = int(os.getenv("CAMERA_FOLLOW_OVERLAY_WIDTH", "360"))
CAMERA_FOLLOW_OWNER_MATCH_MAX = float(os.getenv("CAMERA_FOLLOW_OWNER_MATCH_MAX", "0.50"))
CAMERA_FOLLOW_OWNER_STRONG_MAX = float(os.getenv("CAMERA_FOLLOW_OWNER_STRONG_MAX", "0.44"))
CAMERA_FOLLOW_TARGET_SHOULDER_PX = float(os.getenv("CAMERA_FOLLOW_TARGET_SHOULDER_PX", "210"))
CAMERA_FOLLOW_DISTANCE_A = float(os.getenv("CAMERA_FOLLOW_DISTANCE_A", "198.98"))
CAMERA_FOLLOW_DISTANCE_B = float(os.getenv("CAMERA_FOLLOW_DISTANCE_B", "0.319"))
CAMERA_FOLLOW_CAMERA_TO_LIDAR_M = float(os.getenv("CAMERA_FOLLOW_CAMERA_TO_LIDAR_M", "0.085"))
CAMERA_FOLLOW_RANGE_SCALE = float(os.getenv("CAMERA_FOLLOW_RANGE_SCALE", "0.78125"))
CAMERA_FOLLOW_RANGE_OFFSET_M = float(os.getenv("CAMERA_FOLLOW_RANGE_OFFSET_M", "0.0"))
CAMERA_FOLLOW_AUTO_CONTRAST = os.getenv("CAMERA_FOLLOW_AUTO_CONTRAST", "0").lower() in ("1", "true", "yes", "on")
CAMERA_FOLLOW_AUTO_CONTRAST_CLIP = float(os.getenv("CAMERA_FOLLOW_AUTO_CONTRAST_CLIP", "2.2"))
CAMERA_FOLLOW_AUTO_CONTRAST_BLEND = float(os.getenv("CAMERA_FOLLOW_AUTO_CONTRAST_BLEND", "0.72"))


def _safe_owner_name(name):
    raw = (name or "").strip()
    # Keep Thai/Unicode owner names usable, but block path separators/control chars.
    safe = "".join(ch for ch in raw if ch not in "/\\" and ch.isprintable()).strip().strip(".")
    if not safe:
        safe = secure_filename(raw)
    return safe[:64]


def _auto_contrast_frame(frame):
    if not CAMERA_FOLLOW_AUTO_CONTRAST or frame is None or cv2 is None or np is None:
        return frame
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean = float(np.mean(gray))
        p05, p50, p95 = np.percentile(gray, [5, 50, 95])
        contrast = float(p95 - p05)
        if 85.0 <= mean <= 170.0 and contrast >= 78.0 and p95 < 238.0:
            return frame

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l_chan, a_chan, b_chan = cv2.split(lab)
        clahe = cv2.createCLAHE(
            clipLimit=max(1.0, min(5.0, CAMERA_FOLLOW_AUTO_CONTRAST_CLIP)),
            tileGridSize=(8, 8),
        )
        l_eq = clahe.apply(l_chan)
        if mean < 95.0 or p50 < 85.0:
            gamma = min(1.85, max(1.10, 1.35 + (100.0 - min(mean, 100.0)) / 120.0))
        elif p95 > 235.0:
            gamma = 0.82
        else:
            gamma = 1.0
        inv_gamma = 1.0 / max(0.45, min(2.2, gamma))
        table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype=np.uint8)
        l_eq = cv2.LUT(l_eq, table)
        enhanced = cv2.cvtColor(cv2.merge((l_eq, a_chan, b_chan)), cv2.COLOR_LAB2BGR)
        blend = max(0.0, min(1.0, CAMERA_FOLLOW_AUTO_CONTRAST_BLEND))
        return cv2.addWeighted(enhanced, blend, frame, 1.0 - blend, 0.0)
    except Exception:
        return frame


def _dataset_owner_dir(owner):
    safe = _safe_owner_name(owner)
    if not safe:
        return "", ""
    folder = os.path.abspath(os.path.join(DATASET_DIR, safe))
    dataset_root = os.path.abspath(DATASET_DIR)
    if not (folder == dataset_root or folder.startswith(dataset_root + os.sep)):
        return "", ""
    return safe, folder
CAMERA_FOLLOW_OVERLAY_MAX_HZ = float(os.getenv("CAMERA_FOLLOW_OVERLAY_MAX_HZ", "2.0"))
CAMERA_FOLLOW_DRAW_LANDMARKS = os.getenv("CAMERA_FOLLOW_DRAW_LANDMARKS", "0").lower() in ("1", "true", "yes", "on")

# Ensure only one USB stream owns camera device at a time.
_stream_guard_lock = threading.Lock()
_active_stream_token = 0
_usb_stream_active = 0
_follow_pose = None
_usb_worker_lock = threading.Lock()
_usb_worker_thread = None
_usb_worker_stop = False
_usb_latest_frame = None
_usb_latest_ts = 0.0
_snapshot_stab_lock = threading.Lock()
_snapshot_stab_state = {"prev_gray": None, "lp_dx": 0.0, "lp_dy": 0.0, "lp_da": 0.0}
_follow_overlay_lock = threading.Lock()
_follow_overlay_state = {"ts": 0.0, "data": None}
_follow_overlay_status = {"ts": 0.0, "active": False}
_follow_pose_lock = threading.Lock()
_follow_overlay_profile_lock = threading.Lock()
_follow_overlay_profile_cache = {"owner": "", "key": None, "refs": []}

def _create_follow_pose():
    if not _MEDIAPIPE_OK:
        return None
    try:
        return mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=0.65,
            min_tracking_confidence=0.60,
        )
    except Exception:
        return None


if _MEDIAPIPE_OK:
    _follow_pose = _create_follow_pose()

os.makedirs(DATASET_DIR, exist_ok=True)
os.makedirs(PATH_LIBRARY_DIR, exist_ok=True)


def _default_map_state():
    return {
        "map_mode": "GRID",
        "boundary_points": [],
        "grid_waypoints": [],
        "path_locked": False,
        "updated_at": 0.0,
    }


def _load_map_state():
    if not os.path.isfile(SHARED_MAP_STATE_PATH):
        return _default_map_state()
    try:
        with open(SHARED_MAP_STATE_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        state = _default_map_state()
        if isinstance(data, dict):
            state["map_mode"] = str(data.get("map_mode") or "GRID").upper()
            state["boundary_points"] = data.get("boundary_points") or []
            state["grid_waypoints"] = data.get("grid_waypoints") or []
            state["path_locked"] = bool(data.get("path_locked", False))
            state["updated_at"] = float(data.get("updated_at") or 0.0)
        return state
    except Exception:
        return _default_map_state()


def _save_map_state(state):
    payload = _default_map_state()
    payload["map_mode"] = str(state.get("map_mode") or "GRID").upper()
    payload["boundary_points"] = state.get("boundary_points") or []
    payload["grid_waypoints"] = state.get("grid_waypoints") or []
    payload["path_locked"] = bool(state.get("path_locked", False))
    payload["updated_at"] = time.time()
    tmp_path = SHARED_MAP_STATE_PATH + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False)
    os.replace(tmp_path, SHARED_MAP_STATE_PATH)
    return payload


def _safe_path_name(name):
    raw = (name or "").strip()
    safe = "".join(ch for ch in raw if ch not in "/\\" and ch.isprintable()).strip().strip(".")
    safe = safe or secure_filename(raw)
    if not safe:
        safe = datetime.now().strftime("path_%Y%m%d_%H%M%S")
    return safe[:64]


def _saved_path_file(name):
    safe = _safe_path_name(name)
    path = os.path.abspath(os.path.join(PATH_LIBRARY_DIR, safe + ".json"))
    root = os.path.abspath(PATH_LIBRARY_DIR)
    if not path.startswith(root + os.sep):
        raise ValueError("invalid path name")
    return safe, path


def _saved_path_summary(payload, name=None):
    waypoints = _clean_waypoints((payload or {}).get("grid_waypoints") or [])
    boundary = _clean_waypoints((payload or {}).get("boundary_points") or [])
    return {
        "name": name or str((payload or {}).get("name") or ""),
        "map_mode": str((payload or {}).get("map_mode") or "GRID").upper(),
        "waypoint_count": len(waypoints),
        "boundary_count": len(boundary),
        "updated_at": float((payload or {}).get("updated_at") or 0.0),
    }


def _list_saved_paths():
    items = []
    for filename in sorted(os.listdir(PATH_LIBRARY_DIR)):
        if not filename.endswith(".json"):
            continue
        full_path = os.path.join(PATH_LIBRARY_DIR, filename)
        if not os.path.isfile(full_path):
            continue
        name = filename[:-5]
        try:
            with open(full_path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            items.append(_saved_path_summary(payload, name=name))
        except Exception:
            continue
    items.sort(key=lambda x: x.get("updated_at", 0.0), reverse=True)
    return items


def _load_saved_path(name):
    safe, path = _saved_path_file(name)
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    state = _default_map_state()
    if isinstance(payload, dict):
        state["map_mode"] = str(payload.get("map_mode") or "GRID").upper()
        state["boundary_points"] = _clean_waypoints(payload.get("boundary_points") or [])
        state["grid_waypoints"] = _clean_waypoints(payload.get("grid_waypoints") or [])
        state["path_locked"] = bool(payload.get("path_locked", False))
        state["updated_at"] = float(payload.get("updated_at") or 0.0)
    return {"name": safe, "state": state}


def _save_path_library_item(data):
    name = _safe_path_name((data or {}).get("name") or "")
    safe, path = _saved_path_file(name)
    payload = {
        "name": safe,
        "map_mode": str((data or {}).get("map_mode") or "GRID").upper(),
        "boundary_points": _clean_waypoints((data or {}).get("boundary_points") or []),
        "grid_waypoints": _clean_waypoints((data or {}).get("grid_waypoints") or []),
        "path_locked": False,
        "updated_at": time.time(),
    }
    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    os.replace(tmp_path, path)
    return payload


def _default_ui_state():
    return {
        "follow_target": "NOT SELECTED",
        "safety_enabled": True,
        "gps_start_only": False,
        "auto_speed_pct": 100,
        "safety_cm": 150,
        "lidar_range_m": 8.0,
        "camera_preview": False,
        "camera_stabilize": False,
        "follow_gui": False,
        "waypoint_coord_labels": True,
        "guided_capture_active": False,
        "guided_capture_owner": "",
        "guided_capture_index": 0,
        "updated_at": 0.0,
    }


def _load_ui_state():
    if not os.path.isfile(SHARED_UI_STATE_PATH):
        return _default_ui_state()
    try:
        with open(SHARED_UI_STATE_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        state = _default_ui_state()
        if isinstance(data, dict):
            state["follow_target"] = str(data.get("follow_target") or "NOT SELECTED")
            state["safety_enabled"] = bool(data.get("safety_enabled", True))
            state["gps_start_only"] = bool(data.get("gps_start_only", False))
            state["auto_speed_pct"] = max(0, min(100, int(round(float(data.get("auto_speed_pct", 100))))))
            state["safety_cm"] = max(0, min(400, int(round(float(data.get("safety_cm", 150))))))
            state["lidar_range_m"] = max(1.0, min(25.0, float(data.get("lidar_range_m", 8.0))))
            state["camera_preview"] = bool(data.get("camera_preview", False))
            state["camera_stabilize"] = bool(data.get("camera_stabilize", False))
            state["follow_gui"] = bool(data.get("follow_gui", False))
            state["waypoint_coord_labels"] = bool(data.get("waypoint_coord_labels", True))
            state["guided_capture_active"] = bool(data.get("guided_capture_active", False))
            state["guided_capture_owner"] = str(data.get("guided_capture_owner") or "")
            state["guided_capture_index"] = max(0, int(data.get("guided_capture_index", 0)))
            state["updated_at"] = float(data.get("updated_at") or 0.0)
        return state
    except Exception:
        return _default_ui_state()


def _save_ui_state(state):
    payload = _default_ui_state()
    payload["follow_target"] = str(state.get("follow_target") or "NOT SELECTED")
    payload["safety_enabled"] = bool(state.get("safety_enabled", True))
    payload["gps_start_only"] = bool(state.get("gps_start_only", False))
    payload["auto_speed_pct"] = max(0, min(100, int(round(float(state.get("auto_speed_pct", 100))))))
    payload["safety_cm"] = max(0, min(400, int(round(float(state.get("safety_cm", 150))))))
    payload["lidar_range_m"] = max(1.0, min(25.0, float(state.get("lidar_range_m", 8.0))))
    payload["camera_preview"] = bool(state.get("camera_preview", False))
    payload["camera_stabilize"] = bool(state.get("camera_stabilize", False))
    payload["follow_gui"] = bool(state.get("follow_gui", False))
    payload["waypoint_coord_labels"] = bool(state.get("waypoint_coord_labels", True))
    payload["guided_capture_active"] = bool(state.get("guided_capture_active", False))
    payload["guided_capture_owner"] = str(state.get("guided_capture_owner") or "")
    payload["guided_capture_index"] = max(0, int(state.get("guided_capture_index", 0)))
    payload["updated_at"] = time.time()
    tmp_path = SHARED_UI_STATE_PATH + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False)
    os.replace(tmp_path, SHARED_UI_STATE_PATH)
    return payload


def _safe_mission_id(mission_id):
    raw = str(mission_id or "").strip()
    safe = "".join(ch for ch in raw if ch.isalnum() or ch in ("-", "_"))
    return safe[:80]


def _haversine_m(lat1, lon1, lat2, lon2):
    try:
        if not all(math.isfinite(float(v)) for v in (lat1, lon1, lat2, lon2)):
            return float("nan")
        r = 6371000.0
        p1 = math.radians(float(lat1))
        p2 = math.radians(float(lat2))
        dp = math.radians(float(lat2) - float(lat1))
        dl = math.radians(float(lon2) - float(lon1))
        a = math.sin(dp * 0.5) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl * 0.5) ** 2
        return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    except Exception:
        return float("nan")


def _gps_delta_m(origin_lat, origin_lng, lat, lng):
    try:
        if not all(math.isfinite(float(v)) for v in (origin_lat, origin_lng, lat, lng)):
            return float("nan"), float("nan")
        r = 6371000.0
        d_lat = math.radians(float(lat) - float(origin_lat))
        d_lng = math.radians(float(lng) - float(origin_lng))
        mean_lat = math.radians((float(lat) + float(origin_lat)) * 0.5)
        north = d_lat * r
        east = d_lng * r * math.cos(mean_lat)
        return north, east
    except Exception:
        return float("nan"), float("nan")


def _angle_wrap_deg(angle):
    try:
        return (float(angle) + 180.0) % 360.0 - 180.0
    except Exception:
        return float("nan")


def _point_lat_lng(point):
    if not isinstance(point, dict):
        return None
    try:
        lat = float(point.get("lat"))
        lng = float(point.get("lng"))
        if math.isfinite(lat) and math.isfinite(lng):
            return {"lat": lat, "lng": lng}
    except Exception:
        pass
    return None


def _clean_waypoints(points):
    out = []
    for p in points or []:
        clean = _point_lat_lng(p)
        if clean:
            out.append(clean)
    return out


def _closest_path_metrics(lat, lng, yaw_deg, path):
    if not path:
        return {
            "ref_segment_idx": -1,
            "ref_progress_m": float("nan"),
            "ref_cross_track_m": float("nan"),
            "ref_along_segment_m": float("nan"),
            "ref_segment_len_m": float("nan"),
            "ref_heading_deg": float("nan"),
            "ref_heading_error_deg": float("nan"),
            "ref_dist_to_start_m": float("nan"),
            "ref_dist_to_end_m": float("nan"),
        }
    if len(path) == 1:
        d = _haversine_m(lat, lng, path[0]["lat"], path[0]["lng"])
        return {
            "ref_segment_idx": 0,
            "ref_progress_m": 0.0,
            "ref_cross_track_m": d,
            "ref_along_segment_m": 0.0,
            "ref_segment_len_m": 0.0,
            "ref_heading_deg": float("nan"),
            "ref_heading_error_deg": float("nan"),
            "ref_dist_to_start_m": d,
            "ref_dist_to_end_m": d,
        }

    best = None
    progress_before = 0.0
    total_before = 0.0
    for i in range(len(path) - 1):
        a = path[i]
        b = path[i + 1]
        seg_n, seg_e = _gps_delta_m(a["lat"], a["lng"], b["lat"], b["lng"])
        rob_n, rob_e = _gps_delta_m(a["lat"], a["lng"], lat, lng)
        seg_len = math.hypot(seg_n, seg_e)
        if not math.isfinite(seg_len) or seg_len < 0.05:
            continue
        t = max(0.0, min(1.0, (rob_n * seg_n + rob_e * seg_e) / (seg_len * seg_len)))
        proj_n = seg_n * t
        proj_e = seg_e * t
        dn = rob_n - proj_n
        de = rob_e - proj_e
        dist = math.hypot(dn, de)
        signed_cross = (seg_n * rob_e - seg_e * rob_n) / seg_len
        heading_deg = (math.degrees(math.atan2(seg_e, seg_n)) + 360.0) % 360.0
        item = {
            "ref_segment_idx": i,
            "ref_progress_m": total_before + t * seg_len,
            "ref_cross_track_m": signed_cross,
            "ref_along_segment_m": t * seg_len,
            "ref_segment_len_m": seg_len,
            "ref_heading_deg": heading_deg,
            "ref_heading_error_deg": _angle_wrap_deg(yaw_deg - heading_deg),
            "ref_dist_to_start_m": math.hypot(rob_n, rob_e),
            "ref_dist_to_end_m": _haversine_m(lat, lng, b["lat"], b["lng"]),
            "_abs_dist": dist,
        }
        if best is None or item["_abs_dist"] < best["_abs_dist"]:
            best = item
        total_before += seg_len

    if not best:
        return _closest_path_metrics(lat, lng, yaw_deg, path[:1])
    best.pop("_abs_dist", None)
    return best


def _json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_json_safe(v) for v in value]
    return value


QUALITY_EXTRA_FIELDS = [
    "gps_step_m", "gps_speed_mps", "gps_jump_flag",
    "speed_valid", "battery_valid", "row_quality", "quality_flags",
]


def _to_float(value, default=float("nan")):
    if value is None or value == "":
        return default
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except Exception:
        return default


def _to_int(value, default=0):
    try:
        num = _to_float(value)
        return int(round(num)) if math.isfinite(num) else default
    except Exception:
        return default


def _quality_for_sample(row, prev=None):
    flags = []
    prev = prev or {}

    lat = _to_float(row.get("lat"))
    lng = _to_float(row.get("lng"))
    ts = _to_float(row.get("ts"))
    if not math.isfinite(ts):
        ts = _to_float(row.get("elapsed_s"))

    prev_lat = _to_float(prev.get("lat"))
    prev_lng = _to_float(prev.get("lng"))
    prev_ts = _to_float(prev.get("ts"))
    if not math.isfinite(prev_ts):
        prev_ts = _to_float(prev.get("elapsed_s"))

    gps_step = float("nan")
    gps_speed = float("nan")
    gps_jump = 0
    if all(math.isfinite(x) for x in (lat, lng, prev_lat, prev_lng)):
        gps_step = _haversine_m(prev_lat, prev_lng, lat, lng)
        if math.isfinite(ts) and math.isfinite(prev_ts) and ts > prev_ts:
            gps_speed = gps_step / max(0.001, ts - prev_ts)
        if gps_step > 0.80 or (math.isfinite(gps_speed) and gps_speed > 2.50):
            gps_jump = 1
            flags.append("gps_jump")

    speed_values = [
        _to_float(row.get("wheel_speed_l_kph")),
        _to_float(row.get("wheel_speed_r_kph")),
        _to_float(row.get("wheel_speed_avg_kph")),
    ]
    speed_valid = all(math.isfinite(v) and abs(v) <= 8.0 for v in speed_values)
    if not speed_valid:
        flags.append("speed_spike")

    soc = _to_float(row.get("battery_soc_pct"))
    volt = _to_float(row.get("battery_voltage_v"))
    current = _to_float(row.get("battery_current_a"))
    temp = _to_float(row.get("battery_temp_c"))
    battery_valid = (
        (not math.isfinite(soc) or 0.0 <= soc <= 100.0)
        and math.isfinite(volt) and 20.0 <= volt <= 32.0
        and math.isfinite(current) and abs(current) <= 250.0
        and (not math.isfinite(temp) or -20.0 <= temp <= 85.0)
    )
    if not battery_valid:
        flags.append("battery_invalid")

    if not flags:
        quality = "GOOD"
    elif all(f == "gps_jump" for f in flags):
        quality = "USABLE"
    else:
        quality = "BAD"

    return {
        "gps_step_m": round(gps_step, 3) if math.isfinite(gps_step) else "",
        "gps_speed_mps": round(gps_speed, 3) if math.isfinite(gps_speed) else "",
        "gps_jump_flag": gps_jump,
        "speed_valid": 1 if speed_valid else 0,
        "battery_valid": 1 if battery_valid else 0,
        "row_quality": quality,
        "quality_flags": "|".join(flags),
    }


def _coerce_row_for_json(row):
    clean = {}
    for k, v in row.items():
        if v is None or v == "":
            clean[k] = None
            continue
        try:
            num = float(v)
            clean[k] = int(num) if num.is_integer() else num
        except Exception:
            clean[k] = v
    return clean


def _make_clean_csv(src_path, out_path, keep_usable=True):
    if not os.path.isfile(src_path):
        return False
    rows_written = 0
    prev = None
    with open(src_path, "r", encoding="utf-8", newline="") as f_in:
        reader = csv.DictReader(f_in)
        fieldnames = list(reader.fieldnames or [])
        for extra in QUALITY_EXTRA_FIELDS:
            if extra not in fieldnames:
                fieldnames.append(extra)
        tmp = out_path + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="") as f_out:
            writer = csv.DictWriter(f_out, fieldnames=fieldnames)
            writer.writeheader()
            for row in reader:
                enriched = dict(row)
                if not all(k in enriched and str(enriched.get(k, "")) != "" for k in QUALITY_EXTRA_FIELDS):
                    enriched.update(_quality_for_sample(enriched, prev))
                keep = enriched.get("row_quality") == "GOOD" or (keep_usable and enriched.get("row_quality") == "USABLE")
                if keep:
                    writer.writerow({k: enriched.get(k, "") for k in fieldnames})
                    rows_written += 1
                prev = dict(row)
        os.replace(tmp, out_path)
    return rows_written > 0


def _make_clean_jsonl(src_path, out_path, keep_usable=True):
    if not os.path.isfile(src_path):
        return False
    rows_written = 0
    prev = None
    tmp = out_path + ".tmp"
    with open(src_path, "r", encoding="utf-8", newline="") as f_in, open(tmp, "w", encoding="utf-8") as f_out:
        reader = csv.DictReader(f_in)
        for row in reader:
            enriched = dict(row)
            if not all(k in enriched and str(enriched.get(k, "")) != "" for k in QUALITY_EXTRA_FIELDS):
                enriched.update(_quality_for_sample(enriched, prev))
            keep = enriched.get("row_quality") == "GOOD" or (keep_usable and enriched.get("row_quality") == "USABLE")
            if keep:
                f_out.write(json.dumps(_json_safe(_coerce_row_for_json(enriched)), ensure_ascii=False) + "\n")
                rows_written += 1
            prev = dict(row)
    os.replace(tmp, out_path)
    return rows_written > 0


def _quality_summary_from_csv(src_path):
    summary = {
        "quality_good": 0,
        "quality_usable": 0,
        "quality_bad": 0,
        "gps_jump_rows": 0,
        "speed_invalid_rows": 0,
        "battery_invalid_rows": 0,
    }
    if not os.path.isfile(src_path):
        return summary
    prev = None
    try:
        with open(src_path, "r", encoding="utf-8", newline="") as f:
            for row in csv.DictReader(f):
                enriched = dict(row)
                if not all(k in enriched and str(enriched.get(k, "")) != "" for k in QUALITY_EXTRA_FIELDS):
                    enriched.update(_quality_for_sample(enriched, prev))
                quality = str(enriched.get("row_quality") or "BAD").upper()
                if quality == "GOOD":
                    summary["quality_good"] += 1
                elif quality == "USABLE":
                    summary["quality_usable"] += 1
                else:
                    summary["quality_bad"] += 1
                if _to_int(enriched.get("gps_jump_flag"), 0):
                    summary["gps_jump_rows"] += 1
                if not _to_int(enriched.get("speed_valid"), 0):
                    summary["speed_invalid_rows"] += 1
                if not _to_int(enriched.get("battery_valid"), 0):
                    summary["battery_invalid_rows"] += 1
                prev = dict(row)
    except Exception:
        pass
    return summary


class MissionDatalogger:
    """Mission logger fed by rosbridge, independent of whether a phone is open."""

    CSV_FIELDS = [
        "iso_time", "elapsed_s", "mission_id", "run_mode", "event",
        "hw_mode", "lat", "lng", "yaw_deg", "gps_fix", "gps_ready",
        "lidar_min_front_m", "lidar_obstacle", "pwm_l", "pwm_r",
        "enc_l", "enc_r", "wheel_speed_l_kph", "wheel_speed_r_kph", "wheel_speed_avg_kph",
        "wp_idx", "path_len", "target_lat", "target_lng", "dist_to_target_m",
        "battery_soc_pct", "battery_voltage_v", "battery_current_a", "battery_temp_c", "battery_status",
        "num_sats", "segment",
        "follow_status", "follow_debug",
    ]

    def __init__(self, base_dir):
        self.base_dir = os.path.abspath(base_dir)
        os.makedirs(self.base_dir, exist_ok=True)
        self.lock = threading.RLock()
        self.active = False
        self.pending_start = None
        self.mission_id = ""
        self.run_mode = "UNKNOWN"
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = ""
        self.summary_path = ""
        self.last_sample_ts = 0.0
        self.last_active_ts = 0.0
        self.sample_interval_s = float(os.getenv("DATALOG_SAMPLE_INTERVAL_S", "1.0"))
        self.manual_sample_interval_s = float(os.getenv("DATALOG_MANUAL_SAMPLE_INTERVAL_S", "2.0"))
        self.idle_finish_s = float(os.getenv("DATALOG_IDLE_FINISH_S", "10.0"))
        self.manual_enable = os.getenv("DATALOG_MANUAL_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.manual_session_max_s = float(os.getenv("DATALOG_MANUAL_SESSION_MAX_S", "3600.0"))
        self.mode_change_hold_s = float(os.getenv("DATALOG_MODE_CHANGE_HOLD_S", "2.0"))
        self.manual_start_hold_s = float(os.getenv("DATALOG_MANUAL_START_HOLD_S", "2.0"))
        self.latest_follow_status = ""
        self.latest_follow_debug = ""
        self.last_gps_ts = 0.0
        self.last_hw_mode = None
        self._prev_hw_mode = None
        self.received_gps_count = 0
        self.last_rosbridge_ok_ts = 0.0
        self.last_rosbridge_error = ""
        self.stats = {}
        self._mode_mismatch_since = 0.0
        self._manual_candidate_since = 0.0

    def start_thread(self):
        threading.Thread(target=self._rosbridge_loop, daemon=True).start()
        threading.Thread(target=self._rosbridge_gps_loop, daemon=True).start()
        threading.Thread(target=self._ros2_topic_loop, daemon=True).start()

    def list_missions(self, mode=None):
        mode = str(mode or "").strip().upper()
        out = []
        try:
            for name in os.listdir(self.base_dir):
                if not name.endswith(".summary.json"):
                    continue
                path = os.path.join(self.base_dir, name)
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        data = json.load(f)
                    data = _json_safe(data)
                    if mode and mode != "ALL" and str(data.get("run_mode", "")).upper() != mode:
                        continue
                    out.append(data)
                except Exception:
                    continue
        except Exception:
            pass
        out.sort(key=lambda x: str(x.get("start_time") or ""), reverse=True)
        return out

    def status_snapshot(self):
        with self.lock:
            now = time.time()
            return {
                "active": self.active,
                "pending": bool(self.pending_start),
                "active_mode": self.run_mode if self.active else "",
                "active_mission_id": self.mission_id if self.active else "",
                "last_gps_age_s": round(now - self.last_gps_ts, 2) if self.last_gps_ts > 0 else None,
                "last_hw_mode": self.last_hw_mode,
                "received_gps_count": self.received_gps_count,
                "rosbridge_age_s": round(now - self.last_rosbridge_ok_ts, 2) if self.last_rosbridge_ok_ts > 0 else None,
                "rosbridge_error": self.last_rosbridge_error,
                "manual_enable": self.manual_enable,
                "manual_sample_interval_s": self.manual_sample_interval_s,
                "mode_change_hold_s": self.mode_change_hold_s,
            }

    def download_path(self, mission_id, fmt):
        mission_id = _safe_mission_id(mission_id)
        if not mission_id:
            return None
        if fmt == "csv":
            path = os.path.join(self.base_dir, f"{mission_id}.csv")
        elif fmt == "json":
            path = os.path.join(self.base_dir, f"{mission_id}.summary.json")
        elif fmt == "clean_csv":
            path = os.path.join(self.base_dir, f"{mission_id}.clean.csv")
            if not _make_clean_csv(os.path.join(self.base_dir, f"{mission_id}.csv"), path):
                return None
        elif fmt == "data_json":
            path = os.path.join(self.base_dir, f"{mission_id}.data.json")
            self._make_data_json(mission_id, path)
        elif fmt == "zip":
            path = os.path.join(self.base_dir, f"{mission_id}.zip")
            self._make_zip(mission_id, path)
        else:
            return None
        if not os.path.isfile(path):
            return None
        return path

    def _make_zip(self, mission_id, zip_path):
        csv_path = os.path.join(self.base_dir, f"{mission_id}.csv")
        summary_path = os.path.join(self.base_dir, f"{mission_id}.summary.json")
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as z:
            if os.path.isfile(csv_path):
                z.write(csv_path, arcname=os.path.basename(csv_path))
            if os.path.isfile(summary_path):
                z.write(summary_path, arcname=os.path.basename(summary_path))
            data_json_path = os.path.join(self.base_dir, f"{mission_id}.data.json")
            if os.path.isfile(data_json_path):
                z.write(data_json_path, arcname=os.path.basename(data_json_path))

    def _make_data_json(self, mission_id, out_path):
        csv_path = os.path.join(self.base_dir, f"{mission_id}.csv")
        summary_path = os.path.join(self.base_dir, f"{mission_id}.summary.json")
        rows = []
        if os.path.isfile(csv_path):
            with open(csv_path, "r", encoding="utf-8", newline="") as f:
                for row in csv.DictReader(f):
                    clean = {}
                    for k, v in row.items():
                        if v is None or v == "":
                            clean[k] = None
                            continue
                        try:
                            num = float(v)
                            clean[k] = int(num) if num.is_integer() else num
                        except Exception:
                            clean[k] = v
                    rows.append(clean)
        summary = {}
        if os.path.isfile(summary_path):
            with open(summary_path, "r", encoding="utf-8") as f:
                summary = json.load(f)
        tmp = out_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(_json_safe({"summary": summary, "rows": rows}), f, ensure_ascii=False, indent=2)
        os.replace(tmp, out_path)

    def handle_control(self, cmd):
        cmd = str(cmd or "").strip().upper()
        now = time.time()
        with self.lock:
            if cmd in ("AUTO_START", "START"):
                self.pending_start = {"ts": now, "mode": "AUTO"}
                self._append_event("START_REQUEST")
            elif cmd == "FOLLOW_START":
                self.pending_start = {"ts": now, "mode": "FOLLOW"}
                self._append_event("FOLLOW_START_REQUEST")
            elif cmd == "PAUSE" or cmd.endswith("_PAUSE"):
                self._append_event("PAUSE")
            elif cmd == "STOP":
                self._finish("STOP")
                self.pending_start = None
            elif cmd in ("RETURN_HOME", "RTH", "GO_HOME"):
                self._append_event("RETURN_HOME")

    def handle_emergency(self, cmd):
        cmd = str(cmd or "").strip().upper()
        with self.lock:
            if cmd == "EMERGENCY":
                self._finish("EMERGENCY")
                self.pending_start = None
            elif cmd == "RESET":
                self._append_event("EMERGENCY_RESET")

    def handle_follow_status(self, text):
        with self.lock:
            self.latest_follow_status = str(text or "")[:240]

    def handle_follow_debug(self, text):
        with self.lock:
            self.latest_follow_debug = str(text or "")[:360]

    def handle_gps(self, data):
        now = time.time()
        sample = self._parse_current_gps(data, now)
        if sample is None:
            return
        with self.lock:
            self.last_gps_ts = now
            prev_hw = self._prev_hw_mode
            self._prev_hw_mode = sample["hw_mode"]
            self.last_hw_mode = sample["hw_mode"]
            self.received_gps_count += 1
            self._maybe_start(sample, now, prev_hw)
            if not self.active:
                return

            if self.run_mode == "MANUAL":
                if sample["hw_mode"] != 0:
                    if self._mode_mismatch_persisting(now):
                        self._write_sample(sample, "MODE_CHANGE")
                        self._finish("MODE_CHANGE")
                        return
                else:
                    self._mode_mismatch_since = 0.0
                try:
                    start_ts = datetime.fromisoformat(self.stats["start_time"]).timestamp()
                except Exception:
                    start_ts = now
                if self.manual_session_max_s > 0.0 and (now - start_ts) >= self.manual_session_max_s:
                    self._write_sample(sample, "MANUAL_ROTATE")
                    self._finish("MANUAL_ROTATE")
                    self._start("MANUAL", sample, now)
                    return
                if (now - self.last_sample_ts) >= self.manual_sample_interval_s:
                    self._write_sample(sample, "")
                    self.last_sample_ts = now
                return

            is_active = (
                sample["hw_mode"] in (1, 2)
                and (
                    abs(sample["pwm_l"]) > 5
                    or abs(sample["pwm_r"]) > 5
                    or sample["path_len"] > 0
                    or self.run_mode == "FOLLOW"
                )
            )
            if is_active:
                self.last_active_ts = now

            event = ""
            if sample["hw_mode"] == 3:
                self._write_sample(sample, "EMERGENCY")
                self._finish("EMERGENCY")
                return
            if self.run_mode in ("AUTO", "FOLLOW") and sample["hw_mode"] == 0:
                if self._mode_mismatch_persisting(now):
                    self._write_sample(sample, "MODE_MANUAL")
                    self._finish("MODE_MANUAL")
                    return
            else:
                self._mode_mismatch_since = 0.0
            if sample["path_len"] > 0 and sample["wp_idx"] >= sample["path_len"]:
                self._write_sample(sample, "MISSION_COMPLETE")
                self._finish("COMPLETE")
                return
            if not is_active and self.last_active_ts > 0.0 and (now - self.last_active_ts) >= self.idle_finish_s:
                self._finish("IDLE_TIMEOUT")
                return

            if (now - self.last_sample_ts) >= self.sample_interval_s:
                self._write_sample(sample, event)
                self.last_sample_ts = now

    def _maybe_start(self, sample, now, prev_hw=None):
        if self.active:
            return
        pending = self.pending_start
        if pending:
            self._manual_candidate_since = 0.0
            want = pending.get("mode", "AUTO")
            hw_ok = (want == "FOLLOW" and sample["hw_mode"] == 2) or (want == "AUTO" and sample["hw_mode"] == 1)
            if hw_ok and (sample["gps_ready"] or sample["path_len"] > 0 or abs(sample["pwm_l"]) > 5 or abs(sample["pwm_r"]) > 5 or want == "FOLLOW"):
                self._start(want, sample, now)
                self.pending_start = None
            elif now - float(pending.get("ts", now)) > 60.0:
                self.pending_start = None
            return

        # Immediate start when physical hw switch turns to AUTO or FOLLOW.
        # prev_hw != current means an actual transition happened (not just reconnect
        # on an already-running mode, which is handled by the recovery path below).
        if sample["hw_mode"] == 1 and prev_hw != 1:
            self._manual_candidate_since = 0.0
            self.pending_start = None
            self._start("AUTO", sample, now)
            return
        if sample["hw_mode"] == 2 and prev_hw != 2:
            self._manual_candidate_since = 0.0
            self.pending_start = None
            self._start("FOLLOW", sample, now)
            return

        # Recovery path after web server restart: robot already working, create log.
        if sample["hw_mode"] == 1 and sample["path_len"] > 0 and (abs(sample["pwm_l"]) > 5 or abs(sample["pwm_r"]) > 5):
            self._manual_candidate_since = 0.0
            self._start("AUTO", sample, now)
        elif sample["hw_mode"] == 2 and (abs(sample["pwm_l"]) > 5 or abs(sample["pwm_r"]) > 5):
            self._manual_candidate_since = 0.0
            self._start("FOLLOW", sample, now)
        elif self.manual_enable and sample["hw_mode"] == 0:
            if self._manual_candidate_since <= 0.0:
                self._manual_candidate_since = now
            if (now - self._manual_candidate_since) >= self.manual_start_hold_s:
                self._start("MANUAL", sample, now)
        else:
            self._manual_candidate_since = 0.0

    def _start(self, run_mode, sample, now):
        if self.active:
            self._finish("RESTART")
        stamp = datetime.fromtimestamp(now).strftime("%Y%m%d_%H%M%S")
        self.mission_id = _safe_mission_id(f"mission_{stamp}_{run_mode.lower()}")
        self.run_mode = run_mode
        self.csv_path = os.path.join(self.base_dir, f"{self.mission_id}.csv")
        self.summary_path = os.path.join(self.base_dir, f"{self.mission_id}.summary.json")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        self.active = True
        self._mode_mismatch_since = 0.0
        self._manual_candidate_since = 0.0
        self.last_sample_ts = 0.0
        self.last_active_ts = now
        self.stats = {
            "mission_id": self.mission_id,
            "run_mode": run_mode,
            "start_time": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "end_time": None,
            "status": "RUNNING",
            "start_lat": sample["lat"],
            "start_lng": sample["lng"],
            "end_lat": None,
            "end_lng": None,
            "duration_s": 0.0,
            "distance_m": 0.0,
            "path_len": sample["path_len"],
            "last_wp_idx": sample["wp_idx"],
            "waypoints_reached": [],
            "battery_start_pct": sample["battery_soc_pct"],
            "battery_end_pct": None,
            "battery_drop_pct": None,
            "voltage_min_v": sample["battery_voltage_v"] if math.isfinite(sample["battery_voltage_v"]) else None,
            "voltage_avg_v": None,
            "current_avg_a": None,
            "current_abs_avg_a": None,
            "current_abs_peak_a": None,
            "samples": 0,
            "_last_lat": sample["lat"],
            "_last_lng": sample["lng"],
            "_voltage_sum": 0.0,
            "_voltage_n": 0,
            "_current_sum": 0.0,
            "_current_abs_sum": 0.0,
            "_current_n": 0,
        }
        self._write_sample(sample, "MISSION_START")
        self.last_sample_ts = now

    def _finish(self, reason):
        if not self.active:
            return
        now = time.time()
        try:
            if self.stats:
                self.stats["end_time"] = datetime.fromtimestamp(now).isoformat(timespec="seconds")
                self.stats["status"] = reason
                start_ts = datetime.fromisoformat(self.stats["start_time"]).timestamp()
                self.stats["duration_s"] = max(0.0, now - start_ts)
                self._write_summary_locked()
        finally:
            try:
                if self.csv_file:
                    self.csv_file.flush()
                    self.csv_file.close()
            except Exception:
                pass
            self.active = False
            self.csv_file = None
            self.csv_writer = None
            self.mission_id = ""
            self.csv_path = ""
            self.summary_path = ""

    def _mode_mismatch_persisting(self, now):
        if self.mode_change_hold_s <= 0.0:
            return True
        if self._mode_mismatch_since <= 0.0:
            self._mode_mismatch_since = now
            return False
        return (now - self._mode_mismatch_since) >= self.mode_change_hold_s

    def _append_event(self, event):
        if not self.active or not self.csv_writer:
            return
        now = time.time()
        row = {k: "" for k in self.CSV_FIELDS}
        row.update({
            "iso_time": datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
            "elapsed_s": "",
            "mission_id": self.mission_id,
            "run_mode": self.run_mode,
            "event": event,
            "follow_status": self.latest_follow_status,
            "follow_debug": self.latest_follow_debug,
        })
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _write_sample(self, sample, event):
        if not self.csv_writer:
            return
        now = sample["ts"]
        elapsed = 0.0
        try:
            elapsed = now - datetime.fromisoformat(self.stats["start_time"]).timestamp()
        except Exception:
            pass
        sample["event"] = event
        sample["mission_id"] = self.mission_id
        sample["run_mode"] = self.run_mode
        sample["elapsed_s"] = round(elapsed, 3)
        sample["iso_time"] = datetime.fromtimestamp(now).isoformat(timespec="milliseconds")
        sample["follow_status"] = self.latest_follow_status
        sample["follow_debug"] = self.latest_follow_debug
        self.csv_writer.writerow({k: sample.get(k, "") for k in self.CSV_FIELDS})
        self.csv_file.flush()
        self._update_stats(sample)
        self._write_summary_locked()

    def _update_stats(self, sample):
        if not self.stats:
            return
        self.stats["samples"] = int(self.stats.get("samples", 0)) + 1
        self.stats["end_lat"] = sample["lat"]
        self.stats["end_lng"] = sample["lng"]
        self.stats["path_len"] = max(int(self.stats.get("path_len", 0)), int(sample["path_len"]))
        prev_wp = int(self.stats.get("last_wp_idx", -1))
        cur_wp = int(sample["wp_idx"])
        if cur_wp > prev_wp:
            for wp in range(max(0, prev_wp), cur_wp):
                if wp >= 0:
                    self.stats["waypoints_reached"].append({
                        "wp_index": wp,
                        "time": sample["iso_time"],
                        "lat": sample["lat"],
                        "lng": sample["lng"],
                    })
        self.stats["last_wp_idx"] = cur_wp

        last_lat = self.stats.get("_last_lat")
        last_lng = self.stats.get("_last_lng")
        d = _haversine_m(last_lat, last_lng, sample["lat"], sample["lng"])
        if math.isfinite(d) and 0.02 <= d <= 20.0:
            self.stats["distance_m"] = round(float(self.stats.get("distance_m", 0.0)) + d, 3)
        self.stats["_last_lat"] = sample["lat"]
        self.stats["_last_lng"] = sample["lng"]

        soc = sample["battery_soc_pct"]
        if soc is not None:
            self.stats["battery_end_pct"] = soc
        v = sample["battery_voltage_v"]
        if math.isfinite(v) and v >= 8.0:
            self.stats["voltage_min_v"] = v if self.stats["voltage_min_v"] is None else min(self.stats["voltage_min_v"], v)
            self.stats["_voltage_sum"] += v
            self.stats["_voltage_n"] += 1
        c = sample["battery_current_a"]
        if math.isfinite(c):
            self.stats["_current_sum"] += c
            self.stats["_current_abs_sum"] += abs(c)
            self.stats["_current_n"] += 1
            peak = self.stats.get("current_abs_peak_a")
            self.stats["current_abs_peak_a"] = abs(c) if peak is None else max(float(peak), abs(c))

    def _write_summary_locked(self):
        if not self.stats or not self.summary_path:
            return
        if self.stats.get("battery_start_pct") is not None and self.stats.get("battery_end_pct") is not None:
            self.stats["battery_drop_pct"] = round(float(self.stats["battery_start_pct"]) - float(self.stats["battery_end_pct"]), 2)
        if self.stats.get("_voltage_n", 0) > 0:
            self.stats["voltage_avg_v"] = round(self.stats["_voltage_sum"] / self.stats["_voltage_n"], 3)
        if self.stats.get("_current_n", 0) > 0:
            self.stats["current_avg_a"] = round(self.stats["_current_sum"] / self.stats["_current_n"], 3)
            self.stats["current_abs_avg_a"] = round(self.stats["_current_abs_sum"] / self.stats["_current_n"], 3)
        try:
            start_ts = datetime.fromisoformat(self.stats["start_time"]).timestamp()
            self.stats["duration_s"] = max(0.0, time.time() - start_ts)
        except Exception:
            pass
        public = _json_safe({k: v for k, v in self.stats.items() if not k.startswith("_")})
        tmp = self.summary_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(public, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.summary_path)

    def _parse_current_gps(self, data, now):
        try:
            vals = []
            for x in (data or []):
                if x is None:
                    vals.append(float("nan"))
                    continue
                try:
                    vals.append(float(x))
                except Exception:
                    vals.append(float("nan"))
        except Exception:
            return None
        if len(vals) < 4:
            return None

        def v(i, default=float("nan")):
            return vals[i] if i < len(vals) else default

        lat = v(0)
        lng = v(1)
        target_lat = v(34)
        target_lng = v(35)
        dist_to_target = _haversine_m(lat, lng, target_lat, target_lng)
        soc = v(27, -1.0)
        return {
            "ts": now,
            "hw_mode": int(round(v(3, 0.0))),
            "lat": lat,
            "lng": lng,
            "yaw_deg": math.degrees(v(2, 0.0)),
            "gps_fix": int(round(v(4, -1.0))),
            "gps_ready": bool(v(5, 0.0) >= 0.5),
            "lidar_min_front_m": v(9, -1.0),
            "lidar_obstacle": int(v(11, 0.0) >= 0.5),
            "pwm_l": int(round(v(13, 0.0))),
            "pwm_r": int(round(v(14, 0.0))),
            "enc_l": int(round(v(18, 0.0))),
            "enc_r": int(round(v(19, 0.0))),
            "wp_idx": int(round(v(32, -1.0))),
            "path_len": int(round(v(33, 0.0))),
            "target_lat": target_lat,
            "target_lng": target_lng,
            "dist_to_target_m": dist_to_target,
            "battery_soc_pct": int(round(soc)) if soc >= 0.0 else None,
            "battery_voltage_v": v(28),
            "battery_current_a": v(29),
            "battery_temp_c": v(30),
            "battery_status": int(round(v(31, 2.0))),
            "wheel_speed_l_kph": v(36, 0.0) * 3.6,
            "wheel_speed_r_kph": v(37, 0.0) * 3.6,
            "wheel_speed_avg_kph": v(38, 0.0) * 3.6,
            "num_sats": int(round(v(39, 0.0))),
            "segment": (
                f"{int(round(v(40, -1.0)))}→{int(round(v(41, -1.0)))}"
                if v(40, -1.0) >= 0.0 else "-"
            ),
        }

    def _rosbridge_loop(self):
        while True:
            ws = None
            try:
                ws = websocket.create_connection(ROSBRIDGE_URL, timeout=3)
                with self.lock:
                    self.last_rosbridge_ok_ts = time.time()
                    self.last_rosbridge_error = ""
                for topic, msg_type in (
                    ("/current_gps", "std_msgs/Float32MultiArray"),
                    ("/auto_control", "std_msgs/String"),
                    ("/emergency_stop", "std_msgs/String"),
                    ("/follow_vision_status", "std_msgs/String"),
                    ("/follow_debug", "std_msgs/String"),
                ):
                    ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": msg_type, "throttle_rate": 250}))
                while True:
                    raw = ws.recv()
                    payload = json.loads(raw)
                    if payload.get("op") != "publish":
                        continue
                    with self.lock:
                        self.last_rosbridge_ok_ts = time.time()
                    topic = payload.get("topic")
                    msg = payload.get("msg") or {}
                    if topic == "/current_gps":
                        self.handle_gps(msg.get("data", []))
                    elif topic == "/auto_control":
                        self.handle_control(msg.get("data", ""))
                    elif topic == "/emergency_stop":
                        self.handle_emergency(msg.get("data", ""))
                    elif topic == "/follow_vision_status":
                        self.handle_follow_status(msg.get("data", ""))
                    elif topic == "/follow_debug":
                        self.handle_follow_debug(msg.get("data", ""))
            except Exception as exc:
                with self.lock:
                    self.last_rosbridge_error = str(exc)[:180]
                time.sleep(2.0)
            finally:
                try:
                    if ws is not None:
                        ws.close()
                except Exception:
                    pass

    def _rosbridge_gps_loop(self):
        while True:
            ws = None
            try:
                ws = websocket.create_connection(ROSBRIDGE_URL, timeout=3)
                with self.lock:
                    self.last_rosbridge_ok_ts = time.time()
                    self.last_rosbridge_error = ""
                ws.send(json.dumps({
                    "op": "subscribe",
                    "topic": "/current_gps",
                    "type": "std_msgs/Float32MultiArray",
                    "throttle_rate": 1000,
                }))
                while True:
                    payload = json.loads(ws.recv())
                    if payload.get("op") == "publish" and payload.get("topic") == "/current_gps":
                        with self.lock:
                            self.last_rosbridge_ok_ts = time.time()
                        self.handle_gps((payload.get("msg") or {}).get("data", []))
            except Exception as exc:
                with self.lock:
                    self.last_rosbridge_error = str(exc)[:180]
                time.sleep(2.0)
            finally:
                try:
                    if ws is not None:
                        ws.close()
                except Exception:
                    pass

    def _ros2_topic_loop(self):
        try:
            import rclpy
            from std_msgs.msg import Float32MultiArray, String
        except Exception as exc:
            with self.lock:
                self.last_rosbridge_error = f"rclpy unavailable: {exc}"[:180]
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("web_datalog_bridge")

            node.create_subscription(
                Float32MultiArray,
                "/current_gps",
                lambda msg: self.handle_gps(list(msg.data)),
                10,
            )
            node.create_subscription(String, "/auto_control", lambda msg: self.handle_control(msg.data), 10)
            node.create_subscription(String, "/emergency_stop", lambda msg: self.handle_emergency(msg.data), 10)
            node.create_subscription(String, "/follow_vision_status", lambda msg: self.handle_follow_status(msg.data), 10)
            node.create_subscription(String, "/follow_debug", lambda msg: self.handle_follow_debug(msg.data), 10)

            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=1.0)
        except Exception as exc:
            with self.lock:
                self.last_rosbridge_error = f"rclpy datalog: {exc}"[:180]


class TrainingRecorder:
    CSV_FIELDS = [
        "iso_time", "elapsed_s", "training_id", "event", "label",
        "hw_mode", "lat", "lng", "yaw_deg", "gps_fix", "gps_ready",
        "ref_mode", "ref_path_len", "ref_segment_idx", "ref_progress_m",
        "ref_cross_track_m", "ref_along_segment_m", "ref_segment_len_m",
        "ref_heading_deg", "ref_heading_error_deg", "ref_dist_to_start_m", "ref_dist_to_end_m",
        "lidar_min_front_m", "lidar_obstacle", "pwm_l", "pwm_r",
        "enc_l", "enc_r", "wheel_speed_l_kph", "wheel_speed_r_kph", "wheel_speed_avg_kph",
        "battery_soc_pct", "battery_voltage_v", "battery_current_a", "battery_temp_c", "battery_status",
        "wp_idx", "path_len", "target_lat", "target_lng", "dist_to_target_m",
        *QUALITY_EXTRA_FIELDS,
    ]

    def __init__(self, base_dir, parser):
        self.base_dir = os.path.abspath(os.path.join(base_dir, "training"))
        os.makedirs(self.base_dir, exist_ok=True)
        self.parser = parser
        self.lock = threading.RLock()
        self.active = False
        self.training_id = ""
        self.csv_path = ""
        self.jsonl_path = ""
        self.summary_path = ""
        self.csv_file = None
        self.jsonl_file = None
        self.csv_writer = None
        self.started_at = 0.0
        self.last_sample_ts = 0.0
        self.sample_interval_s = float(os.getenv("TRAINING_SAMPLE_INTERVAL_S", "0.20"))
        self.ref_mode = "UNKNOWN"
        self.ref_path = []
        self.label = ""
        self.stats = {}
        self.prev_row_for_quality = None

    def start(self, map_state=None, label=""):
        with self.lock:
            if self.active:
                self.stop("RESTART")
            now = time.time()
            map_state = map_state or _load_map_state()
            self.ref_mode = str(map_state.get("map_mode") or "UNKNOWN").upper()
            self.ref_path = _clean_waypoints(map_state.get("grid_waypoints") or [])
            self.label = str(label or "").strip()[:80]
            stamp = datetime.fromtimestamp(now).strftime("%Y%m%d_%H%M%S")
            self.training_id = _safe_mission_id(f"training_{stamp}_{self.ref_mode.lower()}")
            self.csv_path = os.path.join(self.base_dir, f"{self.training_id}.csv")
            self.jsonl_path = os.path.join(self.base_dir, f"{self.training_id}.jsonl")
            self.summary_path = os.path.join(self.base_dir, f"{self.training_id}.summary.json")
            self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
            self.jsonl_file = open(self.jsonl_path, "w", encoding="utf-8")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDS)
            self.csv_writer.writeheader()
            self.started_at = now
            self.last_sample_ts = 0.0
            self.active = True
            self.stats = {
                "training_id": self.training_id,
                "run_mode": "TRAINING",
                "label": self.label,
                "ref_mode": self.ref_mode,
                "ref_path_len": len(self.ref_path),
                "ref_path": self.ref_path,
                "start_time": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
                "end_time": None,
                "status": "RUNNING",
                "duration_s": 0.0,
                "samples": 0,
                "manual_samples": 0,
                "distance_m": 0.0,
                "cross_track_abs_avg_m": None,
                "cross_track_abs_peak_m": None,
                "heading_abs_avg_deg": None,
                "heading_abs_peak_deg": None,
                "current_abs_avg_a": None,
                "current_abs_peak_a": None,
                "battery_start_pct": None,
                "battery_end_pct": None,
                "voltage_min_v": None,
                "quality_good": 0,
                "quality_usable": 0,
                "quality_bad": 0,
                "gps_jump_rows": 0,
                "speed_invalid_rows": 0,
                "battery_invalid_rows": 0,
                "_last_lat": None,
                "_last_lng": None,
                "_cross_abs_sum": 0.0,
                "_cross_n": 0,
                "_heading_abs_sum": 0.0,
                "_heading_n": 0,
                "_current_abs_sum": 0.0,
                "_current_n": 0,
            }
            self.prev_row_for_quality = None
            self._write_summary_locked()
            return self.status()

    def stop(self, reason="STOP"):
        with self.lock:
            if not self.active:
                return self.status()
            now = time.time()
            self.stats["end_time"] = datetime.fromtimestamp(now).isoformat(timespec="seconds")
            self.stats["status"] = reason
            self._write_summary_locked()
            try:
                if self.csv_file:
                    self.csv_file.flush()
                    self.csv_file.close()
                if self.jsonl_file:
                    self.jsonl_file.flush()
                    self.jsonl_file.close()
            finally:
                self.csv_file = None
                self.jsonl_file = None
                self.csv_writer = None
                self.active = False
                self.prev_row_for_quality = None
            return self.status()

    def status(self):
        with self.lock:
            return {
                "active": self.active,
                "training_id": self.training_id if self.active else "",
                "ref_mode": self.ref_mode,
                "ref_path_len": len(self.ref_path),
                "sample_interval_s": self.sample_interval_s,
                "summary": _json_safe({k: v for k, v in self.stats.items() if not k.startswith("_")}) if (self.active and self.stats) else {},
            }

    def list_sessions(self):
        out = []
        try:
            for name in os.listdir(self.base_dir):
                if not name.endswith(".summary.json"):
                    continue
                try:
                    with open(os.path.join(self.base_dir, name), "r", encoding="utf-8") as f:
                        item = json.load(f)
                    if "quality_good" not in item:
                        training_id = _safe_mission_id(item.get("training_id", ""))
                        item.update(_quality_summary_from_csv(os.path.join(self.base_dir, f"{training_id}.csv")))
                    out.append(_json_safe(item))
                except Exception:
                    continue
        except Exception:
            pass
        out.sort(key=lambda x: str(x.get("start_time") or ""), reverse=True)
        return out

    def download_path(self, training_id, fmt):
        training_id = _safe_mission_id(training_id)
        if not training_id:
            return None
        if fmt == "csv":
            path = os.path.join(self.base_dir, f"{training_id}.csv")
        elif fmt == "jsonl":
            path = os.path.join(self.base_dir, f"{training_id}.jsonl")
        elif fmt == "json":
            path = os.path.join(self.base_dir, f"{training_id}.summary.json")
        elif fmt == "clean_csv":
            path = os.path.join(self.base_dir, f"{training_id}.clean.csv")
            if not _make_clean_csv(os.path.join(self.base_dir, f"{training_id}.csv"), path):
                return None
        elif fmt == "clean_jsonl":
            path = os.path.join(self.base_dir, f"{training_id}.clean.jsonl")
            if not _make_clean_jsonl(os.path.join(self.base_dir, f"{training_id}.csv"), path):
                return None
        elif fmt == "zip":
            path = os.path.join(self.base_dir, f"{training_id}.zip")
            self._make_zip(training_id, path)
        else:
            return None
        return path if os.path.isfile(path) else None

    def _make_zip(self, training_id, zip_path):
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as z:
            _make_clean_csv(os.path.join(self.base_dir, f"{training_id}.csv"), os.path.join(self.base_dir, f"{training_id}.clean.csv"))
            _make_clean_jsonl(os.path.join(self.base_dir, f"{training_id}.csv"), os.path.join(self.base_dir, f"{training_id}.clean.jsonl"))
            for suffix in (".csv", ".jsonl", ".summary.json", ".clean.csv", ".clean.jsonl"):
                path = os.path.join(self.base_dir, f"{training_id}{suffix}")
                if os.path.isfile(path):
                    z.write(path, arcname=os.path.basename(path))

    def handle_gps(self, data):
        now = time.time()
        with self.lock:
            if not self.active:
                return
            if (now - self.last_sample_ts) < self.sample_interval_s:
                return
        sample = self.parser(data, now)
        if sample is None:
            return
        with self.lock:
            if not self.active:
                return
            row = self._make_row(sample)
            self.csv_writer.writerow({k: row.get(k, "") for k in self.CSV_FIELDS})
            self.csv_file.flush()
            self.jsonl_file.write(json.dumps(_json_safe(row), ensure_ascii=False) + "\n")
            self.jsonl_file.flush()
            self.last_sample_ts = now
            self._update_stats_locked(row)
            self._write_summary_locked()

    def _make_row(self, sample):
        elapsed = max(0.0, sample["ts"] - self.started_at)
        metrics = _closest_path_metrics(sample["lat"], sample["lng"], sample["yaw_deg"], self.ref_path)
        row = {
            "iso_time": datetime.fromtimestamp(sample["ts"]).isoformat(timespec="milliseconds"),
            "elapsed_s": round(elapsed, 3),
            "training_id": self.training_id,
            "event": "SAMPLE",
            "label": self.label,
            "ref_mode": self.ref_mode,
            "ref_path_len": len(self.ref_path),
            **sample,
            **metrics,
        }
        row.update(_quality_for_sample(row, self.prev_row_for_quality))
        self.prev_row_for_quality = dict(row)
        return row

    def _update_stats_locked(self, row):
        self.stats["samples"] = int(self.stats.get("samples", 0)) + 1
        quality = str(row.get("row_quality") or "BAD").upper()
        if quality == "GOOD":
            self.stats["quality_good"] = int(self.stats.get("quality_good", 0)) + 1
        elif quality == "USABLE":
            self.stats["quality_usable"] = int(self.stats.get("quality_usable", 0)) + 1
        else:
            self.stats["quality_bad"] = int(self.stats.get("quality_bad", 0)) + 1
        if _to_int(row.get("gps_jump_flag"), 0):
            self.stats["gps_jump_rows"] = int(self.stats.get("gps_jump_rows", 0)) + 1
        if not _to_int(row.get("speed_valid"), 0):
            self.stats["speed_invalid_rows"] = int(self.stats.get("speed_invalid_rows", 0)) + 1
        if not _to_int(row.get("battery_valid"), 0):
            self.stats["battery_invalid_rows"] = int(self.stats.get("battery_invalid_rows", 0)) + 1
        if int(row.get("hw_mode", -1)) == 0:
            self.stats["manual_samples"] = int(self.stats.get("manual_samples", 0)) + 1
        self.stats["battery_end_pct"] = row.get("battery_soc_pct")
        if self.stats.get("battery_start_pct") is None and row.get("battery_soc_pct") is not None:
            self.stats["battery_start_pct"] = row.get("battery_soc_pct")
        lat = row.get("lat")
        lng = row.get("lng")
        last_lat = self.stats.get("_last_lat")
        last_lng = self.stats.get("_last_lng")
        d = _haversine_m(last_lat, last_lng, lat, lng)
        if math.isfinite(d) and 0.02 <= d <= 20.0 and not _to_int(row.get("gps_jump_flag"), 0):
            self.stats["distance_m"] = round(float(self.stats.get("distance_m", 0.0)) + d, 3)
        self.stats["_last_lat"] = lat
        self.stats["_last_lng"] = lng

        cross = row.get("ref_cross_track_m")
        if isinstance(cross, (int, float)) and math.isfinite(cross):
            a = abs(cross)
            self.stats["_cross_abs_sum"] += a
            self.stats["_cross_n"] += 1
            peak = self.stats.get("cross_track_abs_peak_m")
            self.stats["cross_track_abs_peak_m"] = a if peak is None else max(float(peak), a)
        hdg = row.get("ref_heading_error_deg")
        if isinstance(hdg, (int, float)) and math.isfinite(hdg):
            a = abs(hdg)
            self.stats["_heading_abs_sum"] += a
            self.stats["_heading_n"] += 1
            peak = self.stats.get("heading_abs_peak_deg")
            self.stats["heading_abs_peak_deg"] = a if peak is None else max(float(peak), a)
        cur = row.get("battery_current_a")
        if isinstance(cur, (int, float)) and math.isfinite(cur) and _to_int(row.get("battery_valid"), 0):
            a = abs(cur)
            self.stats["_current_abs_sum"] += a
            self.stats["_current_n"] += 1
            peak = self.stats.get("current_abs_peak_a")
            self.stats["current_abs_peak_a"] = a if peak is None else max(float(peak), a)
        volt = row.get("battery_voltage_v")
        if isinstance(volt, (int, float)) and math.isfinite(volt) and _to_int(row.get("battery_valid"), 0):
            vmin = self.stats.get("voltage_min_v")
            self.stats["voltage_min_v"] = volt if vmin is None else min(float(vmin), volt)

    def _write_summary_locked(self):
        if not self.summary_path or not self.stats:
            return
        try:
            self.stats["duration_s"] = max(0.0, time.time() - self.started_at)
            if self.stats.get("_cross_n", 0) > 0:
                self.stats["cross_track_abs_avg_m"] = round(self.stats["_cross_abs_sum"] / self.stats["_cross_n"], 3)
            if self.stats.get("_heading_n", 0) > 0:
                self.stats["heading_abs_avg_deg"] = round(self.stats["_heading_abs_sum"] / self.stats["_heading_n"], 2)
            if self.stats.get("_current_n", 0) > 0:
                self.stats["current_abs_avg_a"] = round(self.stats["_current_abs_sum"] / self.stats["_current_n"], 3)
            public = _json_safe({k: v for k, v in self.stats.items() if not k.startswith("_")})
            tmp = self.summary_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(public, f, ensure_ascii=False, indent=2)
            os.replace(tmp, self.summary_path)
        except Exception:
            pass


datalogger = MissionDatalogger(DATALOG_DIR)
training_recorder = TrainingRecorder(DATALOG_DIR, datalogger._parse_current_gps)
datalogger.start_thread()

# ================== HTTP ==================

@app.route("/")
def index():
    response = make_response(render_template("index.html"))
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

@app.route("/sw.js")
def service_worker():
    response = send_from_directory(
        os.path.join(BASE_DIR, "static"),
        "sw.js",
        mimetype="application/javascript",
    )
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    response.headers["Service-Worker-Allowed"] = "/"
    return response

@app.route("/manifest.webmanifest")
def web_manifest():
    response = send_from_directory(
        os.path.join(BASE_DIR, "static"),
        "manifest.webmanifest",
        mimetype="application/manifest+json",
    )
    response.headers["Cache-Control"] = "public, max-age=3600"
    return response

@app.route("/offline")
def offline_page():
    response = send_from_directory(os.path.join(BASE_DIR, "static"), "offline.html")
    response.headers["Cache-Control"] = "public, max-age=86400"
    return response

@app.route("/amr_description/meshes/<path:filename>")
def amr_description_mesh(filename):
    meshes_dir = os.path.abspath(os.path.join(AMR_DESCRIPTION_DIR, "meshes"))
    return send_from_directory(meshes_dir, filename)

@app.route("/status")
def status():
    return jsonify({
        "web": "ONLINE",
        "ros": "UNKNOWN"
    })


@app.route("/api/datalog", methods=["GET"])
def datalog_list():
    mode = (request.args.get("mode") or "ALL").strip().upper()
    status = datalogger.status_snapshot()
    return jsonify({
        **status,
        "missions": datalogger.list_missions(mode),
    })


@app.route("/api/datalog/current_gps", methods=["POST"])
def datalog_current_gps():
    payload = request.get_json(silent=True) or {}
    data = payload.get("data", [])
    datalogger.handle_gps(data)
    training_recorder.handle_gps(data)
    return jsonify({"ok": True, **datalogger.status_snapshot()})


@app.route("/api/training", methods=["GET"])
def training_list():
    return jsonify({
        **training_recorder.status(),
        "sessions": training_recorder.list_sessions(),
    })


@app.route("/api/training/start", methods=["POST"])
def training_start():
    payload = request.get_json(silent=True) or {}
    label = payload.get("label", "")
    return jsonify({"ok": True, **training_recorder.start(_load_map_state(), label=label)})


@app.route("/api/training/stop", methods=["POST"])
def training_stop():
    payload = request.get_json(silent=True) or {}
    reason = str(payload.get("reason") or "STOP").upper()
    return jsonify({"ok": True, **training_recorder.stop(reason)})


@app.route("/api/training/download/<training_id>")
def training_download(training_id):
    fmt = (request.args.get("format") or "csv").strip().lower()
    if fmt not in ("csv", "jsonl", "json", "clean_csv", "clean_jsonl", "zip"):
        abort(400)
    path = training_recorder.download_path(training_id, fmt)
    if not path:
        abort(404)
    mimetype = {
        "csv": "text/csv",
        "jsonl": "application/x-ndjson",
        "json": "application/json",
        "clean_csv": "text/csv",
        "clean_jsonl": "application/x-ndjson",
        "zip": "application/zip",
    }[fmt]
    return send_file(path, mimetype=mimetype, as_attachment=True, download_name=os.path.basename(path))


@app.route("/api/datalog/download/<mission_id>")
def datalog_download(mission_id):
    fmt = (request.args.get("format") or "csv").strip().lower()
    if fmt not in ("csv", "clean_csv", "json", "data_json", "zip"):
        abort(400)
    path = datalogger.download_path(mission_id, fmt)
    if not path:
        abort(404)
    mimetype = {
        "csv": "text/csv",
        "clean_csv": "text/csv",
        "json": "application/json",
        "data_json": "application/json",
        "zip": "application/zip",
    }[fmt]
    return send_file(path, mimetype=mimetype, as_attachment=True, download_name=os.path.basename(path))

@app.route("/api/camera_status")
def camera_status():
    stab_capable = (cv2 is not None and np is not None and CAMERA_MODE != "mjpeg")

    if CAMERA_MODE == "mjpeg":
        mjpeg_ok, mjpeg_msg = _probe_mjpeg()
        ok = mjpeg_ok
        src = CAMERA_MJPEG_URL
        msg = mjpeg_msg
    elif CAMERA_MODE == "usb":
        ok, msg = _probe_usb_fast()
        src = CAMERA_DEVICE
    else:  # auto
        # Prefer local USB camera for low latency. Do not block the UI by
        # probing the optional MJPEG source first when it is usually absent.
        ok, msg = _probe_usb_fast()
        src = CAMERA_DEVICE
        if not ok:
            mjpeg_ok, mjpeg_msg = _probe_mjpeg()
            if mjpeg_ok:
                ok = True
                src = CAMERA_MJPEG_URL
                msg = mjpeg_msg
            else:
                msg = f"usb={msg} | mjpeg={mjpeg_msg}"

    status_code = 200 if ok else 503
    return jsonify({
        "status": "ok" if ok else "error",
        "url": "/video_feed",
        "mode": CAMERA_MODE,
        "source": src,
        "msg": msg,
        "stabilize_capable": stab_capable
    }), status_code

# ---------- ARCHIVE API ----------

@app.route("/api/profiles")
def get_profiles():
    profiles = []
    os.makedirs(DATASET_DIR, exist_ok=True)
    for name in sorted(os.listdir(DATASET_DIR)):
        folder = os.path.join(DATASET_DIR, name)
        if os.path.isdir(folder):
            images = sorted(
                f for f in os.listdir(folder)
                if os.path.isfile(os.path.join(folder, f))
                and f.lower().endswith((".jpg", ".jpeg", ".png", ".webp", ".bmp", ".gif"))
            )
            profiles.append({
                "name": name,
                "images": images
            })
    return jsonify(profiles)


@app.route("/api/profiles", methods=["POST"])
def create_profile():
    data = request.get_json(silent=True) or {}
    owner = data.get("name") or request.form.get("owner_name") or ""
    safe, folder = _dataset_owner_dir(owner)
    if not safe:
        return jsonify({"status": "error", "msg": "invalid owner name"}), 400
    if os.path.exists(folder):
        return jsonify({"status": "ok", "name": safe, "exists": True})
    os.makedirs(folder, exist_ok=True)
    return jsonify({"status": "ok", "name": safe, "exists": False})


@app.route("/api/map_state", methods=["GET"])
def get_map_state():
    return jsonify(_load_map_state())


@app.route("/api/map_state", methods=["POST"])
def save_map_state():
    data = request.get_json(silent=True) or {}
    state = _save_map_state(data)
    return jsonify({"status": "ok", "state": state})


@app.route("/api/map_state", methods=["DELETE"])
def clear_map_state():
    state = _save_map_state(_default_map_state())
    return jsonify({"status": "ok", "state": state})


@app.route("/api/saved_paths", methods=["GET"])
def list_saved_paths():
    return jsonify({"status": "ok", "paths": _list_saved_paths()})


@app.route("/api/saved_paths", methods=["POST"])
def save_named_path():
    data = request.get_json(silent=True) or {}
    name = _safe_path_name(data.get("name") or "")
    if not name:
        return jsonify({"status": "error", "msg": "invalid path name"}), 400
    payload = _save_path_library_item(data)
    return jsonify({"status": "ok", "path": _saved_path_summary(payload, name=payload.get("name"))})


@app.route("/api/saved_paths/<path:name>", methods=["GET"])
def load_named_path(name):
    item = _load_saved_path(name)
    if item is None:
        return jsonify({"status": "error", "msg": "not found"}), 404
    state = _save_map_state(item["state"])
    return jsonify({"status": "ok", "name": item["name"], "state": state})


@app.route("/api/saved_paths/<path:name>", methods=["DELETE"])
def delete_named_path(name):
    safe, path = _saved_path_file(name)
    if not os.path.isfile(path):
        return jsonify({"status": "error", "msg": "not found"}), 404
    os.remove(path)
    return jsonify({"status": "ok", "name": safe})


@app.route("/api/ui_state", methods=["GET"])
def get_ui_state():
    return jsonify(_load_ui_state())


@app.route("/api/ui_state", methods=["POST"])
def save_ui_state():
    data = request.get_json(silent=True) or {}
    state = _save_ui_state(data)
    return jsonify({"status": "ok", "state": state})

@app.route("/upload", methods=["POST"])
def upload_photo():
    if "photo" not in request.files:
        return jsonify({"status": "error", "msg": "no file"})

    file = request.files["photo"]
    owner = request.form.get("owner_name")
    safe_owner, folder = _dataset_owner_dir(owner)
    if not safe_owner:
        return jsonify({"status": "error", "msg": "invalid owner"}), 400
    os.makedirs(folder, exist_ok=True)

    filename = secure_filename(file.filename)
    if not filename:
        return jsonify({"status": "error", "msg": "invalid filename"}), 400
    file.save(os.path.join(folder, filename))

    return jsonify({"status": "ok", "owner": safe_owner, "file": filename})

@app.route("/api/capture_owner", methods=["POST"])
def capture_owner():
    owner = (request.form.get("owner_name") or "").strip()
    safe_owner, folder = _dataset_owner_dir(owner)
    if not safe_owner:
        return jsonify({"status": "error", "msg": "invalid owner"}), 400
    if cv2 is None:
        return jsonify({"status": "error", "msg": "opencv unavailable"}), 503

    _ensure_usb_worker()
    deadline = time.time() + 1.2
    frame = None
    while time.time() < deadline:
        frame = _get_latest_usb_frame(max_age_s=1.2)
        if frame is not None:
            break
        time.sleep(0.02)
    if frame is None:
        return jsonify({"status": "error", "msg": "no camera frame"}), 503

    os.makedirs(folder, exist_ok=True)
    filename = f"capture_{int(time.time() * 1000)}.jpg"
    full_path = os.path.join(folder, filename)
    ok = cv2.imwrite(full_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    if not ok:
        return jsonify({"status": "error", "msg": "save failed"}), 500
    return jsonify({"status": "ok", "file": filename})

@app.route("/api/rename_folder", methods=["POST"])
def rename_folder():
    data = request.json or {}
    old_safe, old = _dataset_owner_dir(data.get("old_name"))
    new_safe, new = _dataset_owner_dir(data.get("new_name"))
    if not old_safe or not new_safe:
        return jsonify({"status": "error", "msg": "invalid name"}), 400

    if os.path.exists(old):
        if os.path.exists(new):
            return jsonify({"status": "error", "msg": "target exists"}), 409
        os.rename(old, new)
        return jsonify({"status": "ok", "name": new_safe})
    return jsonify({"status": "error", "msg": "not found"}), 404

@app.route("/api/delete_folder", methods=["DELETE"])
def delete_folder():
    name = request.args.get("name")
    safe_owner, folder = _dataset_owner_dir(name)
    if not safe_owner:
        return jsonify({"status": "error", "msg": "invalid owner"}), 400

    if os.path.exists(folder):
        # Owner folders may contain hidden files or nested artifacts from uploads.
        # Remove recursively so "delete owner" behaves like users expect.
        shutil.rmtree(folder)
        return jsonify({"status": "ok", "owner": safe_owner})
    return jsonify({"status": "error", "msg": "not found"}), 404


@app.route("/api/delete_owner_image", methods=["DELETE"])
def delete_owner_image():
    owner = request.args.get("owner", "")
    image = (request.args.get("image", "") or "").strip()
    safe_owner, folder = _dataset_owner_dir(owner)
    if not safe_owner or not image or "/" in image or "\\" in image or not os.path.basename(image) == image:
        return jsonify({"status": "error", "msg": "invalid request"}), 400
    full_path = os.path.abspath(os.path.join(folder, image))
    if not full_path.startswith(os.path.abspath(folder) + os.sep):
        return jsonify({"status": "error", "msg": "invalid path"}), 400
    if not os.path.isfile(full_path):
        return jsonify({"status": "error", "msg": "not found"}), 404
    os.remove(full_path)
    return jsonify({"status": "ok", "owner": safe_owner, "file": image})

# ---------- STATIC DATASET (ให้ frontend โหลดรูปได้) ----------

@app.route("/dataset/<path:filename>")
def dataset_files(filename):
    safe_rel = os.path.normpath(filename).lstrip("/\\")
    full_path = os.path.join(DATASET_DIR, safe_rel)
    if not os.path.isfile(full_path):
        return abort(404)
    return send_from_directory(DATASET_DIR, safe_rel)

@app.route("/video_feed")
def video_feed():
    req_stab = request.args.get("stabilize", "").strip().lower()
    if req_stab in ("1", "true", "yes", "on"):
        stabilize = True
    elif req_stab in ("0", "false", "no", "off"):
        stabilize = False
    else:
        stabilize = CAMERA_STABILIZE_DEFAULT
    req_follow_ai = request.args.get("follow_ai", "").strip().lower()
    follow_ai = req_follow_ai in ("1", "true", "yes", "on")
    req_fast = request.args.get("fast", "").strip().lower()
    fast = req_fast in ("1", "true", "yes", "on")

    use_mjpeg = False
    if CAMERA_MODE == "mjpeg":
        use_mjpeg = True
    elif CAMERA_MODE == "auto":
        use_mjpeg, _ = _probe_mjpeg()

    def generate():
        if use_mjpeg:
            yield from _iter_mjpeg_proxy()
        else:
            # Multi-client safe: web preview + follow tracker can read concurrently.
            yield from _iter_usb_mjpeg(stabilize=stabilize, token=None, follow_ai=follow_ai, fast=fast)

    resp = Response(
        stream_with_context(generate()),
        content_type="multipart/x-mixed-replace; boundary=frame"
    )
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp


@app.route("/video_frame.jpg")
def video_frame_jpg():
    if cv2 is None:
        return Response(status=503)
    req_fast = request.args.get("fast", "").strip().lower()
    fast_mode = req_fast in ("1", "true", "yes", "on")
    req_stab = request.args.get("stabilize", "").strip().lower()
    stabilize = req_stab in ("1", "true", "yes", "on")
    req_follow_ai = request.args.get("follow_ai", "").strip().lower()
    follow_ai = req_follow_ai in ("1", "true", "yes", "on")
    _ensure_usb_worker()
    deadline = time.time() + 1.8
    frame = None
    while time.time() < deadline:
        frame = _get_latest_usb_frame(max_age_s=0.7)
        if frame is not None:
            break
        time.sleep(0.02)
    if frame is None:
        return Response(status=503)
    if fast_mode and frame is not None and frame.size > 0:
        try:
            h, w = frame.shape[:2]
            if w > CAMERA_FAST_WIDTH > 0:
                new_h = max(120, int(h * (CAMERA_FAST_WIDTH / float(w))))
                frame = cv2.resize(frame, (CAMERA_FAST_WIDTH, new_h), interpolation=cv2.INTER_AREA)
        except Exception:
            pass
    if stabilize:
        with _snapshot_stab_lock:
            frame = _apply_soft_stabilization(frame, _snapshot_stab_state)
    if follow_ai:
        try:
            frame = _auto_contrast_frame(frame)
            frame = _apply_follow_overlay(frame)
        except Exception:
            pass
    q = CAMERA_FAST_QUALITY if fast_mode else 70
    q = max(30, min(95, int(q)))
    ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    if not ok:
        return Response(status=503)
    resp = Response(jpeg.tobytes(), mimetype="image/jpeg")
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


@app.route("/api/follow_preview_status")
def follow_preview_status():
    with _follow_overlay_lock:
        payload = dict(_follow_overlay_status)
        overlay_ts = float(_follow_overlay_state.get("ts", 0.0) or 0.0)
    payload["age_s"] = round(max(0.0, time.time() - overlay_ts), 2) if overlay_ts > 0 else None
    return jsonify(payload)


def _probe_mjpeg():
    try:
        r = requests.get(CAMERA_MJPEG_URL, stream=True, timeout=1.5)
        ctype = (r.headers.get("Content-Type") or "").lower()
        ok = (r.status_code == 200) and (
            "multipart" in ctype or "jpeg" in ctype or "octet-stream" in ctype
        )
        r.close()
        return ok, f"http={r.status_code} type={ctype}"
    except Exception as e:
        return False, str(e)


def _open_usb_capture():
    if cv2 is None:
        return None
    # Support /dev/video0 or numeric index
    if CAMERA_DEVICE.isdigit():
        cap = cv2.VideoCapture(int(CAMERA_DEVICE))
    else:
        cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not cap.isOpened() and CAMERA_DEVICE.startswith("/dev/video"):
            try:
                idx = int(CAMERA_DEVICE.replace("/dev/video", "", 1))
                cap = cv2.VideoCapture(idx)
            except Exception:
                pass

    if not cap or not cap.isOpened():
        return None

    # Ask camera for MJPEG when possible to reduce USB bandwidth/latency.
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    # Keep camera buffer short to reduce end-to-end latency.
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap


def _probe_usb():
    if cv2 is None:
        return False, "opencv-python not installed"
    global _usb_stream_active
    # If an active stream already owns the camera, treat as healthy/available.
    with _stream_guard_lock:
        active = _usb_stream_active
    if active > 0:
        return True, "usb streaming"

    cap = None
    try:
        # If worker already has fresh frame, camera is healthy.
        with _usb_worker_lock:
            ts = _usb_latest_ts
        if ts > 0 and (time.time() - ts) < 1.5:
            return True, "usb worker frame ok"
        for _ in range(20):
            cap = _open_usb_capture()
            if cap is not None:
                break
            time.sleep(0.10)
        if cap is None:
            return False, f"cannot open {CAMERA_DEVICE}"
        ok, _ = cap.read()
        return (ok, "usb ok" if ok else "read failed")
    finally:
        if cap is not None:
            cap.release()


def _probe_usb_fast():
    if cv2 is None:
        return False, "opencv-python not installed"
    global _usb_stream_active
    with _stream_guard_lock:
        active = _usb_stream_active
    if active > 0:
        return True, "usb streaming"

    # Start the shared worker and wait briefly for its first frame. This avoids
    # opening the same camera twice and keeps /api/camera_status responsive.
    _ensure_usb_worker()
    deadline = time.time() + 0.8
    while time.time() < deadline:
        with _usb_worker_lock:
            ts = _usb_latest_ts
        if ts > 0 and (time.time() - ts) < 1.5:
            return True, "usb worker frame ok"
        time.sleep(0.03)
    if isinstance(CAMERA_DEVICE, str) and CAMERA_DEVICE.startswith("/dev/") and os.path.exists(CAMERA_DEVICE):
        return True, "usb device present, warming up"
    return False, f"no fresh usb frame from {CAMERA_DEVICE}"


def _iter_mjpeg_proxy():
    try:
        with requests.get(CAMERA_MJPEG_URL, stream=True, timeout=5) as r:
            if r.status_code != 200:
                return
            for chunk in r.iter_content(chunk_size=4096):
                if chunk:
                    yield chunk
    except Exception:
        return


def _ensure_usb_worker():
    global _usb_worker_thread, _usb_worker_stop
    with _usb_worker_lock:
        if _usb_worker_thread is not None and _usb_worker_thread.is_alive():
            return
        _usb_worker_stop = False
        _usb_worker_thread = threading.Thread(target=_usb_worker_loop, daemon=True)
        _usb_worker_thread.start()


def _usb_worker_loop():
    global _usb_latest_frame, _usb_latest_ts, _usb_worker_stop
    cap = None
    try:
        while not _usb_worker_stop:
            if cap is None:
                cap = _open_usb_capture()
                if cap is None:
                    time.sleep(0.20)
                    continue

            # Drain stale buffered frames first. Some V4L2 drivers ignore BUFFERSIZE,
            # so explicitly grabbing a couple of frames keeps latency from building up.
            try:
                for _ in range(6):
                    cap.grab()
            except Exception:
                pass

            ok, frame = cap.read()
            if not ok or frame is None or getattr(frame, "size", 0) == 0:
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(0.05)
                continue
            with _usb_worker_lock:
                _usb_latest_frame = frame
                _usb_latest_ts = time.time()
            # Favor low latency over exact FPS pacing.
            time.sleep(0.005)
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


def _get_latest_usb_frame(max_age_s=2.0):
    with _usb_worker_lock:
        frame = _usb_latest_frame
        ts = _usb_latest_ts
    if frame is None:
        return None
    if ts <= 0 or (time.time() - ts) > max_age_s:
        return None
    try:
        return frame.copy()
    except Exception:
        return None


def _apply_soft_stabilization(frame, st):
    if cv2 is None or np is None:
        return frame
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prev_gray = st.get("prev_gray")
        if prev_gray is None:
            st["prev_gray"] = gray
            return frame

        p0 = cv2.goodFeaturesToTrack(
            prev_gray,
            maxCorners=120,
            qualityLevel=0.02,
            minDistance=15,
            blockSize=3
        )
        if p0 is None or len(p0) < 12:
            st["prev_gray"] = gray
            return frame

        p1, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None)
        if p1 is None or status is None:
            st["prev_gray"] = gray
            return frame

        good_prev = p0[status.flatten() == 1]
        good_cur = p1[status.flatten() == 1]
        if len(good_prev) < 12:
            st["prev_gray"] = gray
            return frame

        m, _ = cv2.estimateAffinePartial2D(good_prev, good_cur)
        if m is None:
            st["prev_gray"] = gray
            return frame

        dx = float(m[0, 2])
        dy = float(m[1, 2])
        da = float(np.arctan2(m[1, 0], m[0, 0]))

        # Low-pass trajectory to keep intentional motion, remove high-frequency shake.
        beta = 0.92
        st["lp_dx"] = beta * st.get("lp_dx", 0.0) + (1.0 - beta) * dx
        st["lp_dy"] = beta * st.get("lp_dy", 0.0) + (1.0 - beta) * dy
        st["lp_da"] = beta * st.get("lp_da", 0.0) + (1.0 - beta) * da

        jx = dx - st["lp_dx"]
        jy = dy - st["lp_dy"]
        ja = da - st["lp_da"]

        h, w = frame.shape[:2]
        m2 = cv2.getRotationMatrix2D((w * 0.5, h * 0.5), -np.degrees(ja), 1.0)
        m2[0, 2] += -jx
        m2[1, 2] += -jy

        out = cv2.warpAffine(
            frame,
            m2,
            (w, h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REFLECT
        )
        st["prev_gray"] = gray
        return out
    except Exception:
        return frame


def _selected_follow_owner():
    try:
        owner = str(_load_ui_state().get("follow_target") or "").strip()
    except Exception:
        return ""
    if not owner or owner.upper() == "NOT SELECTED":
        return ""
    return owner


def _compute_overlay_owner_feature(image):
    if cv2 is None or np is None or image is None:
        return None
    try:
        if image.size == 0:
            return None
        h, w = image.shape[:2]
        if h < 24 or w < 12:
            return None
        x1 = int(w * 0.18)
        x2 = int(w * 0.82)
        y1 = int(h * 0.22)
        y2 = int(h * 0.92)
        crop = image[max(0, y1):max(y1 + 1, y2), max(0, x1):max(x1 + 1, x2)]
        if crop.size == 0:
            crop = image
        crop = cv2.resize(crop, (72, 96), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [24, 18], [0, 180, 0, 256])
        hist = cv2.normalize(hist, hist).flatten().astype("float32")
        mean = np.mean(hsv.reshape(-1, 3), axis=0).astype("float32")
        return {"hist": hist, "mean": mean}
    except Exception:
        return None


def _compare_overlay_owner_feature(a, b):
    if cv2 is None or np is None or not a or not b:
        return float("inf")
    try:
        hist_dist = float(cv2.compareHist(a["hist"], b["hist"], cv2.HISTCMP_BHATTACHARYYA))
        mean_dist = float(np.linalg.norm(a["mean"] - b["mean"]) / 255.0)
        return 0.78 * hist_dist + 0.22 * mean_dist
    except Exception:
        return float("inf")


def _load_overlay_owner_refs(owner):
    safe, folder = _dataset_owner_dir(owner)
    if not safe or not os.path.isdir(folder):
        return []
    files = []
    try:
        for name in sorted(os.listdir(folder)):
            if name.lower().endswith((".jpg", ".jpeg", ".png", ".webp")):
                path = os.path.join(folder, name)
                try:
                    files.append((path, os.path.getmtime(path), os.path.getsize(path)))
                except Exception:
                    pass
    except Exception:
        return []
    key = tuple(files)
    with _follow_overlay_profile_lock:
        if (
            _follow_overlay_profile_cache.get("owner") == safe
            and _follow_overlay_profile_cache.get("key") == key
        ):
            return list(_follow_overlay_profile_cache.get("refs") or [])
    refs = []
    for path, _, _ in files[:40]:
        try:
            img = cv2.imread(path) if cv2 is not None else None
            feat = _compute_overlay_owner_feature(img)
            if feat is not None:
                refs.append(feat)
        except Exception:
            pass
    with _follow_overlay_profile_lock:
        _follow_overlay_profile_cache["owner"] = safe
        _follow_overlay_profile_cache["key"] = key
        _follow_overlay_profile_cache["refs"] = list(refs)
    return refs


def _overlay_owner_score(frame, bbox, owner):
    if cv2 is None or np is None or not owner or bbox is None:
        return float("inf")
    refs = _load_overlay_owner_refs(owner)
    if not refs:
        return float("inf")
    try:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        pad_x = int((x2 - x1) * 0.18)
        pad_y = int((y2 - y1) * 0.10)
        x1 = max(0, x1 - pad_x)
        y1 = max(0, y1 - pad_y)
        x2 = min(w - 1, x2 + pad_x)
        y2 = min(h - 1, y2 + pad_y)
        feat = _compute_overlay_owner_feature(frame[y1:y2 + 1, x1:x2 + 1])
        if feat is None:
            return float("inf")
        return min(_compare_overlay_owner_feature(feat, ref) for ref in refs)
    except Exception:
        return float("inf")


def _estimate_preview_camera_distance(shoulder_px):
    try:
        sh = float(shoulder_px)
        if sh <= 1.0:
            return float("inf"), float("inf")
        raw_cam_m = (CAMERA_FOLLOW_DISTANCE_A / max(1.0, sh)) + CAMERA_FOLLOW_DISTANCE_B
        raw_cam_m = max(0.20, min(8.0, raw_cam_m))
        scale = max(0.10, min(3.0, CAMERA_FOLLOW_RANGE_SCALE))
        offset = max(-3.0, min(3.0, CAMERA_FOLLOW_RANGE_OFFSET_M))
        cam_m = max(0.05, scale * raw_cam_m + offset)
        lidar_m = max(0.05, scale * (raw_cam_m + CAMERA_FOLLOW_CAMERA_TO_LIDAR_M) + offset)
        return cam_m, lidar_m
    except Exception:
        return float("inf"), float("inf")


def _estimate_follow_overlay(frame):
    global _follow_pose
    if _follow_pose is None or cv2 is None:
        return None
    try:
        h, w = frame.shape[:2]
        work = frame
        scale = 1.0
        if CAMERA_FOLLOW_OVERLAY_WIDTH > 0 and w > CAMERA_FOLLOW_OVERLAY_WIDTH:
            scale = CAMERA_FOLLOW_OVERLAY_WIDTH / float(w)
            new_h = max(96, int(h * scale))
            work = cv2.resize(frame, (CAMERA_FOLLOW_OVERLAY_WIDTH, new_h), interpolation=cv2.INTER_AREA)
        wh, ww = work.shape[:2]
        rgb = cv2.cvtColor(work, cv2.COLOR_BGR2RGB)
        # MediaPipe Pose is not thread-safe. Web preview, snapshot endpoint, and
        # follow tracker may request overlay frames together; serialize access so
        # one bad timestamp cannot crash the Flask server.
        with _follow_pose_lock:
            if _follow_pose is None:
                return None
            result = _follow_pose.process(rgb)
        label = "NO TARGET"
        color = (70, 70, 255)
        center_x = w // 2
        deadzone = max(24, w // 22)
        overlay = {
            "label": label,
            "color": color,
            "bbox": None,
            "center": None,
            "shoulder_px": None,
            "camera_range_m": None,
            "camera_lidar_est_m": None,
            "draw_landmarks": None,
            "owner": _selected_follow_owner(),
            "owner_score": None,
            "shape": (h, w),
        }
        if result and result.pose_landmarks:
            lm = result.pose_landmarks.landmark
            l_sh = lm[11]
            r_sh = lm[12]
            l_hip = lm[23]
            r_hip = lm[24]
            nose = lm[0]
            visible_core = sum(
                1
                for p in (nose, l_sh, r_sh, l_hip, r_hip)
                if getattr(p, "visibility", 0.0) > 0.35
            )
            hips_ok = (
                l_hip.visibility > 0.28
                and r_hip.visibility > 0.28
                and ((l_hip.y + r_hip.y) * 0.5) > ((l_sh.y + r_sh.y) * 0.5)
            )
            if l_sh.visibility > 0.65 and r_sh.visibility > 0.65 and visible_core >= 4 and hips_ok:
                inv = 1.0 / max(scale, 1e-6)
                cx = int(((l_sh.x + r_sh.x) * 0.5) * ww * inv)
                shoulder_px = abs(l_sh.x - r_sh.x) * ww * inv
                if shoulder_px < max(38.0, CAMERA_FOLLOW_TARGET_SHOULDER_PX * 0.16):
                    overlay["label"] = "NO HUMAN"
                    overlay["color"] = (70, 70, 255)
                    return overlay
                cam_m, lidar_est_m = _estimate_preview_camera_distance(shoulder_px)
                x1 = max(0, int(min(l_sh.x, r_sh.x) * ww * inv) - int(40 * inv))
                x2 = min(w - 1, int(max(l_sh.x, r_sh.x) * ww * inv) + int(40 * inv))
                y = int(((l_sh.y + r_sh.y) * 0.5) * wh * inv)
                y1 = max(0, y - int(70 * inv))
                y2 = min(h - 1, y + int(90 * inv))
                candidate_bbox = (x1, y1, x2, y2)
                owner = overlay.get("owner") or ""
                if not owner:
                    label = "SELECT OWNER"
                    color = (0, 180, 255)
                else:
                    owner_score = _overlay_owner_score(frame, candidate_bbox, owner)
                    overlay["owner_score"] = owner_score if owner_score != float("inf") else None
                    if owner_score > CAMERA_FOLLOW_OWNER_MATCH_MAX:
                        label = (
                            f"NOT OWNER {owner_score:.2f}>{CAMERA_FOLLOW_OWNER_MATCH_MAX:.2f}"
                            if owner_score != float("inf") else "NO OWNER DATA"
                        )
                        color = (70, 70, 255)
                        overlay["bbox"] = candidate_bbox
                        overlay["center"] = (cx, y)
                        overlay["shoulder_px"] = shoulder_px
                        overlay["camera_range_m"] = cam_m
                        overlay["camera_lidar_est_m"] = lidar_est_m
                        overlay["owner_match"] = False
                    else:
                        overlay["bbox"] = candidate_bbox
                        overlay["center"] = (cx, y)
                        overlay["shoulder_px"] = shoulder_px
                        overlay["camera_range_m"] = cam_m
                        overlay["camera_lidar_est_m"] = lidar_est_m
                        offset_norm = max(-1.0, min(1.0, (cx - center_x) / max(1.0, center_x)))
                        overlay["offset_norm"] = offset_norm
                        offset_pct = int(round(abs(offset_norm) * 100))
                        if owner_score > CAMERA_FOLLOW_OWNER_STRONG_MAX:
                            overlay["owner_match"] = "locking"
                            label = f"LOCK OWNER {owner_score:.2f}"
                            color = (0, 180, 255)
                        elif cx < (center_x - deadzone):
                            overlay["owner_match"] = True
                            label = f"STEER LEFT {offset_pct}%"
                            color = (0, 180, 255)
                        elif cx > (center_x + deadzone):
                            overlay["owner_match"] = True
                            label = f"STEER RIGHT {offset_pct}%"
                            color = (0, 180, 255)
                        elif shoulder_px < 210:
                            overlay["owner_match"] = True
                            label = f"FORWARD offset {offset_pct}%"
                            color = (0, 255, 0)
                        else:
                            overlay["owner_match"] = True
                            label = "HOLD"
                            color = (0, 255, 255)
                        if CAMERA_FOLLOW_DRAW_LANDMARKS and mp_drawing is not None and mp_pose is not None:
                            overlay["draw_landmarks"] = result.pose_landmarks
        overlay["label"] = label
        overlay["color"] = color
        return overlay
    except Exception as e:
        msg = str(e)
        if "Packet timestamp mismatch" in msg or "CalculatorGraph" in msg:
            try:
                with _follow_pose_lock:
                    try:
                        if _follow_pose is not None:
                            _follow_pose.close()
                    except Exception:
                        pass
                    _follow_pose = _create_follow_pose()
            except Exception:
                _follow_pose = None
        return None


def _draw_follow_overlay(frame, overlay):
    if frame is None or overlay is None or cv2 is None:
        return frame
    try:
        h, w = frame.shape[:2]
        center_x = w // 2
        deadzone = max(24, w // 22)
        cv2.line(frame, (center_x - deadzone, 0), (center_x - deadzone, h), (90, 90, 90), 1)
        cv2.line(frame, (center_x + deadzone, 0), (center_x + deadzone, h), (90, 90, 90), 1)
        bbox = overlay.get("bbox")
        center = overlay.get("center")
        shoulder_px = overlay.get("shoulder_px")
        if bbox is not None and center is not None:
            x1, y1, x2, y2 = bbox
            cx, cy = center
            offset_norm = overlay.get("offset_norm")
            offset_txt = ""
            if isinstance(offset_norm, (int, float)):
                offset_txt = f" off={float(offset_norm):+.2f}"
            owner_score = overlay.get("owner_score")
            score_txt = ""
            if isinstance(owner_score, (int, float)):
                score_txt = f" owner={float(owner_score):.2f}"
            cam_m = overlay.get("camera_range_m")
            lidar_est_m = overlay.get("camera_lidar_est_m")
            dist_txt = ""
            if isinstance(cam_m, (int, float)) and math.isfinite(cam_m):
                dist_txt = f" cam~{float(cam_m):.2f}m"
                if isinstance(lidar_est_m, (int, float)) and math.isfinite(lidar_est_m):
                    dist_txt += f" lidar_est~{float(lidar_est_m):.2f}m"
            owner_match = overlay.get("owner_match")
            if owner_match is True:
                box_color = (0, 255, 0)
            elif owner_match == "locking":
                box_color = (0, 180, 255)
            elif owner_match is False:
                box_color = (0, 0, 255)
            else:
                box_color = overlay.get("color", (0, 180, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 3)
            cv2.circle(frame, (cx, cy), 5, box_color, -1)
            cv2.putText(
                frame,
                f"TARGET cx={cx} sh={float(shoulder_px or 0):.0f}px{offset_txt}{score_txt}{dist_txt}",
                (14, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            frame,
            f"FOLLOW AI: {overlay.get('label', 'NO TARGET')}",
            (14, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            overlay.get("color", (70, 70, 255)),
            2,
            cv2.LINE_AA,
        )
    except Exception:
        return frame
    return frame


def _follow_overlay_status_payload(overlay):
    now = time.time()
    if not overlay:
        return {"ts": now, "active": False}
    cam_m = overlay.get("camera_range_m")
    lidar_est_m = overlay.get("camera_lidar_est_m")
    shoulder_px = overlay.get("shoulder_px")
    owner_score = overlay.get("owner_score")
    active = overlay.get("bbox") is not None and isinstance(cam_m, (int, float)) and math.isfinite(cam_m)
    return {
        "ts": now,
        "active": bool(active),
        "label": overlay.get("label") or "NO TARGET",
        "owner": overlay.get("owner") or "",
        "owner_match": overlay.get("owner_match"),
        "owner_score": None if not isinstance(owner_score, (int, float)) or not math.isfinite(owner_score) else round(float(owner_score), 3),
        "camera_range_m": None if not isinstance(cam_m, (int, float)) or not math.isfinite(cam_m) else round(float(cam_m), 2),
        "camera_lidar_est_m": None if not isinstance(lidar_est_m, (int, float)) or not math.isfinite(lidar_est_m) else round(float(lidar_est_m), 2),
        "camera_shoulder_px": None if not isinstance(shoulder_px, (int, float)) or not math.isfinite(shoulder_px) else round(float(shoulder_px), 0),
    }


def _apply_follow_overlay(frame):
    global _follow_overlay_status
    if frame is None:
        return frame
    now = time.time()
    min_dt = 1.0 / max(1.0, CAMERA_FOLLOW_OVERLAY_MAX_HZ)
    with _follow_overlay_lock:
        cached = _follow_overlay_state.get("data")
        cached_ts = _follow_overlay_state.get("ts", 0.0)
    overlay = None
    if cached is not None and (now - cached_ts) <= min_dt:
        overlay = cached
    else:
        overlay = _estimate_follow_overlay(frame)
        # Do not reuse an old person bbox when the person has left the frame.
        # A stale yellow box is worse than no box because it suggests tracking is active.
        with _follow_overlay_lock:
            _follow_overlay_state["data"] = overlay
            _follow_overlay_state["ts"] = now
            _follow_overlay_status = _follow_overlay_status_payload(overlay)
    return _draw_follow_overlay(frame, overlay)


def _iter_usb_mjpeg(stabilize=False, token=None, follow_ai=False, fast=False):
    global _usb_stream_active
    _ensure_usb_worker()
    try:
        with _stream_guard_lock:
            _usb_stream_active += 1
        jpeg_q = CAMERA_FAST_QUALITY if fast else CAMERA_QUALITY
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), max(28, min(92, jpeg_q))]
        stab_state = {"prev_gray": None, "lp_dx": 0.0, "lp_dy": 0.0, "lp_da": 0.0}
        last_emit = 0.0
        while True:
            frame = _get_latest_usb_frame(max_age_s=0.7)
            if frame is None:
                time.sleep(0.03)
                continue
            if fast and CAMERA_FAST_WIDTH > 0:
                h, w = frame.shape[:2]
                if w > CAMERA_FAST_WIDTH:
                    scale = CAMERA_FAST_WIDTH / float(w)
                    new_h = max(120, int(h * scale))
                    frame = cv2.resize(frame, (CAMERA_FAST_WIDTH, new_h), interpolation=cv2.INTER_AREA)
            if stabilize:
                frame = _apply_soft_stabilization(frame, stab_state)
            if follow_ai:
                try:
                    frame = _auto_contrast_frame(frame)
                    frame = _apply_follow_overlay(frame)
                except Exception:
                    pass
            ok, jpeg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue
            data = jpeg.tobytes()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n" +
                data + b"\r\n"
            )
            now = time.time()
            dt = now - last_emit
            target_hz = CAMERA_FAST_FPS if fast else CAMERA_FPS
            target_dt = 1.0 / max(4, int(target_hz))
            if dt < target_dt:
                time.sleep(target_dt - dt)
            last_emit = time.time()
    finally:
        with _stream_guard_lock:
            _usb_stream_active = max(0, _usb_stream_active - 1)

# ================== WS PROXY ==================

@sock.route("/ros")
def ros_proxy(ws):
    try:
        ros = websocket.create_connection(ROSBRIDGE_URL)
        print("✅ Connected to ROSBridge")
    except Exception as e:
        print("❌ ROSBridge connect failed:", e)
        return

    def ros_to_web():
        while True:
            try:
                ws.send(ros.recv())
            except:
                break

    threading.Thread(target=ros_to_web, daemon=True).start()


    while True:
        try:
            msg = ws.receive()
            if msg is None:
                break
            ros.send(msg)
        except:
            break

    ros.close()
    ws.close()

# ================== RUN ==================

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)
