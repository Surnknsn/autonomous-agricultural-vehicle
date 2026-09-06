from flask import Flask, render_template, request, jsonify, send_from_directory, abort, Response, stream_with_context
from flask_sock import Sock
import websocket
import threading
import os
import time
import json
import requests
import shutil
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
DATASET_DIR = os.path.join(BASE_DIR, "dataset")
SHARED_MAP_STATE_PATH = os.path.join(BASE_DIR, "shared_map_state.json")
SHARED_UI_STATE_PATH = os.path.join(BASE_DIR, "shared_ui_state.json")
CAMERA_MJPEG_URL = os.getenv("CAMERA_MJPEG_URL", "http://127.0.0.1:8081/video_feed")
CAMERA_MODE = os.getenv("CAMERA_MODE", "auto").lower()  # auto | mjpeg | usb
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "/dev/video0")
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "360"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "20"))
CAMERA_QUALITY = int(os.getenv("CAMERA_QUALITY", "75"))
CAMERA_STABILIZE_DEFAULT = os.getenv("CAMERA_STABILIZE_DEFAULT", "0").lower() in ("1", "true", "yes", "on")
CAMERA_FAST_QUALITY = int(os.getenv("CAMERA_FAST_QUALITY", "52"))
CAMERA_FAST_WIDTH = int(os.getenv("CAMERA_FAST_WIDTH", "512"))
CAMERA_FOLLOW_OVERLAY_WIDTH = int(os.getenv("CAMERA_FOLLOW_OVERLAY_WIDTH", "320"))
CAMERA_FOLLOW_OWNER_MATCH_MAX = float(os.getenv("CAMERA_FOLLOW_OWNER_MATCH_MAX", "0.50"))
CAMERA_FOLLOW_AUTO_CONTRAST = os.getenv("CAMERA_FOLLOW_AUTO_CONTRAST", "1").lower() in ("1", "true", "yes", "on")
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
CAMERA_FOLLOW_OVERLAY_MAX_HZ = float(os.getenv("CAMERA_FOLLOW_OVERLAY_MAX_HZ", "4.0"))
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


def _default_map_state():
    return {
        "map_mode": "GRID",
        "boundary_points": [],
        "grid_waypoints": [],
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
            state["updated_at"] = float(data.get("updated_at") or 0.0)
        return state
    except Exception:
        return _default_map_state()


def _save_map_state(state):
    payload = _default_map_state()
    payload["map_mode"] = str(state.get("map_mode") or "GRID").upper()
    payload["boundary_points"] = state.get("boundary_points") or []
    payload["grid_waypoints"] = state.get("grid_waypoints") or []
    payload["updated_at"] = time.time()
    tmp_path = SHARED_MAP_STATE_PATH + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False)
    os.replace(tmp_path, SHARED_MAP_STATE_PATH)
    return payload


def _default_ui_state():
    return {
        "follow_target": "NOT SELECTED",
        "safety_enabled": True,
        "gps_start_only": False,
        "auto_speed_pct": 100,
        "safety_cm": 100,
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
            state["safety_cm"] = max(0, min(400, int(round(float(data.get("safety_cm", 100))))))
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
    payload["safety_cm"] = max(0, min(400, int(round(float(state.get("safety_cm", 100))))))
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

# ================== HTTP ==================

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/status")
def status():
    return jsonify({
        "web": "ONLINE",
        "ros": "UNKNOWN"
    })

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

    return Response(
        stream_with_context(generate()),
        content_type="multipart/x-mixed-replace; boundary=frame"
    )


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
        frame = _get_latest_usb_frame(max_age_s=2.0)
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
                for _ in range(2):
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
            "draw_landmarks": None,
            "owner": _selected_follow_owner(),
            "owner_score": None,
            "shape": (h, w),
        }
        if result and result.pose_landmarks:
            lm = result.pose_landmarks.landmark
            l_sh = lm[11]
            r_sh = lm[12]
            if l_sh.visibility > 0.55 and r_sh.visibility > 0.55:
                inv = 1.0 / max(scale, 1e-6)
                cx = int(((l_sh.x + r_sh.x) * 0.5) * ww * inv)
                shoulder_px = abs(l_sh.x - r_sh.x) * ww * inv
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
                        label = f"NO OWNER MATCH {owner_score:.2f}" if owner_score != float("inf") else "NO OWNER DATA"
                        color = (70, 70, 255)
                        overlay["bbox"] = candidate_bbox
                        overlay["center"] = (cx, y)
                        overlay["shoulder_px"] = shoulder_px
                        overlay["owner_match"] = False
                    else:
                        overlay["bbox"] = candidate_bbox
                        overlay["center"] = (cx, y)
                        overlay["shoulder_px"] = shoulder_px
                        overlay["owner_match"] = True
                        offset_norm = max(-1.0, min(1.0, (cx - center_x) / max(1.0, center_x)))
                        overlay["offset_norm"] = offset_norm
                        offset_pct = int(round(abs(offset_norm) * 100))
                        if cx < (center_x - deadzone):
                            label = f"STEER LEFT {offset_pct}%"
                            color = (0, 180, 255)
                        elif cx > (center_x + deadzone):
                            label = f"STEER RIGHT {offset_pct}%"
                            color = (0, 180, 255)
                        elif shoulder_px < 210:
                            label = f"FORWARD offset {offset_pct}%"
                            color = (0, 255, 0)
                        else:
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
            owner_match = overlay.get("owner_match")
            if owner_match is True:
                box_color = (0, 255, 0)
            elif owner_match is False:
                box_color = (0, 0, 255)
            else:
                box_color = overlay.get("color", (0, 180, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 3)
            cv2.circle(frame, (cx, cy), 5, box_color, -1)
            cv2.putText(
                frame,
                f"TARGET cx={cx} sh={float(shoulder_px or 0):.0f}px{offset_txt}{score_txt}",
                (14, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
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


def _apply_follow_overlay(frame):
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
            frame = _get_latest_usb_frame(max_age_s=2.0)
            if frame is None:
                time.sleep(0.03)
                continue
            if stabilize:
                frame = _apply_soft_stabilization(frame, stab_state)
            if follow_ai:
                try:
                    frame = _auto_contrast_frame(frame)
                    frame = _apply_follow_overlay(frame)
                except Exception:
                    pass
            if fast and CAMERA_FAST_WIDTH > 0:
                h, w = frame.shape[:2]
                if w > CAMERA_FAST_WIDTH:
                    scale = CAMERA_FAST_WIDTH / float(w)
                    new_h = max(120, int(h * scale))
                    frame = cv2.resize(frame, (CAMERA_FAST_WIDTH, new_h), interpolation=cv2.INTER_AREA)
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
            target_dt = 1.0 / max(6, CAMERA_FPS)
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
