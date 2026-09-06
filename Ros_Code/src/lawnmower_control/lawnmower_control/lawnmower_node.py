#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import math
import glob
import os
from collections import deque
import statistics
import re
import json
import threading
import fcntl

from pymavlink import mavutil
from std_msgs.msg import Float32MultiArray, String, Float32, Bool
from sensor_msgs.msg import LaserScan

# =========================
# HW MODE DEFINITION (FIXED)
# =========================
# 0 = MANUAL
# 1 = AUTO
# 2 = FOLLOW
# 3 = EMERGENCY (virtual, Jetson only)

HW_MANUAL = 0
HW_AUTO   = 1
HW_FOLLOW = 2
HW_EMER   = 3

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


# =========================
# PID CONTROLLER
# =========================
class PID:
    def __init__(self, kp, ki, kd, limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i = 0.0
        self.prev = 0.0

    def reset(self):
        self.i = 0.0
        self.prev = 0.0

    def step(self, err, dt):
        self.i += err * dt
        d = (err - self.prev) / dt if dt > 1e-6 else 0.0
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        if self.limit is not None:
            out = max(-self.limit, min(self.limit, out))
        return out


class LawnmowerNode(Node):
    def __init__(self):
        super().__init__('lawnmower_node')
        # ---------- PID (AUTO DRIVE) ----------
        # Keep AUTO steering calm. Large yaw gain made the mower "snake" around
        # the path after small GPS/yaw corrections.
        self.pid_yaw = PID(kp=11.0, ki=0.0, kd=7.0, limit=18.0)
        self.pid_speed = PID(kp=28.0, ki=0.0, kd=3.0, limit=60.0)
        self._last_ctrl_time = time.time()


        # ---------- CONFIG (ห้ามแก้) ----------
        # Encoder calibration:
        # measured wheel pulses per 1 full wheel revolution is ~1100-1200.
        # use effective wheel pulses directly to avoid gearbox/count-mode mismatch.
        self.WHEEL_PULSES_PER_REV = 1150.0
        self.PULSE_PER_REV = 600.0
        self.GEAR_RATIO = 3.0
        self.WHEEL_DIAMETER_M = 0.30
        self.WHEEL_CIRCUM_M = math.pi * self.WHEEL_DIAMETER_M

        self.ENCODER_PORT_CANDIDATES = [
            "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24336303633351411171-if00",
        ]
        # Hard lock to the mower Arduino by-id device. Do not use /dev/ttyACM*
        # aliases here; ACM numbers move when Pixhawk/Arduino are replugged.
        arduino_port_override = os.getenv("ARDUINO_PORT", "").strip()
        self.arduino_port_override = arduino_port_override
        if arduino_port_override:
            self.ENCODER_PORT_CANDIDATES = [arduino_port_override]
        self.arduino_by_id_lock = os.getenv("ARDUINO_BY_ID_LOCK", "1").lower() in ("1", "true", "yes", "on")
        self.PIXHAWK_PORT_CANDIDATES = [
            "/dev/serial/by-id/usb-Holybro_Pixhawk6C_1D0027000151333238373537-if00"
        ]
        pixhawk_port_override = os.getenv("PIXHAWK_PORT", "").strip()
        if pixhawk_port_override:
            self.PIXHAWK_PORT_CANDIDATES = [pixhawk_port_override] + self.PIXHAWK_PORT_CANDIDATES
        self.PIXHAWK_PORT = self.PIXHAWK_PORT_CANDIDATES[0]

        # ---------- STATE ----------
        self.yaw = 0.0
        self.yaw_ui = 0.0
        # Fixed body-frame correction for Pixhawk mounting relative to mower chassis.
        self.body_yaw_offset_deg = float(os.getenv("BODY_YAW_OFFSET_DEG", "0.0"))
        self.body_yaw_offset_rad = math.radians(self.body_yaw_offset_deg)
        # Disable dynamic yaw compensation: trust Pixhawk yaw directly after body-frame correction.
        self.yaw_offset_deg = 0.0
        self.yaw_offset_rad = 0.0
        self._yaw_ui_prev = None
        self._imu_yaw_initialized = False
        self.last_imu_yaw_rate = 0.0
        self.prev_enc = None
        self._prev_enc_l = None
        self._prev_enc_r = None
        self.last_enc_delta = 0.0
        self.last_enc_delta_l = 0.0
        self.last_enc_delta_r = 0.0
        self.enc_l_raw = 0
        self.enc_r_raw = 0
        self.arduino_pump_on = False
        self.arduino_blade_state = 0
        self.arduino_manual_led = False
        self.arduino_auto_lamp = False
        self.arduino_emg_flag = False
        self.arduino_can_soc = -1.0
        self.arduino_can_voltage = float("nan")
        self.arduino_can_current = float("nan")
        self.arduino_can_temperature = float("nan")
        self.arduino_can_status = 2
        self.arduino_can_last_valid_ts = 0.0
        self.arduino_can_invalid_hold_s = float(os.getenv("CAN_INVALID_HOLD_S", "30.0"))
        self.arduino_can_soc_max_step = float(os.getenv("CAN_SOC_MAX_STEP", "8.0"))
        self.arduino_speed_l_mps = 0.0
        self.arduino_speed_r_mps = 0.0
        self.arduino_speed_avg_mps = 0.0
        self.arduino_pwm_l = 0
        self.arduino_pwm_r = 0
        self.arduino_pwm_valid = False
        self._last_ctrl_time = time.time()
        self._kick_pwm = 0.0
        self.KICK_RAMP = 18.0
        self.KICK_MAX = 35.0
        # Motor/encoder has about 30 pulses of mechanical free play.
        # Do not let that backlash look like real movement or slip.
        self.ENC_FREE_GAP_PULSES = 30.0
        self.ENC_THRESHOLD = self.ENC_FREE_GAP_PULSES
        self.ENC_MOVING_PULSE_DEADBAND = self.ENC_FREE_GAP_PULSES
        self.ENC_DRIFT_PULSE_DEADBAND = 8.0
        self.TARGET_SPEED_MPS = 0.24
        # Real garden soil needs more breakaway torque than flat concrete.
        # Keep AUTO slow by limiting speed/ramps, not by commanding PWM that
        # cannot move the chassis and only digs the wheels into soft ground.
        self.MIN_PWM = 95.0
        # MIN is the inner-wheel floor during arc differential steering.
        # MAX is the cruise speed target and DRIVE_MAX_PWM base.
        # Keeping MIN < MAX gives _shape_drive_pwm room to produce differential
        # commands so the robot can arc-correct while still cruising at ~100.
        self.STRAIGHT_PWM_MIN = float(os.getenv("AUTO_STRAIGHT_PWM_MIN", "70"))
        self.STRAIGHT_PWM_MAX = float(os.getenv("AUTO_STRAIGHT_PWM_MAX", "82"))
        self.DRIVE_MAX_PWM = int(self.STRAIGHT_PWM_MAX)
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self.DRIVE_STALL_BOOST_RAMP = float(os.getenv("DRIVE_STALL_BOOST_RAMP", "85.0"))
        self.DRIVE_STALL_BOOST_DECAY = float(os.getenv("DRIVE_STALL_BOOST_DECAY", "160.0"))
        self.DRIVE_STALL_BOOST_MAX = float(os.getenv("DRIVE_STALL_BOOST_MAX", "0.0"))
        self.DRIVE_STALL_BOOST_DELAY_S = float(os.getenv("DRIVE_STALL_BOOST_DELAY_S", "0.55"))
        self.DRIVE_STALL_BOOST_CAP_PWM = float(os.getenv("DRIVE_STALL_BOOST_CAP_PWM", "128.0"))
        self.TURN_MAX_PWM = int(float(os.getenv("AUTO_TURN_MAX_PWM", "235")))
        self.AUTO_PWM_ABS_CAP = int(float(os.getenv("AUTO_PWM_ABS_CAP", "235")))
        self.FOLLOW_PWM_ABS_CAP = int(float(os.getenv("FOLLOW_PWM_ABS_CAP", "150")))
        self.PWM_DRIVE_UP_PER_S = float(os.getenv("PWM_DRIVE_UP_PER_S", "150.0"))
        self.PWM_DRIVE_DOWN_PER_S = float(os.getenv("PWM_DRIVE_DOWN_PER_S", "220.0"))
        # Pivot turns need to overcome sand/static friction quickly; slow ramp
        # digs a groove before the body rotates. Down-ramp stays controlled so
        # the robot does not snap back and overshoot heading.
        self.PWM_TURN_UP_PER_S = float(os.getenv("PWM_TURN_UP_PER_S", "420.0"))
        self.PWM_TURN_DOWN_PER_S = float(os.getenv("PWM_TURN_DOWN_PER_S", "420.0"))
        self.PWM_ZERO_CROSS_PER_S = 420.0
        self._pwm_cmd_l = 0.0
        self._pwm_cmd_r = 0.0
        self._last_pwm_send_ts = time.time()
        self._startup_ts = time.time()

        self.mode_hardware = HW_MANUAL   # จาก Arduino
        self._mode_candidate = self.mode_hardware
        self._mode_candidate_count = 0
        # Mode switch noise should not throw AUTO/FOLLOW into MANUAL. Active
        # modes can engage quickly, but MANUAL must stay stable longer.
        self.MODE_DEBOUNCE_COUNT_ACTIVE = int(os.getenv("MODE_DEBOUNCE_COUNT_ACTIVE", "2"))
        self.MODE_DEBOUNCE_COUNT_MANUAL = int(os.getenv("MODE_DEBOUNCE_COUNT_MANUAL", "8"))
        self.MODE_DEBOUNCE_TIME_ACTIVE_S = float(os.getenv("MODE_DEBOUNCE_TIME_ACTIVE_S", "0.12"))
        self.MODE_DEBOUNCE_TIME_MANUAL_S = float(os.getenv("MODE_DEBOUNCE_TIME_MANUAL_S", "0.80"))
        self.MODE_MANUAL_IO_CONFIRM_ENABLE = os.getenv("MODE_MANUAL_IO_CONFIRM_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self._last_arduino_rx_ts = 0.0
        self._last_mode_rx_ts = 0.0
        self._last_serial_err_log_ts = 0.0
        self._last_serial_write_timeout_ts = 0.0
        self._arduino_write_timeout_count = 0
        self._last_arduino_usb_reset_ts = 0.0
        self._last_serial_parse_warn_ts = 0.0
        self._last_mode_parse_debug_ts = 0.0
        self._mode_candidate_since = time.time()
        self._last_arduino_verbose_log_ts = 0.0
        self._warned_fw_old_log_format = False
        self._arduino_rx_seen = False
        self._serial_rx_buf = ""
        self.ARDUINO_ACCEPT_OLD_3_FIELD = os.getenv("ARDUINO_ACCEPT_OLD_3_FIELD", "0").lower() in ("1", "true", "yes", "on")
        self.arduino_connected = False
        # Serial can pause briefly while USB/MCU handles mode-switch noise.
        # Keep timeout relaxed to avoid false reconnect storms.
        self.arduino_rx_timeout_s = float(os.getenv("ARDUINO_RX_TIMEOUT_S", "12.0"))
        self._last_arduino_retry_ts = 0.0
        self._last_arduino_state_warn_ts = 0.0
        self._last_start_blocked_warn_ts = 0.0
        self.mode_web = "STOP"           # จาก Web
        self.web_run_mode = "AUTO"       # AUTO / FOLLOW; START alone is treated as AUTO-safe.
        self.AUTO_MANUAL_ASSIST_ENABLE = os.getenv("AUTO_MANUAL_ASSIST_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.auto_manual_assist_enabled = False
        self.auto_manual_assist_reach_m = float(os.getenv("AUTO_MANUAL_ASSIST_REACH_M", "0.85"))
        self.auto_manual_assist_hold_s = float(os.getenv("AUTO_MANUAL_ASSIST_HOLD_S", "0.35"))
        self.auto_manual_assist_drive_until = 0.0
        self.emergency_latched = False
        # Slip-tolerant waypoint tracking for real ground conditions.
        self.WAYPOINT_REACH_M = float(os.getenv("WP_REACH_M", "0.95"))
        self.WAYPOINT_TURN_REACH_M = float(os.getenv("WP_TURN_REACH_M", "0.85"))
        self.WAYPOINT_SHORT_SEGMENT_REACH_RATIO = float(os.getenv("WP_SHORT_SEGMENT_REACH_RATIO", "0.45"))
        self.WAYPOINT_SHORT_SEGMENT_MIN_REACH_M = float(os.getenv("WP_SHORT_SEGMENT_MIN_REACH_M", "0.30"))
        self.WAYPOINT_TURN_ANGLE_RAD = math.radians(float(os.getenv("WP_TURN_ANGLE_DEG", "55.0")))
        self.SEGMENT_LOOKAHEAD_M = float(os.getenv("SEGMENT_LOOKAHEAD_M", "2.35"))
        self.SEGMENT_LOOKAHEAD_MIN_M = float(os.getenv("SEGMENT_LOOKAHEAD_MIN_M", "1.35"))
        # Simple waypoint mode: let GPS/IMU drive the drawn points directly.
        # Old training-derived offsets are kept as env hooks, but default to a
        # conservative slow controller so field tests are predictable.
        self.CROSSTRACK_TIGHTEN_M = float(os.getenv("CROSSTRACK_TIGHTEN_M", "1.55"))
        self.CROSSTRACK_LOOKAHEAD_M = float(os.getenv("CROSSTRACK_LOOKAHEAD_M", "1.90"))
        self.GPS_CROSSTRACK_DEADBAND_M = float(os.getenv("GPS_CROSSTRACK_DEADBAND_M", "0.35"))
        self.GPS_CROSSTRACK_MAX_CORR_M = float(os.getenv("GPS_CROSSTRACK_MAX_CORR_M", "0.85"))
        self.WAYPOINT_REACH_MAX_CROSSTRACK_M = float(os.getenv("WP_REACH_MAX_CROSSTRACK_M", "0.70"))
        self.IMU_SEGMENT_HEADING_ENABLE = os.getenv("AUTO_IMU_SEGMENT_HEADING", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_SEGMENT_HEADING_LOCK = os.getenv("AUTO_SEGMENT_HEADING_LOCK", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_GPS_PATH_HEADING_ENABLE = os.getenv("AUTO_GPS_PATH_HEADING_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_DR_REACH_ENABLE = os.getenv("AUTO_DR_REACH_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_GPS_PASS_ENABLE = os.getenv("AUTO_GPS_PASS_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_STRICT_WAYPOINT_REACH = os.getenv("AUTO_STRICT_WAYPOINT_REACH", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_DR_ORDERED_HANDOFF_ENABLE = os.getenv("AUTO_DR_ORDERED_HANDOFF_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_DR_PASS_MARGIN_M = float(os.getenv("AUTO_DR_PASS_MARGIN_M", "0.20"))
        self.AUTO_DR_FINAL_PASS_MARGIN_M = float(os.getenv("AUTO_DR_FINAL_PASS_MARGIN_M", "0.90"))
        self.AUTO_DR_FINAL_MAX_CROSSTRACK_M = float(os.getenv("AUTO_DR_FINAL_MAX_CROSSTRACK_M", "1.50"))
        self.AUTO_DR_PASS_REQUIRE_GPS_CORRIDOR = os.getenv("AUTO_DR_PASS_REQUIRE_GPS_CORRIDOR", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_GPS_CORRIDOR_CONFIRM_M = float(os.getenv("AUTO_GPS_CORRIDOR_CONFIRM_M", "1.80"))
        self.AUTO_GPS_PASS_WINDOW_M = float(os.getenv("AUTO_GPS_PASS_WINDOW_M", "1.25"))
        self.IMU_SEGMENT_MIN_REMAIN_M = float(os.getenv("AUTO_IMU_SEGMENT_MIN_REMAIN_M", "1.20"))
        self.IMU_SEGMENT_MAX_CROSSTRACK_M = float(os.getenv("AUTO_IMU_SEGMENT_MAX_CROSSTRACK_M", "2.50"))
        # Breadcrumb sub-waypoints keep the mower from drifting across a long
        # segment. User waypoints remain marked as corners for stop/confirm.
        self.PATH_DENSIFY_ENABLE = os.getenv("PATH_DENSIFY_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.PATH_DENSIFY_SPACING_M = float(os.getenv("PATH_DENSIFY_SPACING_M", "2.50"))
        self.PATH_DENSE_REACH_M = float(os.getenv("PATH_DENSE_REACH_M", "0.80"))
        self.PATH_DENSIFY_MIN_SEG_M = float(os.getenv("PATH_DENSIFY_MIN_SEG_M", "0.08"))
        self.PATH_MIN_USER_POINT_SPACING_M = float(os.getenv("PATH_MIN_USER_POINT_SPACING_M", "0.75"))
        self.PATH_DUPLICATE_POINT_EPS_M = float(os.getenv("PATH_DUPLICATE_POINT_EPS_M", "0.15"))
        # Keep AUTO close to the drawn path using IMU/yaw feedback.
        # This is path correction, not left/right motor-side compensation.
        self.HEADING_DEADBAND_RAD = math.radians(float(os.getenv("AUTO_HEADING_DEADBAND_DEG", "10.0")))
        self.AUTO_LINE_STRAIGHT_HOLD_RAD = math.radians(float(os.getenv("AUTO_LINE_STRAIGHT_HOLD_DEG", "24.0")))
        self.MIN_FORWARD_PWM = self.STRAIGHT_PWM_MIN
        self.TURN_RATIO_MAX = float(os.getenv("AUTO_TURN_RATIO_MAX", "0.30"))
        self.STRAIGHT_HOLD_MIN_STEER_PWM = float(os.getenv("AUTO_STRAIGHT_HOLD_MIN_STEER_PWM", "0.0"))
        self.AUTO_STEER_MIX_SLEW_PWM_PER_S = float(os.getenv("AUTO_STEER_MIX_SLEW_PWM_PER_S", "10.0"))
        self._auto_steer_mix_prev = 0.0
        self.MIN_TURN_OUTER_PWM = float(os.getenv("AUTO_MIN_TURN_OUTER_PWM", "120.0"))
        self.MIN_TURN_INNER_PWM = float(os.getenv("AUTO_MIN_TURN_INNER_PWM", "90.0"))
        self.MIN_TURN_DIFF_PWM = 42.0
        self.TURN_ACTIVE_YAW_RAD = math.radians(38.0)
        self.PIVOT_ALLOW_YAW_RAD = math.radians(float(os.getenv("AUTO_PIVOT_ALLOW_YAW_DEG", "170.0")))
        self.PWM_NOISE_FLOOR = 18
        self.TURN_STALL_BOOST_PWM = 16
        self.TURN_STALL_ENC_THRESH = self.ENC_FREE_GAP_PULSES
        # If heading error is large, rotate in place first before moving forward.
        self.TURN_IN_PLACE_ENTER_RAD = math.radians(float(os.getenv("AUTO_TURN_IN_PLACE_ENTER_DEG", "170.0")))
        self.TURN_IN_PLACE_EXIT_RAD = math.radians(float(os.getenv("AUTO_TURN_IN_PLACE_EXIT_DEG", "16.0")))
        self.TURN_IN_PLACE_PWM = float(os.getenv("AUTO_TURN_IN_PLACE_PWM", "155.0"))
        self.TURN_IN_PLACE_MAX_PWM = float(os.getenv("AUTO_TURN_IN_PLACE_MAX_PWM", "235.0"))
        self.AUTO_HARD_ALIGN_ENABLE = os.getenv("AUTO_HARD_ALIGN_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_HARD_ALIGN_YAW_RAD = math.radians(float(os.getenv("AUTO_HARD_ALIGN_YAW_DEG", "24.0")))
        self.AUTO_HARD_ALIGN_MIN_DIST_M = float(os.getenv("AUTO_HARD_ALIGN_MIN_DIST_M", "0.75"))
        self.AUTO_HARD_ALIGN_PIVOT_PWM = float(os.getenv("AUTO_HARD_ALIGN_PIVOT_PWM", "155.0"))
        self.AUTO_HARD_ALIGN_MAX_PWM = float(os.getenv("AUTO_HARD_ALIGN_MAX_PWM", "235.0"))
        self.TURN_BREAKAWAY_PWM = float(os.getenv("AUTO_TURN_BREAKAWAY_PWM", "215.0"))
        self.WP_ALIGN_TURN_PWM = float(os.getenv("AUTO_WP_ALIGN_TURN_PWM", "155.0"))
        self.TURN_BRAKE_MIN_PWM = float(os.getenv("AUTO_TURN_BRAKE_MIN_PWM", "85.0"))
        self.TURN_PROFILE_MIN_PWM = float(os.getenv("AUTO_TURN_PROFILE_MIN_PWM", "150.0"))
        self.TURN_PROFILE_PEAK_PWM = float(os.getenv("AUTO_TURN_PROFILE_PEAK_PWM", "215.0"))
        self.TURN_PROFILE_PEAK_ERR_DEG = float(os.getenv("AUTO_TURN_PROFILE_PEAK_ERR_DEG", "70.0"))
        self.TURN_PROFILE_SIGMA_DEG = float(os.getenv("AUTO_TURN_PROFILE_SIGMA_DEG", "42.0"))
        self.AUTO_CORNER_SLOWDOWN_DIST_M = float(os.getenv("AUTO_CORNER_SLOWDOWN_DIST_M", "4.0"))
        self.AUTO_CORNER_APPROACH_PWM = float(os.getenv("AUTO_CORNER_APPROACH_PWM", "72.0"))
        self.TURN_BREAKAWAY_MIN_ERR_RAD = math.radians(float(os.getenv("AUTO_TURN_BREAKAWAY_MIN_ERR_DEG", "20.0")))
        self.TURN_BRAKE_ERR_RAD = math.radians(float(os.getenv("AUTO_TURN_BRAKE_ERR_DEG", "28.0")))
        self.TURN_BRAKE_YAW_RATE_RAD_S = math.radians(float(os.getenv("AUTO_TURN_BRAKE_YAW_RATE_DEG_S", "16.0")))
        self.TURN_BRAKE_SCALE = float(os.getenv("AUTO_TURN_BRAKE_SCALE", "0.55"))
        self.FORWARD_BLOCK_YAW_RAD = math.radians(float(os.getenv("AUTO_FORWARD_BLOCK_YAW_DEG", "178.0")))
        self.HEADING_COMMIT_ENTER_RAD = math.radians(95.0)
        self.HEADING_COMMIT_EXIT_RAD = math.radians(28.0)
        self.AUTO_TURN_POLICY_ENABLE = os.getenv("AUTO_TURN_POLICY_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_TURN_POLICY_SHARP_DEG = float(os.getenv("AUTO_TURN_POLICY_SHARP_DEG", "70.0"))
        self.AUTO_TURN_POLICY_UTURN_DEG = float(os.getenv("AUTO_TURN_POLICY_UTURN_DEG", "135.0"))
        self.AUTO_TURN_POLICY_MEDIUM_ENTER_DEG = float(os.getenv("AUTO_TURN_POLICY_MEDIUM_ENTER_DEG", "68.0"))
        self.AUTO_TURN_POLICY_SHARP_ENTER_DEG = float(os.getenv("AUTO_TURN_POLICY_SHARP_ENTER_DEG", "54.0"))
        self.AUTO_TURN_POLICY_GPS_JUMP_ENTER_DEG = float(os.getenv("AUTO_TURN_POLICY_GPS_JUMP_ENTER_DEG", "48.0"))
        self.AUTO_TURN_POLICY_EXIT_DEG = float(os.getenv("AUTO_TURN_POLICY_EXIT_DEG", "12.0"))
        self.AUTO_TURN_POLICY_UTURN_EXIT_DEG = float(os.getenv("AUTO_TURN_POLICY_UTURN_EXIT_DEG", "9.0"))
        self.AUTO_TURN_POLICY_PREALIGN_DIST_M = float(os.getenv("AUTO_TURN_POLICY_PREALIGN_DIST_M", "1.25"))
        self.AUTO_TURN_POLICY_PREALIGN_PWM = float(os.getenv("AUTO_TURN_POLICY_PREALIGN_PWM", "115.0"))
        self.AUTO_TURN_POLICY_CRAWL_YAW_DEG = float(os.getenv("AUTO_TURN_POLICY_CRAWL_YAW_DEG", "24.0"))
        self.AUTO_TURN_POLICY_CRAWL_PWM = float(os.getenv("AUTO_TURN_POLICY_CRAWL_PWM", "115.0"))
        self.AUTO_TURN_POLICY_SLIP_ENTER_DEG = float(os.getenv("AUTO_TURN_POLICY_SLIP_ENTER_DEG", "35.0"))
        self._last_turn_policy = "line"
        self._heading_commit_active = False
        self.turn_boost_pwm = 0.0
        self.turn_boost_ramp_per_s = float(os.getenv("AUTO_TURN_BOOST_RAMP_PWM_PER_S", "165.0"))
        self.turn_boost_decay_per_s = 180.0
        self.turn_stall_yaw_rate_rad_s = math.radians(3.0)
        self.turn_stall_err_rad = math.radians(25.0)
        self.turn_wrong_way_progress_rad_s = math.radians(float(os.getenv("AUTO_TURN_WRONG_WAY_DEG_S", "2.0")))
        self.turn_wrong_way_guard_s = float(os.getenv("AUTO_TURN_WRONG_WAY_GUARD_S", "0.60"))
        self.turn_wrong_way_override_s = float(os.getenv("AUTO_TURN_WRONG_WAY_OVERRIDE_S", "1.20"))
        self.turn_latch_release_rad = math.radians(float(os.getenv("AUTO_TURN_LATCH_RELEASE_DEG", "18.0")))
        self.turn_latch_ambiguous_rad = math.radians(float(os.getenv("AUTO_TURN_LATCH_AMBIGUOUS_DEG", "150.0")))
        self._turn_wrong_way_since = 0.0
        self._turn_direction_override_until = 0.0
        self._turn_direction_override_sign = 0.0
        self._turn_latch_sign = 0.0
        self._turn_in_place_active = False
        # If AUTO steering direction feels opposite to IMU heading, keep this enabled.
        # Pivot and forward-arc steering need separate signs with the current
        # Arduino/hardware PWM mapping. Positive heading error should make the
        # right wheel faster during forward arc tracking so the mower turns left
        # into the planned segment instead of peeling right off the line.
        self.auto_steer_invert = os.getenv("AUTO_STEER_INVERT", "1").lower() in ("1", "true", "yes", "on")
        self.auto_arc_steer_invert = os.getenv("AUTO_ARC_STEER_INVERT", "0").lower() in ("1", "true", "yes", "on")
        self.auto_pivot_steer_invert = os.getenv("AUTO_PIVOT_STEER_INVERT", "0").lower() in ("1", "true", "yes", "on")
        # Final PWM mapping for the current Jetson->driver coordinate frame.
        # Current Arduino/motor frame:
        # logical FWD(+,+)->raw(-,+), BACK(-,-)->raw(+,-),
        # LEFT(-,+)->raw(-,-), RIGHT(+,-)->raw(+,+).
        # This is rawL=-logicalR and rawR=logicalL.
        # Keep this fixed in code so old shell exports cannot silently flip the robot again.
        self.jetson_pwm_rotate_map_90 = True
        self.auto_pwm_swap_lr = False
        self.auto_pwm_invert_l = False
        self.auto_pwm_invert_r = False
        self._last_wp_warn_ts = 0.0
        self._last_auto_drive_dbg_ts = 0.0
        self._last_wp_gate_dbg_ts = 0.0
        self.wp_stop_settle_s = float(os.getenv("WP_STOP_SETTLE_S", "0.60"))
        self.wp_align_tol_rad = math.radians(float(os.getenv("WP_ALIGN_TOL_DEG", "8.0")))
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self.WAYPOINT_CONFIRM_M = float(os.getenv("WP_CONFIRM_M", "1.10"))
        self.wp_reach_hold_s = float(os.getenv("WP_REACH_HOLD_S", "0.18"))
        self.gps_reach_hold_s = float(os.getenv("GPS_REACH_HOLD_S", "0.12"))
        self.GPS_WAYPOINT_CONFIRM_WITH_DR = os.getenv("GPS_WP_CONFIRM_WITH_DR", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_WP_REQUIRE_SEGMENT_PROGRESS = os.getenv("AUTO_WP_REQUIRE_SEGMENT_PROGRESS", "0").lower() in ("1", "true", "yes", "on")
        self.GPS_WP_CONFIRM_DR_WINDOW_M = float(os.getenv("GPS_WP_CONFIRM_DR_WINDOW_M", "1.25"))
        self.GPS_WP_CONFIRM_DR_MAX_CROSSTRACK_M = float(os.getenv("GPS_WP_CONFIRM_DR_MAX_CROSSTRACK_M", "3.00"))
        self.AUTO_WP_CONFIRM_PROGRESS_WINDOW_M = float(os.getenv("AUTO_WP_CONFIRM_PROGRESS_WINDOW_M", "1.70"))
        self.AUTO_START_SNAP_ENABLE = os.getenv("AUTO_START_SNAP_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_START_SNAP_MAX_CROSSTRACK_M = float(os.getenv("AUTO_START_SNAP_MAX_CROSSTRACK_M", "0.65"))
        self.AUTO_START_SNAP_MIN_GAIN_M = float(os.getenv("AUTO_START_SNAP_MIN_GAIN_M", "2.50"))
        self.AUTO_START_SNAP_MIN_ALONG_M = float(os.getenv("AUTO_START_SNAP_MIN_ALONG_M", "0.35"))
        self.AUTO_REJOIN_ARC_ONLY_YAW_DEG = float(os.getenv("AUTO_REJOIN_ARC_ONLY_YAW_DEG", "170.0"))
        self.AUTO_ALIGN_ON_START = os.getenv("AUTO_ALIGN_ON_START", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_REJOIN_LINE_ENABLE = os.getenv("AUTO_REJOIN_LINE_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_REJOIN_HEADING_ENABLE = os.getenv("AUTO_REJOIN_HEADING_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_WP_DIRECT_HEADING_ENABLE = os.getenv("AUTO_WP_DIRECT_HEADING_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_WP_DIRECT_APPROACH_M = float(os.getenv("AUTO_WP_DIRECT_APPROACH_M", "1.00"))
        self.AUTO_WP_OVERSHOOT_RECOVER_M = float(os.getenv("AUTO_WP_OVERSHOOT_RECOVER_M", "0.35"))
        self.AUTO_OVERSHOOT_HANDOFF_M = float(os.getenv("AUTO_OVERSHOOT_HANDOFF_M", "1.20"))
        self.AUTO_OVERSHOOT_MAX_CROSSTRACK_M = float(os.getenv("AUTO_OVERSHOOT_MAX_CROSSTRACK_M", "2.20"))
        self.AUTO_FORCE_USER_WP_CORNERS = os.getenv("AUTO_FORCE_USER_WP_CORNERS", "1").lower() in ("1", "true", "yes", "on")
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self.DRIVE_STUCK_SPEED_MAX = 0.03
        self.DRIVE_STUCK_PROGRESS_MIN_MPS = float(os.getenv("DRIVE_STUCK_PROGRESS_MIN_MPS", "0.020"))
        self.DRIVE_STUCK_PROGRESS_MIN_DIST_M = float(os.getenv("DRIVE_STUCK_PROGRESS_MIN_DIST_M", "0.90"))
        self.DRIVE_STUCK_DETECT_S = float(os.getenv("DRIVE_STUCK_DETECT_S", "0.90"))
        self.TURN_STUCK_DETECT_S = float(os.getenv("TURN_STUCK_DETECT_S", "0.75"))
        self.AUTO_STUCK_RECOVERY_ENABLE = os.getenv("AUTO_STUCK_RECOVERY_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_TURN_NUDGE_RECOVERY_ENABLE = os.getenv("AUTO_TURN_NUDGE_RECOVERY_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_SLIP_PANIC_ENABLE = os.getenv("AUTO_SLIP_PANIC_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_TRACTION_ASSIST_ENABLE = os.getenv("AUTO_TRACTION_ASSIST_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.TRACTION_ASSIST_START_S = float(os.getenv("TRACTION_ASSIST_START_S", "0.35"))
        self.TRACTION_ASSIST_RAMP_PWM_PER_S = float(os.getenv("TRACTION_ASSIST_RAMP_PWM_PER_S", "35.0"))
        self.TRACTION_ASSIST_DECAY_PWM_PER_S = float(os.getenv("TRACTION_ASSIST_DECAY_PWM_PER_S", "220.0"))
        self.TRACTION_ASSIST_MAX_PWM = float(os.getenv("TRACTION_ASSIST_MAX_PWM", "18.0"))
        self.TRACTION_ASSIST_MIN_CMD_PWM = float(os.getenv("TRACTION_ASSIST_MIN_CMD_PWM", "90.0"))
        self.TURN_STUCK_YAW_PROGRESS_RAD_S = math.radians(float(os.getenv("TURN_STUCK_YAW_PROGRESS_DEG_S", "2.0")))
        self.SLIP_SIDE_RATIO_MAX = 0.28
        self.RECOVERY_REVERSE_PWM = int(os.getenv("RECOVERY_REVERSE_PWM", "112"))
        self.RECOVERY_REVERSE_S = float(os.getenv("RECOVERY_REVERSE_S", "0.24"))
        self.RECOVERY_PIVOT_PWM = int(os.getenv("RECOVERY_PIVOT_PWM", "125"))
        self.RECOVERY_PIVOT_S = 0.45
        self.RECOVERY_COOLDOWN_S = 0.9
        self.UNSTICK_WIGGLE_ENABLE = os.getenv("UNSTICK_WIGGLE_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.UNSTICK_WIGGLE_PWM = int(os.getenv("UNSTICK_WIGGLE_PWM", "122"))
        self.UNSTICK_WIGGLE_INNER_PWM = int(os.getenv("UNSTICK_WIGGLE_INNER_PWM", "88"))
        self.UNSTICK_WIGGLE_STEP_S = float(os.getenv("UNSTICK_WIGGLE_STEP_S", "0.28"))
        self.UNSTICK_WIGGLE_STEPS = int(os.getenv("UNSTICK_WIGGLE_STEPS", "5"))
        self.OBSTACLE_RECOVERY_ENABLE = os.getenv("OBSTACLE_RECOVERY_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.OBSTACLE_BACKUP_PWM = int(os.getenv("OBSTACLE_BACKUP_PWM", "110"))
        self.OBSTACLE_BACKUP_S = float(os.getenv("OBSTACLE_BACKUP_S", "0.18"))
        self.OBSTACLE_RECOVERY_PIVOT_PWM = int(os.getenv("OBSTACLE_RECOVERY_PIVOT_PWM", "125"))
        self.OBSTACLE_RECOVERY_PIVOT_S = float(os.getenv("OBSTACLE_RECOVERY_PIVOT_S", "0.45"))
        self.OBSTACLE_RECOVERY_COOLDOWN_S = float(os.getenv("OBSTACLE_RECOVERY_COOLDOWN_S", "0.65"))
        self.TURN_NUDGE_OUTER_PWM = int(os.getenv("AUTO_TURN_NUDGE_OUTER_PWM", "122"))
        self.TURN_NUDGE_INNER_PWM = int(os.getenv("AUTO_TURN_NUDGE_INNER_PWM", "90"))
        self.TURN_NUDGE_S = float(os.getenv("AUTO_TURN_NUDGE_S", "0.55"))
        self.TURN_NUDGE_STEPS = int(os.getenv("AUTO_TURN_NUDGE_STEPS", "6"))
        self.TURN_NUDGE_COOLDOWN_S = float(os.getenv("AUTO_TURN_NUDGE_COOLDOWN_S", "0.20"))
        # Floor for outer/inner wheel during wp_align and turn_in_place arcs.
        # Must be >= TURN_IN_PLACE_PWM so _smooth_turn_pwm result is not wasted,
        # and <= TURN_PROFILE_PEAK_PWM so the bell curve still controls the shape.
        self.TURN_ARC_OUTER_PWM = int(os.getenv("AUTO_TURN_ARC_OUTER_PWM", "125"))
        self.TURN_ARC_INNER_PWM = int(os.getenv("AUTO_TURN_ARC_INNER_PWM", "105"))
        self.TURN_REVERSE_ARC_OUTER_PWM = int(os.getenv("AUTO_TURN_REVERSE_ARC_OUTER_PWM", "125"))
        self.TURN_REVERSE_ARC_INNER_PWM = int(os.getenv("AUTO_TURN_REVERSE_ARC_INNER_PWM", "90"))
        self.STATIONARY_PIVOT_MIN_YAW_RAD = math.radians(float(os.getenv("AUTO_STATIONARY_PIVOT_MIN_YAW_DEG", "170.0")))
        self.SLIP_PANIC_PWM = int(os.getenv("SLIP_PANIC_PWM", "120"))
        self.SLIP_PANIC_HOLD_S = float(os.getenv("SLIP_PANIC_HOLD_S", "0.70"))
        self.SLIP_PANIC_S = float(os.getenv("SLIP_PANIC_S", "0.22"))
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_turn_sign = 1.0
        self._recovery_wiggle_step = 0
        self._recovery_cooldown_until = 0.0
        self._slip_panic_since = 0.0
        self._slip_panic_until = 0.0
        self._traction_assist_since = 0.0
        self.traction_assist_pwm = 0.0
        self._path_progress_rate_mps = 0.0
        self._last_progress_dist_to_wp = None
        self._last_progress_idx = -1
        self.AUTO_NO_PROGRESS_GUARD_ENABLE = os.getenv("AUTO_NO_PROGRESS_GUARD_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_NO_PROGRESS_CHECK_S = float(os.getenv("AUTO_NO_PROGRESS_CHECK_S", "2.80"))
        self.AUTO_NO_PROGRESS_MIN_GAIN_M = float(os.getenv("AUTO_NO_PROGRESS_MIN_GAIN_M", "0.28"))
        self.AUTO_NO_PROGRESS_RETRY_S = float(os.getenv("AUTO_NO_PROGRESS_RETRY_S", "4.00"))
        self._auto_wp_guard_idx = -1
        self._auto_wp_guard_start_dist = None
        self._auto_wp_guard_start_ts = 0.0
        self._auto_wp_guard_best_dist = None
        self._auto_force_direct_wp_until = 0.0
        self._auto_no_progress_count = 0
        self._turn_yaw_err_rate = 0.0
        self._last_turn_yaw_err_abs = None
        self.auto_speed_scale = max(0.0, min(1.0, float(os.getenv("AUTO_SPEED_DEFAULT_PCT", "30")) / 100.0))

        # ---------- PWM DEBUG ----------
        self.last_pwm_l = 0
        self.last_pwm_r = 0

        self.raw_path_points = []
        self.path_points = []
        self.path_corner_indices = set()
        self.current_idx = 0
        self._auto_start_gps = None   # robot GPS position when AUTO mission starts
        self._home_gps = None         # robot GPS position at mission start / return-home target
        self._return_home_active = False

        # ---------- GPS ----------
        self.gps_x = None
        self.gps_y = None
        self.gps_raw_x = None
        self.gps_raw_y = None
        self.gps_fix = 0
        self.gps_num_sats = 0
        self.gps_control_mode = "LIVE"
        self.gps_start_only_active = False
        self.last_gps_time = 0.0
        self.gps_from_pixhawk = False
        self._last_gps_update_ts = 0.0
        self.gps_max_speed_mps = 2.5     # reject impossible GPS jumps, but allow live mower motion
        self.gps_min_step_m = 0.8        # tolerate normal RTK/GPS step while driving
        self.gps_live_auto_alpha = float(os.getenv("GPS_LIVE_AUTO_ALPHA", "0.85"))
        self.gps_live_auto_direct = os.getenv("GPS_LIVE_AUTO_DIRECT", "1").lower() in ("1", "true", "yes", "on")
        self.gps_live_auto_max_jump_m = float(os.getenv("GPS_LIVE_AUTO_MAX_JUMP_M", "8.00"))
        self.gps_live_auto_along_alpha = float(os.getenv("GPS_LIVE_AUTO_ALONG_ALPHA", "0.45"))
        self.gps_live_auto_lateral_alpha = float(os.getenv("GPS_LIVE_AUTO_LATERAL_ALPHA", "0.24"))
        self.gps_live_auto_along_max_step_m = float(os.getenv("GPS_LIVE_AUTO_ALONG_MAX_STEP_M", "0.25"))
        self.gps_live_auto_along_min_step_m = float(os.getenv("GPS_LIVE_AUTO_ALONG_MIN_STEP_M", "0.06"))
        self.gps_live_auto_along_speed_margin_mps = float(os.getenv("GPS_LIVE_AUTO_ALONG_SPEED_MARGIN_MPS", "0.35"))
        self.gps_live_auto_lateral_max_step_m = float(os.getenv("GPS_LIVE_AUTO_LATERAL_MAX_STEP_M", "0.13"))
        self.gps_live_jump_damp_threshold_m = float(os.getenv("GPS_LIVE_JUMP_DAMP_THRESHOLD_M", "0.45"))
        self.gps_live_jump_damp_hold_s = float(os.getenv("GPS_LIVE_JUMP_DAMP_HOLD_S", "2.20"))
        self.gps_live_jump_damp_lateral_alpha = float(os.getenv("GPS_LIVE_JUMP_DAMP_LATERAL_ALPHA", "0.08"))
        self.gps_live_jump_damp_lateral_max_step_m = float(os.getenv("GPS_LIVE_JUMP_DAMP_LATERAL_MAX_STEP_M", "0.04"))
        self.gps_live_jump_damp_rejoin_xtrack_m = float(os.getenv("GPS_LIVE_JUMP_DAMP_REJOIN_XTRACK_M", "1.25"))
        self._gps_live_jump_damp_until = 0.0
        self.gps_alpha_min = 0.05        # stronger smoothing when nearly stationary
        self.gps_alpha_max = 0.18        # capped response for smoother track
        self._gps_jump_drop_count = 0
        self._last_gps_drop_log_ts = 0.0
        # Low-latency yaw filter states (unit-circle EMA)
        self._yaw_filt_s = None
        self._yaw_filt_c = None
        self.yaw_alpha_stop = 0.55
        self.yaw_alpha_move = 0.82
        self.use_raw_imu_yaw = True
        # IMU-first heading: do not freeze yaw updates when stationary.
        self.imu_yaw_hold_when_still = False
        self.imu_yaw_rate_deadband = math.radians(0.8)  # rad/s
        self.compass_heading_enu = None
        self._compass_last_ts = 0.0
        self.yaw_compass_weight_stop = 0.88
        self.yaw_compass_weight_move = 0.35
        # Use Pixhawk GLOBAL_POSITION_INT.hdg as primary heading source for control.
        self.control_use_global_heading = os.getenv("CONTROL_USE_GLOBAL_HEADING", "0").lower() in ("1", "true", "yes", "on")
        # UI heading can prefer compass-derived yaw to reduce visual drift/jumps on web.
        self.ui_use_compass_heading = os.getenv("UI_USE_COMPASS_HEADING", "0").lower() in ("1", "true", "yes", "on")
        self.ui_compass_max_age_s = float(os.getenv("UI_COMPASS_MAX_AGE_S", "0.8"))
        self.gps_ready = False
        self.gps_init_samples = deque(maxlen=30)
        self.gps_init_min_samples = 30
        self.gps_init_max_spread_m = 0.45
        self.gps_warmup_min_fix = 1
        self.gps_ready_min_fix = 3
        self.gps_allow_fix1_for_display = True
        self._gps_init_spread_m = -1.0
        self._last_gps_ready_warn_ts = 0.0
        self.gps_calibrating = False
        self.gps_calib_samples = deque(maxlen=300)
        self.gps_calib_duration_s = 8.0
        self.gps_calib_min_samples = 25
        self.gps_calib_start_ts = 0.0
        self._last_gps_calib_log_ts = 0.0

        # ---------- FUSION (Encoder + GPS) ----------
        self.dr_lat = None
        self.dr_lon = None
        self._last_enc_update_ts = 0.0
        self.speed_mps = 0.0
        self.speed_ema_alpha = 0.25
        self.enc_trust_speed_mps = 0.04
        self.gps_corr_alpha_move = 0.35
        self.gps_corr_alpha_stop = 0.08
        self.gps_corr_max_step_move_m = 0.8
        self.gps_corr_max_step_stop_m = 0.18
        self._fusion_last_corr_m = 0.0

        # ---------- LIDAR SAFETY ----------
        self.safety_distance_cm = float(os.getenv("SAFETY_DISTANCE_CM", "150.0"))
        self.safety_distance_m = self.safety_distance_cm / 100.0
        self.lidar_safety_enabled = True
        # 0.0 = no extra cap (use LiDAR driver range_max directly)
        self.lidar_max_use_m = float(os.getenv("LIDAR_MAX_USE_M", "0.0"))
        # Use a forward cone for AUTO stops. A very wide sector makes objects
        # beside the mower look like frontal blockers after it has already passed.
        self.lidar_front_sector_deg = float(os.getenv("LIDAR_FRONT_SECTOR_DEG", "170.0"))
        self.lidar_angle_offset_deg = float(os.getenv("LIDAR_ANGLE_OFFSET_DEG", "180.0"))
        self.lidar_angle_offset_rad = math.radians(self.lidar_angle_offset_deg)
        self.lidar_min_front_m = float('inf')
        self.lidar_confirmed_min_front_m = float('inf')
        self.lidar_last_update_ts = 0.0
        self.lidar_front_threat = 0.0
        self.lidar_left_threat = 0.0
        self.lidar_right_threat = 0.0
        self.lidar_close_points = 0
        self.lidar_close_cluster = 0
        self.lidar_obstacle_candidate = False
        self.lidar_obstacle_candidate_since = 0.0
        self.lidar_clear_candidate_since = 0.0
        # Start steering around obstacles before they enter the hard safety stop.
        # Safety distance still controls stop/pivot; avoid distance only biases path.
        self.lidar_avoid_distance_m = float(os.getenv("AUTO_LIDAR_AVOID_DISTANCE_M", "2.20"))
        self.lidar_avoid_clear_margin_m = float(os.getenv("AUTO_LIDAR_AVOID_CLEAR_MARGIN_M", "0.55"))
        self.lidar_hard_pivot_m = float(os.getenv("AUTO_LIDAR_HARD_PIVOT_M", "1.10"))
        self.lidar_avoid_min_base_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_BASE_PWM", "96.0"))
        self.lidar_avoid_min_steer_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_STEER_PWM", "92.0"))
        self.lidar_avoid_min_outer_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_OUTER_PWM", "118.0"))
        self.lidar_avoid_min_inner_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_INNER_PWM", "82.0"))
        self.lidar_pivot_pwm = float(os.getenv("AUTO_LIDAR_PIVOT_PWM", "120.0"))
        self.lidar_obstacle_enter_s = float(os.getenv("LIDAR_OBS_ENTER_S", "0.35"))
        self.lidar_obstacle_exit_s = float(os.getenv("LIDAR_OBS_EXIT_S", "0.80"))
        self.lidar_min_close_points = int(os.getenv("LIDAR_MIN_CLOSE_POINTS", "8"))
        self.lidar_min_close_cluster = int(os.getenv("LIDAR_MIN_CLOSE_CLUSTER", "5"))
        self.lidar_min_cluster_points = int(os.getenv("LIDAR_MIN_CLUSTER_POINTS", "4"))
        self.lidar_min_cluster_width_m = float(os.getenv("LIDAR_MIN_CLUSTER_WIDTH_M", "0.06"))
        self.lidar_cluster_max_gap = int(os.getenv("LIDAR_CLUSTER_MAX_GAP", "2"))
        self.lidar_cluster_max_range_jump_m = float(os.getenv("LIDAR_CLUSTER_MAX_RANGE_JUMP_M", "0.45"))
        self.lidar_min_threat = float(os.getenv("LIDAR_MIN_THREAT", "0.12"))
        self.lidar_noise_margin_m = float(os.getenv("LIDAR_NOISE_MARGIN_M", "0.18"))
        self.lidar_swap_lr = os.getenv("LIDAR_SWAP_LR", "0").lower() in ("1", "true", "yes", "on")
        self.lidar_obstacle_active = False
        self.lidar_avoid_turn = 0.0
        self.lidar_avoid_turn_filt = 0.0
        self.lidar_avoid_filter_alpha = float(os.getenv("AUTO_LIDAR_AVOID_FILTER_ALPHA", "0.28"))
        self.lidar_avoid_bias_max = math.radians(float(os.getenv("AUTO_LIDAR_AVOID_BIAS_MAX_DEG", "55.0")))
        self.lidar_avoid_commit_sign = 0.0
        self.lidar_avoid_commit_until = 0.0
        self.lidar_avoid_hold_s = float(os.getenv("AUTO_LIDAR_AVOID_HOLD_S", "6.50"))
        self.auto_lidar_avoid_enable = os.getenv("AUTO_LIDAR_AVOID_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.lidar_avoid_release_ratio = float(os.getenv("AUTO_LIDAR_AVOID_RELEASE_RATIO", "0.62"))
        self.lidar_escape_min_turn_rad = math.radians(float(os.getenv("AUTO_LIDAR_ESCAPE_MIN_TURN_DEG", "38.0")))
        self.lidar_escape_front_deg = float(os.getenv("AUTO_LIDAR_ESCAPE_FRONT_DEG", "42.0"))
        self.lidar_escape_threat_bias = float(os.getenv("AUTO_LIDAR_ESCAPE_THREAT_BIAS", "0.10"))
        self.orchard_avoid_enable = os.getenv("AUTO_ORCHARD_AVOID_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.orchard_candidate_step_deg = float(os.getenv("AUTO_ORCHARD_CANDIDATE_STEP_DEG", "7.5"))
        self.orchard_candidate_max_deg = float(os.getenv("AUTO_ORCHARD_CANDIDATE_MAX_DEG", "72.0"))
        self.orchard_min_clear_ahead_m = float(os.getenv("AUTO_ORCHARD_MIN_CLEAR_AHEAD_M", "2.20"))
        self.orchard_gap_score_bias = float(os.getenv("AUTO_ORCHARD_GAP_SCORE_BIAS", "1.35"))
        self.orchard_path_bias = float(os.getenv("AUTO_ORCHARD_PATH_BIAS", "0.80"))
        self.orchard_side_change_penalty = float(os.getenv("AUTO_ORCHARD_SIDE_CHANGE_PENALTY", "0.45"))
        self.orchard_no_gap_stop_m = float(os.getenv("AUTO_ORCHARD_NO_GAP_STOP_M", "1.80"))
        self.orchard_crawl_pwm = float(os.getenv("AUTO_ORCHARD_CRAWL_PWM", "128.0"))
        self.orchard_gap_turn_boost_deg = float(os.getenv("AUTO_ORCHARD_GAP_TURN_BOOST_DEG", "18.0"))
        # Local corridor planner for tree/obstacle-dense areas. This keeps
        # avoidance reactive and lightweight, but chooses the clearest forward
        # gap instead of only comparing total left-vs-right threat.
        self.lidar_corridor_enable = os.getenv("AUTO_LIDAR_CORRIDOR_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.lidar_corridor_sectors = max(5, int(os.getenv("AUTO_LIDAR_CORRIDOR_SECTORS", "13")))
        # Actual mower front/body width is about 0.60m. Keep this default close
        # to the real footprint; safety clearance is added separately below.
        # If this is too wide, trees beside a 3-4m orchard lane can look like a
        # blocked front corridor even when the mower can physically pass.
        self.robot_width_m = float(os.getenv("ROBOT_WIDTH_M", "0.60"))
        self.robot_length_m = float(os.getenv("ROBOT_LENGTH_M", "1.60"))
        self.robot_front_overhang_m = float(os.getenv("ROBOT_FRONT_OVERHANG_M", str(max(0.0, self.robot_length_m * 0.5))))
        self.auto_front_clearance_m = float(os.getenv("AUTO_FRONT_CLEARANCE_M", "2.20"))
        self.front_stop_distance_m = max(0.0, self.robot_front_overhang_m + self.auto_front_clearance_m)
        self.lidar_hard_pivot_m = max(self.lidar_hard_pivot_m, self.front_stop_distance_m)
        self.lidar_corridor_side_margin_m = float(os.getenv("AUTO_LIDAR_CORRIDOR_SIDE_MARGIN_M", "0.40"))
        self.lidar_corridor_min_clear_m = float(os.getenv(
            "AUTO_LIDAR_CORRIDOR_MIN_CLEAR_M",
            str(max(1.55, self.robot_width_m + 2.0 * self.lidar_corridor_side_margin_m))
        ))
        # Corridor blocked at 3m should mean "plan a slower escape", not "freeze".
        # Only treat a blocked corridor as a hard stop once the obstacle is in
        # the close safety bubble; farther obstacles are handled by steering.
        self.lidar_corridor_stop_m = float(os.getenv(
            "AUTO_LIDAR_CORRIDOR_STOP_M",
            str(max(self.safety_distance_m, self.lidar_hard_pivot_m))
        ))
        self.lidar_corridor_prefer_front = float(os.getenv("AUTO_LIDAR_CORRIDOR_FRONT_WEIGHT", "0.35"))
        self.lidar_corridor_blocked = False
        self.lidar_corridor_best_angle = 0.0
        self.lidar_corridor_best_clear_m = float('inf')
        self.lidar_corridor_sector_clear = []
        self.lidar_force_pivot_m = float(os.getenv("AUTO_LIDAR_FORCE_PIVOT_M", "1.25"))
        self.lidar_planned_bypass_min_clear_m = float(os.getenv("AUTO_LIDAR_PLANNED_BYPASS_MIN_CLEAR_M", "1.60"))
        # Footprint-aware local planner: use real LiDAR points in robot frame to
        # test whether the mower body plus safety buffer would collide along
        # candidate directions. This is the same geometry shown in the 3D/top
        # view, but used for control instead of display only.
        self.lidar_footprint_enable = os.getenv("AUTO_LIDAR_FOOTPRINT_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.lidar_footprint_buffer_m = float(os.getenv("AUTO_LIDAR_FOOTPRINT_BUFFER_M", "0.40"))
        self.lidar_footprint_lookahead_m = float(os.getenv("AUTO_LIDAR_FOOTPRINT_LOOKAHEAD_M", "4.20"))
        self.lidar_footprint_stop_margin_m = float(os.getenv("AUTO_LIDAR_FOOTPRINT_STOP_MARGIN_M", "0.15"))
        self.lidar_footprint_best_angle = 0.0
        self.lidar_footprint_best_clear_m = float('inf')
        self.lidar_footprint_blocked = False
        self.lidar_footprint_min_collision_m = float('inf')
        self.lidar_footprint_points = []
        self.footprint_collision_enable = os.getenv("AUTO_FOOTPRINT_POLYGON_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.footprint_collision_buffer_m = float(os.getenv("AUTO_FOOTPRINT_POLYGON_BUFFER_M", "0.28"))
        self.footprint_polygon = self._load_footprint_polygon()
        self.lidar_raw_hard_stop_enable = os.getenv("AUTO_LIDAR_RAW_HARD_STOP_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.lidar_raw_hard_stop_points = int(os.getenv("AUTO_LIDAR_RAW_HARD_STOP_POINTS", "3"))
        self.lidar_raw_hard_stop_m = float('inf')
        self.lidar_raw_hard_stop_active = False
        # 2D rolling local costmap around the mower. This is a lightweight
        # explorer-style map: LiDAR + short obstacle memory are rasterized in
        # robot frame, inflated by mower footprint, then candidate headings are
        # checked against the inflated grid. It never changes waypoint order.
        self.local_costmap_enable = os.getenv("AUTO_LOCAL_COSTMAP_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.local_costmap_size_m = float(os.getenv("AUTO_LOCAL_COSTMAP_SIZE_M", "8.0"))
        self.local_costmap_resolution_m = float(os.getenv("AUTO_LOCAL_COSTMAP_RESOLUTION_M", "0.12"))
        self.local_costmap_back_m = float(os.getenv("AUTO_LOCAL_COSTMAP_BACK_M", "1.2"))
        self.local_costmap_inflate_m = float(os.getenv(
            "AUTO_LOCAL_COSTMAP_INFLATE_M",
            str((self.robot_width_m * 0.5) + self.lidar_footprint_buffer_m)
        ))
        self.local_costmap_lookahead_m = float(os.getenv("AUTO_LOCAL_COSTMAP_LOOKAHEAD_M", "4.60"))
        self.local_costmap_sample_step_m = float(os.getenv("AUTO_LOCAL_COSTMAP_SAMPLE_STEP_M", "0.14"))
        self.local_costmap_active = False
        self.local_costmap_blocked = False
        self.local_costmap_best_angle = 0.0
        self.local_costmap_best_clear_m = float('inf')
        self.local_costmap_min_collision_m = float('inf')
        self.local_costmap_grid = []
        self.local_costmap_points = []
        # Short-lived obstacle memory. This is not full SLAM; it remembers
        # recently seen LiDAR clusters in GPS-local space so AUTO does not snap
        # back into an obstacle as soon as it leaves the forward scan cone.
        self.obstacle_memory_enable = os.getenv("AUTO_OBSTACLE_MEMORY_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.obstacle_memory_ttl_s = float(os.getenv("AUTO_OBSTACLE_MEMORY_TTL_S", "14.0"))
        self.obstacle_memory_max_points = int(os.getenv("AUTO_OBSTACLE_MEMORY_MAX_POINTS", "180"))
        self.obstacle_memory_merge_m = float(os.getenv("AUTO_OBSTACLE_MEMORY_MERGE_M", "0.35"))
        self.obstacle_memory_lookahead_m = float(os.getenv("AUTO_OBSTACLE_MEMORY_LOOKAHEAD_M", "6.0"))
        self.obstacle_memory_behind_m = float(os.getenv("AUTO_OBSTACLE_MEMORY_BEHIND_M", "1.0"))
        self.obstacle_memory_side_margin_m = float(os.getenv("AUTO_OBSTACLE_MEMORY_SIDE_MARGIN_M", "0.55"))
        self.obstacle_memory_corridor_half_m = float(os.getenv(
            "AUTO_OBSTACLE_MEMORY_CORRIDOR_HALF_M",
            str((self.robot_width_m * 0.5) + self.obstacle_memory_side_margin_m)
        ))
        self.obstacle_memory_turn_bias_rad = math.radians(float(os.getenv("AUTO_OBSTACLE_MEMORY_TURN_DEG", "24.0")))
        self.obstacle_memory = []
        self.obstacle_memory_active = False
        self.obstacle_memory_min_m = float('inf')
        self.obstacle_memory_turn = 0.0
        # Enable real AUTO obstacle bypass by default: when LiDAR confirms an
        # obstacle on the current segment, insert short temporary waypoints to
        # step around it, then return to the original path.
        self.auto_bypass_enable = os.getenv("AUTO_BYPASS_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.auto_bypass_side_offset_m = float(os.getenv("AUTO_BYPASS_SIDE_OFFSET_M", "1.65"))
        self.auto_bypass_forward_m = float(os.getenv("AUTO_BYPASS_FORWARD_M", "1.20"))
        self.auto_bypass_pass_m = float(os.getenv("AUTO_BYPASS_PASS_M", "4.20"))
        self.auto_bypass_return_m = float(os.getenv("AUTO_BYPASS_RETURN_M", "5.20"))
        self.auto_bypass_min_remaining_m = float(os.getenv("AUTO_BYPASS_MIN_REMAINING_M", "2.20"))
        self.auto_bypass_cooldown_s = float(os.getenv("AUTO_BYPASS_COOLDOWN_S", "5.0"))
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1
        self._auto_bypass_last_ts = 0.0
        self._base_path_points = []
        self._base_path_corner_indices = set()
        self.steer_cmd_filt = 0.0
        self.steer_filter_alpha = float(os.getenv("AUTO_STEER_FILTER_ALPHA", "0.12"))
        self._last_pixhawk_retry_ts = 0.0

        # ---------- FOLLOW TRACKER CMD ----------
        self.follow_tracker_enabled = os.getenv("FOLLOW_TRACKER_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.follow_cmd = "STOP"
        self.follow_cmd_ts = 0.0
        self._last_follow_cmd_wait_warn_ts = 0.0
        self.follow_pwm_override_l = 0
        self.follow_pwm_override_r = 0
        self.follow_cmd_timeout_s = float(os.getenv("FOLLOW_CMD_TIMEOUT_S", "1.8"))
        self.follow_forward_pwm = float(os.getenv("FOLLOW_FORWARD_PWM", "92"))
        self.follow_turn_pwm = float(os.getenv("FOLLOW_TURN_PWM", "105"))
        self.follow_lidar_stop_m = float(os.getenv("FOLLOW_LIDAR_STOP_M", "2.0"))
        self.follow_stall_boost_pwm = 0.0
        self.follow_stall_boost_ramp = float(os.getenv("FOLLOW_STALL_BOOST_RAMP", "40.0"))
        self.follow_stall_boost_decay = float(os.getenv("FOLLOW_STALL_BOOST_DECAY", "120.0"))
        self.follow_stall_boost_max = float(os.getenv("FOLLOW_STALL_BOOST_MAX", "22.0"))
        # Wheel-side encoder trim is disabled by default because both drive
        # sides are now mechanically matched. Keep the env hook only for tests.
        self.follow_balance_enabled = os.getenv("FOLLOW_BALANCE_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.follow_balance_kp = float(os.getenv("FOLLOW_BALANCE_KP", "0.35"))
        self.follow_balance_max_pwm = float(os.getenv("FOLLOW_BALANCE_MAX_PWM", "24.0"))
        self.follow_balance_deadband_pulse = float(os.getenv("FOLLOW_BALANCE_DEADBAND_PULSE", "8.0"))
        self.follow_balance_alpha = float(os.getenv("FOLLOW_BALANCE_ALPHA", "0.25"))
        self.follow_balance_cmd_diff_max = float(os.getenv("FOLLOW_BALANCE_CMD_DIFF_MAX", "45.0"))
        self._follow_balance_corr_pwm = 0.0
        self._last_follow_boost_ts = time.time()
        self._last_follow_dbg_ts = 0.0

        # ---------- CAMERA LANE ASSIST ----------
        self.camera_lane_assist_enable = os.getenv("CAMERA_LANE_ASSIST_USE", "0").lower() in ("1", "true", "yes", "on")
        self.camera_lane_max_age_s = float(os.getenv("CAMERA_LANE_MAX_AGE_S", "0.65"))
        self.camera_lane_min_conf = float(os.getenv("CAMERA_LANE_MIN_CONF", "0.35"))
        self.camera_lane_max_bias_rad = math.radians(float(os.getenv("CAMERA_LANE_CONTROL_MAX_BIAS_DEG", "12.0")))
        self.camera_lane_obstacle_scale = float(os.getenv("CAMERA_LANE_OBSTACLE_SCALE", "0.35"))
        self.camera_lane_bias_rad = 0.0
        self.camera_lane_conf = 0.0
        self.camera_lane_ts = 0.0
        self.camera_lane_ok = False
        self.camera_lane_debug = {}

        # ---------- DEBUG ----------
        self._dbg_ts = time.time()

        # ---------- ROS ----------
        self.gps_pub = self.create_publisher(Float32MultiArray, '/current_gps', 10)
        self.robot_status_mode_pub = self.create_publisher(Float32, '/robot/status/mode', 10)
        self.robot_status_enc_l_pub = self.create_publisher(Float32, '/robot/status/enc_l', 10)
        self.robot_status_enc_r_pub = self.create_publisher(Float32, '/robot/status/enc_r', 10)
        self.robot_status_pump_pub = self.create_publisher(Bool, '/robot/status/pump', 10)
        self.robot_status_blade_pub = self.create_publisher(Float32, '/robot/status/blade', 10)
        self.robot_status_emg_pub = self.create_publisher(Bool, '/robot/status/emg', 10)
        self.robot_battery_soc_pub = self.create_publisher(Float32, '/robot/battery/soc', 10)
        self.robot_battery_voltage_pub = self.create_publisher(Float32, '/robot/battery/voltage', 10)
        self.robot_battery_current_pub = self.create_publisher(Float32, '/robot/battery/current', 10)
        self.robot_battery_temp_pub = self.create_publisher(Float32, '/robot/battery/temp', 10)
        self.robot_battery_status_pub = self.create_publisher(Float32, '/robot/battery/status', 10)
        self.create_subscription(String, '/auto_control', self.web_mode_cb, 10)
        self.create_subscription(Float32MultiArray, '/waypoint', self.waypoint_cb, 10)
        self.create_subscription(String, '/emergency_stop', self.emergency_cb, 10)
        self.create_subscription(String, '/manual_cmd', self.manual_cb, 10)
        self.create_subscription(String, '/follow_cmd', self.follow_cmd_cb, 10)
        self.follow_status_pub = self.create_publisher(String, "/follow_vision_status", 10)
        self.create_subscription(Float32, '/safety_distance_cm', self.safety_distance_cb, 10)
        self.create_subscription(Float32, '/lidar_max_use_m', self.lidar_max_use_cb, 10)
        self.create_subscription(Bool, '/lidar_safety_enable', self.lidar_safety_enable_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.create_subscription(String, '/camera_lane_assist', self.camera_lane_cb, 10)


        # ---------- SERIAL (Arduino) ----------
        self.ser = None
        try:
            self.ser = self.connect_arduino(self.ENCODER_PORT_CANDIDATES)
            self.arduino_connected = True
            self._last_arduino_rx_ts = time.time()
        except Exception as e:
            self.arduino_connected = False
            self._last_arduino_rx_ts = 0.0
            self.get_logger().error(f"❌ Arduino initial connect failed; node will retry: {e}")

        # ---------- MAVLINK ----------
        self.master = None
        self._pixhawk_connect_in_progress = False
        self._pixhawk_retry_interval_s = float(os.getenv("PIXHAWK_RETRY_INTERVAL_S", "5.0"))
        self._last_pixhawk_retry_ts = 0.0
        self.get_logger().warn("⚠️ Pixhawk connect will run in background; Arduino/status loop starts immediately")

        # ---------- TIMER ----------
        self.timer = self.create_timer(0.005, self.update)  # 200 Hz

        self.get_logger().info("[STATE] READY | MANUAL")
        self.get_logger().info(f"[CFG] AUTO_STEER_INVERT={self.auto_steer_invert}")
        self.get_logger().info(f"[CFG] AUTO_ARC_STEER_INVERT={self.auto_arc_steer_invert}")
        self.get_logger().info(f"[CFG] AUTO_PIVOT_STEER_INVERT={self.auto_pivot_steer_invert}")
        self.get_logger().info(
            f"[CFG] AUTO_LINE_STRAIGHT_HOLD_DEG={math.degrees(self.AUTO_LINE_STRAIGHT_HOLD_RAD):.1f} "
            f"AUTO_STUCK_RECOVERY_ENABLE={self.AUTO_STUCK_RECOVERY_ENABLE} "
            f"AUTO_SLIP_PANIC_ENABLE={self.AUTO_SLIP_PANIC_ENABLE}"
        )
        self.get_logger().info(
            f"[CFG] WP_TURN_REACH_M={self.WAYPOINT_TURN_REACH_M:.2f} "
            f"PATH_DENSIFY_ENABLE={self.PATH_DENSIFY_ENABLE} "
            f"PATH_DENSIFY_SPACING_M={self.PATH_DENSIFY_SPACING_M:.2f} "
            f"PATH_DENSE_REACH_M={self.PATH_DENSE_REACH_M:.2f} "
            f"GPS_REACH_HOLD_S={self.gps_reach_hold_s:.2f} "
            f"GPS_WP_CONFIRM_WITH_DR={self.GPS_WAYPOINT_CONFIRM_WITH_DR} "
            f"WP_ALIGN_TOL_DEG={math.degrees(self.wp_align_tol_rad):.1f} "
            f"FRONT_CLEARANCE_M={self.auto_front_clearance_m:.2f} "
            f"FRONT_STOP_DISTANCE_M={self.front_stop_distance_m:.2f} "
            f"AUTO_DR_FINAL_PASS_MARGIN_M={self.AUTO_DR_FINAL_PASS_MARGIN_M:.2f} "
            f"AUTO_DR_FINAL_MAX_CROSSTRACK_M={self.AUTO_DR_FINAL_MAX_CROSSTRACK_M:.2f} "
            f"AUTO_GPS_CORRIDOR_CONFIRM_M={self.AUTO_GPS_CORRIDOR_CONFIRM_M:.2f}"
        )
        self.get_logger().info(
            f"[CFG] GPS_LIVE_AUTO_DIRECT={self.gps_live_auto_direct} "
            f"GPS_LIVE_AUTO_MAX_JUMP_M={self.gps_live_auto_max_jump_m:.2f}"
        )
        self.get_logger().info(
            f"[CFG] JETSON_PWM_ROTATE_MAP_90={self.jetson_pwm_rotate_map_90} "
            f"AUTO_PWM_SWAP_LR={self.auto_pwm_swap_lr} "
            f"AUTO_PWM_INVERT_L={self.auto_pwm_invert_l} AUTO_PWM_INVERT_R={self.auto_pwm_invert_r}"
        )

    # =================================================
    # CALLBACKS
    # =================================================

    def get_encoder_port_candidates(self, configured_candidates):
        ports = []
        seen = set()

        def add_port(p):
            if not p or p in seen:
                return
            seen.add(p)
            ports.append(p)

        # 1) configured candidates first
        for p in configured_candidates:
            add_port(p)

        # In the mower, Arduino must be addressed by its stable by-id name only.
        # Falling back to /dev/ttyACM* can accidentally grab Pixhawk or another
        # Arduino after a reboot/replug and make AUTO appear stuck or dead.
        if getattr(self, "arduino_by_id_lock", True) or getattr(self, "arduino_port_override", ""):
            return ports

        # 2) stable by-id names for common USB-UART chipsets (exclude Pixhawk/Holybro)
        by_id_patterns = ["/dev/serial/by-id/*Arduino*"]
        for pattern in by_id_patterns:
            for p in sorted(glob.glob(pattern)):
                name = os.path.basename(p).lower()
                if "holybro" in name or "pixhawk" in name or "silicon_labs" in name or "cp210" in name:
                    continue
                add_port(p)

        # Prefer stable by-id device names. If Arduino by-id exists, do not also
        # try /dev/ttyACM* aliases; ACM numbers can move between reboots/plugs.
        if any(p.startswith("/dev/serial/by-id/") and os.path.exists(p) for p in ports):
            return ports

        # 3) fallback to ACM only (avoid LiDAR on ttyUSB*)
        for p in sorted(glob.glob("/dev/ttyACM*")):
            add_port(p)

        return ports

    def connect_arduino(self, configured_candidates, baud=115200, timeout=None, require_handshake=None, dtr_toggle=None):
        if timeout is None:
            timeout = float(os.getenv("ARDUINO_SERIAL_TIMEOUT_S", "0.0"))
        candidates = self.get_encoder_port_candidates(configured_candidates)
        pixhawk_reals = {os.path.realpath(p) for p in self.PIXHAWK_PORT_CANDIDATES}
        for pattern in ("/dev/serial/by-id/*Holybro*", "/dev/serial/by-id/*Pixhawk*"):
            pixhawk_reals.update(os.path.realpath(p) for p in glob.glob(pattern))
        last_error = None
        for port in candidates:
            if port.startswith("/dev/") and not os.path.exists(port):
                continue
            # Never steal Pixhawk serial port
            if os.path.realpath(port) in pixhawk_reals:
                continue
            try:
                # Match the older known-good Arduino connection path:
                # no exclusive lock, no write timeout, no handshake gate.
                ser = serial.Serial(port, baud, timeout=timeout)
                try:
                    ser.setDTR(False)
                except Exception:
                    pass
                try:
                    time.sleep(float(os.getenv("ARDUINO_BOOT_DELAY_S", "1.0")))
                except Exception:
                    pass
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                self.get_logger().info(f"✅ Arduino Ready: {port}")
                return ser
            except Exception as e:
                last_error = e
                self.get_logger().warn(f"⚠️ Arduino not available: {port} ({e})")

        raise RuntimeError(
            f"Cannot open Arduino on any candidate port: {candidates} | last_error={last_error}"
        )

    def _usb_reset_serial_device(self, serial_port):
        if not (os.getenv("ARDUINO_USB_RESET_ON_TIMEOUT", "1").lower() in ("1", "true", "yes", "on")):
            return False
        real_port = os.path.realpath(serial_port or "")
        tty_name = os.path.basename(real_port)
        if not tty_name:
            return False
        sys_base = f"/sys/class/tty/{tty_name}/device/.."
        try:
            with open(os.path.join(sys_base, "busnum"), "r", encoding="ascii") as f:
                bus = int(f.read().strip())
            with open(os.path.join(sys_base, "devnum"), "r", encoding="ascii") as f:
                dev = int(f.read().strip())
        except Exception as e:
            self.get_logger().warn(f"⚠️ Arduino USB reset unavailable: cannot resolve bus/dev for {real_port}: {e}")
            return False

        usb_node = f"/dev/bus/usb/{bus:03d}/{dev:03d}"
        USBDEVFS_RESET = 21780
        try:
            fd = os.open(usb_node, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            finally:
                os.close(fd)
            self.get_logger().warn(f"🔌 Arduino USB device reset: {usb_node}")
            time.sleep(2.0)
            return True
        except PermissionError:
            self.get_logger().error(
                f"❌ Arduino USB reset needs permission for {usb_node}; add a udev rule or run reset helper as root"
            )
        except Exception as e:
            self.get_logger().warn(f"⚠️ Arduino USB reset failed for {usb_node}: {e}")
        return False

    def get_pixhawk_port_candidates(self, configured_candidates):
        ports = []
        seen = set()
        arduino_reals = {os.path.realpath(p) for p in self.ENCODER_PORT_CANDIDATES}
        for pattern in ("/dev/serial/by-id/*Arduino*",):
            arduino_reals.update(os.path.realpath(p) for p in glob.glob(pattern))

        def add_port(p):
            if not p or p in seen:
                return
            if os.path.realpath(p) in arduino_reals:
                return
            seen.add(p)
            ports.append(p)

        # 1) configured candidates first
        for p in configured_candidates:
            add_port(p)

        # 2) Pixhawk/Holybro by-id names
        by_id_patterns = [
            "/dev/serial/by-id/*Holybro*",
            "/dev/serial/by-id/*Pixhawk*",
        ]
        for pattern in by_id_patterns:
            for p in sorted(glob.glob(pattern)):
                add_port(p)

        # If stable by-id Pixhawk ports exist, do not try the same devices again
        # through /dev/ttyACM*. Duplicate ACM retries can block startup long
        # enough that the Arduino link times out before the main loop starts.
        if ports:
            return ports

        # 3) fallback to ACM ports commonly used by Pixhawk USB CDC
        for p in sorted(glob.glob("/dev/ttyACM*")):
            add_port(p)

        return ports

    def connect_pixhawk(self, candidates, baud=921600):
        candidates = self.get_pixhawk_port_candidates(candidates)
        hb_timeout = float(os.getenv("PIXHAWK_HEARTBEAT_TIMEOUT_S", "0.60"))
        last_error = None
        for port in candidates:
            master = None
            try:
                master = mavutil.mavlink_connection(
                    port,
                    baud=baud,
                    autoreconnect=False,
                    robust_parsing=True,
                    source_system=255,
                    retries=0,
                    timeout=0.20
                )
                hb = master.wait_heartbeat(timeout=hb_timeout)
                if hb is None:
                    raise RuntimeError("no heartbeat")
                self.PIXHAWK_PORT = port
                self.get_logger().info(f"✅ Pixhawk connected: {port}")
                return master
            except Exception as e:
                last_error = e
                try:
                    master.close()
                except Exception:
                    pass
                self.get_logger().warn(f"⚠️ Pixhawk not available: {port} ({e})")
        raise RuntimeError(
            f"Cannot open Pixhawk on any candidate port: {candidates} | last_error={last_error}"
        )

    def setup_mavlink_streams(self):
        try:
            # Force stream rates so ATTITUDE/GPS arrive even when autopilot defaults are low.
            requests = [
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5_000),        # 200 Hz
                (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 100_000),   # 10 Hz
                (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50_000),  # 20 Hz
            ]
            for msg_id, interval_us in requests:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(msg_id),
                    float(interval_us),
                    0, 0, 0, 0, 0
                )
        except Exception as e:
            self.get_logger().warn(f"⚠️ MAVLink stream setup failed: {e}")

    def start_pixhawk_connect_async(self):
        if self._pixhawk_connect_in_progress:
            return
        self._pixhawk_connect_in_progress = True

        def worker():
            master = None
            try:
                master = self.connect_pixhawk(self.PIXHAWK_PORT_CANDIDATES)
                self.master = master
                self.setup_mavlink_streams()
            except Exception as e:
                try:
                    if master is not None:
                        master.close()
                except Exception:
                    pass
                self.get_logger().warn(f"⚠️ Pixhawk reconnect failed: {e}")
            finally:
                self._last_pixhawk_retry_ts = time.time()
                self._pixhawk_connect_in_progress = False

        threading.Thread(target=worker, daemon=True).start()
    
    def web_mode_cb(self, msg):
        raw_cmd = msg.data.strip().upper()
        cmd = raw_cmd

        # Normalize web command aliases so AUTO/FOLLOW pages can use dedicated labels.
        if cmd == "AUTO_START":
            self.web_run_mode = "AUTO"
            cmd = "START"
        elif cmd == "FOLLOW_START":
            self.web_run_mode = "FOLLOW"
            self.follow_cmd = "STOP"
            self.follow_cmd_ts = 0.0
            self._last_follow_cmd_wait_warn_ts = 0.0
            cmd = "START"
        elif cmd == "START":
            self.web_run_mode = "AUTO"
        elif cmd in ["AUTO_PAUSE", "FOLLOW_PAUSE", "PAUSE"]:
            cmd = "PAUSE"

        if cmd != self.mode_web:
            self.get_logger().info(f"▶ WEB CMD: {cmd}")

        if cmd in ["GPS_REFRESH", "REFRESH_GPS", "GPS_RESET"]:
            self.reset_gps_estimator()
            return
        if cmd in ["GPS_RESYNC", "RESYNC_GPS", "GPS_RESYNC"]:
            self.resync_gps_state()
            return
        if cmd in ["GPS_CALIBRATE", "CALIBRATE_GPS", "GPS_CAL"]:
            self.start_gps_calibration()
            return
        if cmd in ["GPSMODE_START_ONLY", "GPS_MODE_START_ONLY", "GPS_START_ONLY"]:
            self.gps_control_mode = "START_ONLY"
            self.gps_start_only_active = False
            self.get_logger().warn("🛰️ GPS control mode -> START_ONLY")
            return
        if cmd in ["GPSMODE_LIVE", "GPS_MODE_LIVE", "GPS_LIVE"]:
            self.gps_control_mode = "LIVE"
            self.gps_start_only_active = False
            self.get_logger().warn("🛰️ GPS control mode -> LIVE")
            return
        if cmd.startswith("AUTOSPEED,"):
            try:
                pct = float(cmd.split(",", 1)[1])
                pct = max(0.0, min(100.0, pct))
                self.auto_speed_scale = pct / 100.0
                self.get_logger().warn(f"🚜 AUTO speed scale -> {pct:.0f}%")
            except Exception:
                self.get_logger().warn(f"⚠️ Invalid AUTOSPEED command: {cmd}")
            return

        if cmd in ["AUTO_ASSIST_ON", "ASSIST_ON", "MANUAL_ASSIST_ON"]:
            if not self.AUTO_MANUAL_ASSIST_ENABLE:
                self.auto_manual_assist_enabled = False
                self.auto_manual_assist_drive_until = 0.0
                self.send_pwm(0, 0)
                self.get_logger().warn("🕹️ DRIVE ASSIST ignored: disabled in simple waypoint mode")
                return
            self.auto_manual_assist_enabled = True
            self.auto_manual_assist_drive_until = 0.0
            self.send_pwm(0, 0)
            self._last_ctrl_time = time.time()
            self._wp_reach_candidate_idx = -1
            self._wp_reach_candidate_since = 0.0
            self._drive_stuck_since = 0.0
            self._turn_stuck_since = 0.0
            self._recovery_phase = ""
            self._recovery_until = 0.0
            self._recovery_cooldown_until = 0.0
            self.turn_boost_pwm = 0.0
            self.drive_stall_boost_pwm = 0.0
            self._drive_boost_since = 0.0
            self.pid_yaw.reset()
            self.pid_speed.reset()
            self.get_logger().warn(
                f"🕹️ DRIVE ASSIST ON: manual override for AUTO/FOLLOW, WP "
                f"{self.current_idx + 1}/{len(self.path_points) if self.path_points else 0}"
            )
            return
        if cmd in ["AUTO_ASSIST_OFF", "ASSIST_OFF", "MANUAL_ASSIST_OFF"]:
            self.auto_manual_assist_enabled = False
            self.auto_manual_assist_drive_until = 0.0
            self._wp_reach_candidate_idx = -1
            self._wp_reach_candidate_since = 0.0
            self._last_ctrl_time = time.time()
            self.pid_yaw.reset()
            self.pid_speed.reset()
            self.get_logger().warn("🕹️ DRIVE ASSIST OFF: AUTO/FOLLOW controller may resume")
            return

        if cmd in ["RETURN_HOME", "RTH", "GO_HOME"]:
            self.return_home()
            return

        # ===== PAUSE = hold mission state, only stop motion =====
        if cmd == "PAUSE":
            self.mode_web = "PAUSE"
            self.send_pwm(0, 0)
            self._last_ctrl_time = time.time()
            self._last_pwm_send_ts = time.time()
            self._wp_reach_candidate_idx = -1
            self._wp_reach_candidate_since = 0.0
            self._drive_stuck_since = 0.0
            self._turn_stuck_since = 0.0
            self._recovery_phase = ""
            self._recovery_until = 0.0
            self._recovery_cooldown_until = 0.0
            self._reset_motion_progress_state()
            self.turn_boost_pwm = 0.0
            self.drive_stall_boost_pwm = 0.0
            self._drive_boost_since = 0.0
            self.pid_yaw.reset()
            self.pid_speed.reset()
            self.get_logger().warn(
                f"⏸️ PAUSE: mission held at WP {self.current_idx + 1}/{len(self.path_points) if self.path_points else 0}"
            )
            return

        # ===== STOP = soft reset AUTO/FOLLOW =====
        if cmd == "STOP":
            self.mode_web = "STOP"
            self.web_run_mode = "AUTO"
            self.auto_manual_assist_enabled = False
            self.gps_start_only_active = False
            self.send_pwm(0, 0)
            self._clear_mission_path()
            try:
                self.pid_yaw.reset()
                self.pid_speed.reset()
            except:
                pass
            self.wp_align_active = False
            self.wp_settle_until = 0.0
            self._wp_reach_candidate_idx = -1
            self._wp_reach_candidate_since = 0.0
            self._drive_stuck_since = 0.0
            self._turn_stuck_since = 0.0
            self._recovery_phase = ""
            self._recovery_until = 0.0
            self._recovery_cooldown_until = 0.0
            self._reset_motion_progress_state()
            self.turn_boost_pwm = 0.0
            self.drive_stall_boost_pwm = 0.0
            self._drive_boost_since = 0.0
            self._auto_bypass_inserted = False
            self._auto_bypass_end_idx = -1
            self._return_home_active = False
            self._reset_auto_lidar_avoid_state()

            self.get_logger().warn("⏹️ STOP: AUTO/FOLLOW RESET + TARGET CLEARED (upload waypoint again)")
            return

        if cmd == "START":
            was_paused = self.mode_web == "PAUSE"
            self._last_ctrl_time = time.time()
            self._last_pwm_send_ts = time.time()
            if not was_paused:
                self.wp_align_active = False
                self.wp_settle_until = 0.0
                self._wp_reach_candidate_idx = -1
                self._wp_reach_candidate_since = 0.0
                self._heading_commit_active = False
                self._turn_in_place_active = False
                self._drive_stuck_since = 0.0
                self._turn_stuck_since = 0.0
                self._recovery_phase = ""
                self._recovery_until = 0.0
                self._recovery_cooldown_until = 0.0
                self._reset_motion_progress_state()
                self.turn_boost_pwm = 0.0
                self.drive_stall_boost_pwm = 0.0
                self._drive_boost_since = 0.0
                self.steer_cmd_filt = 0.0
                self._reset_auto_lidar_avoid_state()
                self.pid_yaw.reset()
                self.pid_speed.reset()
            if self.gps_control_mode == "START_ONLY" and (not was_paused or not self.gps_start_only_active):
                anchor_lat = self.gps_raw_x if (self.gps_raw_x is not None and self.gps_raw_y is not None and not self._gps_invalid(self.gps_raw_x, self.gps_raw_y)) else self.gps_x
                anchor_lon = self.gps_raw_y if (self.gps_raw_x is not None and self.gps_raw_y is not None and not self._gps_invalid(self.gps_raw_x, self.gps_raw_y)) else self.gps_y
                self._reset_dead_reckoning_state(sync_to_gps=False)
                if anchor_lat is not None and anchor_lon is not None:
                    self.gps_x = anchor_lat
                    self.gps_y = anchor_lon
                    self.dr_lat = anchor_lat
                    self.dr_lon = anchor_lon
                self.gps_start_only_active = True
                self.get_logger().warn("🛰️ START_ONLY mission anchor locked")
            elif self.gps_control_mode == "START_ONLY" and was_paused:
                self.get_logger().warn("▶ RESUME: keeping START_ONLY mission anchor")
            else:
                self.gps_start_only_active = False

            if not was_paused:
                if self.web_run_mode == "AUTO" and not self._return_home_active:
                    self.current_idx = 0
                # Record the robot's GPS position as the virtual segment start so
                # cross-track correction works even for the first leg (start → WP1).
                if self.gps_x is not None and self.gps_y is not None:
                    self._auto_start_gps = (self.gps_x, self.gps_y)
                    if not self._return_home_active:
                        self._home_gps = self._auto_start_gps
                else:
                    self._auto_start_gps = None
                if not self._return_home_active and self._home_gps is None and self.path_points:
                    self._home_gps = self.path_points[0]
                self._snap_start_idx_to_nearest_path()
                if (
                    self.AUTO_ALIGN_ON_START
                    and self.web_run_mode == "AUTO"
                    and self.path_points
                    and self.current_idx < len(self.path_points)
                ):
                    self.wp_align_active = True
                    self.wp_settle_until = time.time() + self.wp_stop_settle_s
                    self._reset_turn_direction_state()
                    self.pid_yaw.reset()
                    self.get_logger().warn(
                        f"🧭 AUTO start align to WP {self.current_idx + 1}/{len(self.path_points)}"
                    )

        self.mode_web = cmd

    def _abort_mission_after_emergency(self):
        """Emergency from web is terminal: require a fresh path + START."""
        self.mode_web = "STOP"
        self.web_run_mode = "AUTO"
        self.gps_start_only_active = False
        self._clear_mission_path()
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_cooldown_until = 0.0
        self._reset_turn_direction_state()
        self._reset_motion_progress_state()
        self.turn_boost_pwm = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1
        self._auto_bypass_last_ts = 0.0
        self.obstacle_memory.clear()
        self.obstacle_memory_active = False
        self.steer_cmd_filt = 0.0
        self._auto_steer_mix_prev = 0.0
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self.send_pwm(0, 0)

    def _clear_mission_path(self):
        self.raw_path_points = []
        self.path_points = []
        self.path_corner_indices = set()
        self._base_path_points = []
        self._base_path_corner_indices = set()
        self.current_idx = 0
        self._auto_start_gps = None
        self._home_gps = None
        self._return_home_active = False
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1
        self._auto_bypass_last_ts = 0.0
        self._auto_wp_guard_idx = -1
        self._auto_wp_guard_start_dist = None
        self._auto_wp_guard_start_ts = 0.0
        self._auto_wp_guard_best_dist = None
        self._auto_force_direct_wp_until = 0.0
        self._auto_no_progress_count = 0
        self._reset_turn_direction_state()
        self._reset_motion_progress_state()

    def _reset_dead_reckoning_state(self, sync_to_gps=True):
        self.last_enc_delta = 0.0
        self.last_enc_delta_l = 0.0
        self.last_enc_delta_r = 0.0
        self.speed_mps = 0.0
        self._last_enc_update_ts = time.time()
        self.prev_enc = None
        self._prev_enc_l = None
        self._prev_enc_r = None
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_cooldown_until = 0.0
        self._fusion_last_corr_m = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self._turn_wrong_way_since = 0.0
        self._turn_direction_override_until = 0.0
        self._turn_direction_override_sign = 0.0
        self._turn_latch_sign = 0.0
        self._reset_motion_progress_state()
        if sync_to_gps and self.gps_x is not None and self.gps_y is not None:
            self.dr_lat = self.gps_x
            self.dr_lon = self.gps_y
        else:
            self.dr_lat = None
            self.dr_lon = None

    def _reset_motion_progress_state(self):
        self._path_progress_rate_mps = 0.0
        self._last_progress_dist_to_wp = None
        self._last_progress_idx = -1
        self._auto_wp_guard_idx = -1
        self._auto_wp_guard_start_dist = None
        self._auto_wp_guard_start_ts = 0.0
        self._auto_wp_guard_best_dist = None
        self._auto_force_direct_wp_until = 0.0
        self._auto_no_progress_count = 0
        self._turn_yaw_err_rate = 0.0
        self._last_turn_yaw_err_abs = None

    def _reset_turn_direction_state(self):
        self._turn_wrong_way_since = 0.0
        self._turn_direction_override_until = 0.0
        self._turn_direction_override_sign = 0.0
        self._turn_latch_sign = 0.0
        self._turn_yaw_err_rate = 0.0
        self._last_turn_yaw_err_abs = None
        self.steer_cmd_filt = 0.0

    def resync_gps_state(self):
        self._reset_dead_reckoning_state(sync_to_gps=False)
        if self.gps_raw_x is not None and self.gps_raw_y is not None and not self._gps_invalid(self.gps_raw_x, self.gps_raw_y):
            self.gps_x = self.gps_raw_x
            self.gps_y = self.gps_raw_y
            self.dr_lat = self.gps_x
            self.dr_lon = self.gps_y
            self.gps_from_pixhawk = True
            self.last_gps_time = time.time()
            self._last_gps_update_ts = self.last_gps_time
            self.get_logger().warn("🔄 GPS re-synced to raw fix")
        else:
            self.get_logger().warn("⚠️ GPS re-sync skipped: raw GPS not ready")

    def reset_gps_estimator(self):
        self.gps_calibrating = False
        self.gps_calib_samples.clear()
        self.gps_calib_start_ts = 0.0
        self.gps_ready = False
        self.gps_from_pixhawk = False
        self.gps_x = None
        self.gps_y = None
        self.gps_raw_x = None
        self.gps_raw_y = None
        self.gps_fix = 0
        self.last_gps_time = 0.0
        self._last_gps_update_ts = 0.0
        self._gps_init_spread_m = -1.0
        self._gps_jump_drop_count = 0
        self.gps_init_samples.clear()
        self._reset_dead_reckoning_state(sync_to_gps=False)
        self.get_logger().warn("🔄 GPS estimator refreshed from WEB command")

    def _gps_invalid(self, lat, lon):
        # Many stacks report 0,0 when GPS is not valid yet.
        return abs(lat) < 1e-9 and abs(lon) < 1e-9

    def start_gps_calibration(self):
        self.gps_calibrating = True
        self.gps_calib_samples.clear()
        self.gps_calib_start_ts = time.time()
        self.mode_web = "STOP"
        self.send_pwm(0, 0)
        self.get_logger().warn("🧭 GPS calibration started (vehicle locked)")

    def _process_gps_calibration(self, lat, lon, fix_type):
        now = time.time()
        if not self.gps_calibrating:
            return
        if fix_type >= self.gps_warmup_min_fix and not self._gps_invalid(lat, lon):
            self.gps_calib_samples.append((lat, lon))
        if now - self._last_gps_calib_log_ts > 1.0:
            self._last_gps_calib_log_ts = now
            self.get_logger().info(
                f"🧭 Calibrating GPS... n={len(self.gps_calib_samples)} fix={fix_type}"
            )
        if now - self.gps_calib_start_ts < self.gps_calib_duration_s:
            return

        self.gps_calibrating = False
        if len(self.gps_calib_samples) < self.gps_calib_min_samples:
            self.get_logger().warn(
                f"⚠️ GPS calibration failed: not enough samples ({len(self.gps_calib_samples)})"
            )
            return

        lats = [p[0] for p in self.gps_calib_samples]
        lons = [p[1] for p in self.gps_calib_samples]
        med_lat = statistics.median(lats)
        med_lon = statistics.median(lons)
        max_spread_m = 0.0
        for s_lat, s_lon in self.gps_calib_samples:
            dn, de = self._gps_delta_m(med_lat, med_lon, s_lat, s_lon)
            max_spread_m = max(max_spread_m, math.hypot(dn, de))

        self.gps_x = med_lat
        self.gps_y = med_lon
        self.dr_lat = med_lat
        self.dr_lon = med_lon
        self.gps_from_pixhawk = True
        self.gps_ready = True
        self._gps_init_spread_m = max_spread_m
        self.gps_init_samples.clear()
        self.last_gps_time = now
        self._last_gps_update_ts = now
        self.get_logger().warn(
            f"✅ GPS calibrated: n={len(self.gps_calib_samples)} spread={max_spread_m:.2f}m"
        )

    def _densify_path_points(self, points):
        def raw_corner_indices(raw_points):
            if not raw_points:
                return set()
            if self.AUTO_FORCE_USER_WP_CORNERS:
                return set(range(len(raw_points)))
            corners = {0, len(raw_points) - 1}
            for i in range(1, len(raw_points) - 1):
                prev_lat, prev_lon = raw_points[i - 1]
                cur_lat, cur_lon = raw_points[i]
                next_lat, next_lon = raw_points[i + 1]
                in_n, in_e = self._gps_delta_m(prev_lat, prev_lon, cur_lat, cur_lon)
                out_n, out_e = self._gps_delta_m(cur_lat, cur_lon, next_lat, next_lon)
                in_len = math.hypot(in_n, in_e)
                out_len = math.hypot(out_n, out_e)
                if in_len < self.PATH_DENSIFY_MIN_SEG_M or out_len < self.PATH_DENSIFY_MIN_SEG_M:
                    corners.add(i)
                    continue
                dot = max(-1.0, min(1.0, (in_n * out_n + in_e * out_e) / (in_len * out_len)))
                if math.acos(dot) >= self.WAYPOINT_TURN_ANGLE_RAD:
                    corners.add(i)
            return corners

        if not self.PATH_DENSIFY_ENABLE or len(points) < 2:
            return list(points), raw_corner_indices(points)

        spacing_m = max(0.15, self.PATH_DENSIFY_SPACING_M)
        dense = [points[0]]
        corner_indices = {0}

        for start, end in zip(points, points[1:]):
            s_lat, s_lon = start
            e_lat, e_lon = end
            dn, de = self._gps_delta_m(s_lat, s_lon, e_lat, e_lon)
            seg_len = math.hypot(dn, de)

            if seg_len < self.PATH_DENSIFY_MIN_SEG_M:
                # Still keep the original corner/end point if it is not a true duplicate.
                last_lat, last_lon = dense[-1]
                last_dn, last_de = self._gps_delta_m(last_lat, last_lon, e_lat, e_lon)
                if math.hypot(last_dn, last_de) >= self.PATH_DENSIFY_MIN_SEG_M:
                    dense.append((e_lat, e_lon))
                corner_indices.add(len(dense) - 1)
                continue

            steps = max(1, int(math.ceil(seg_len / spacing_m)))
            for k in range(1, steps + 1):
                t = k / steps
                p = (s_lat + (e_lat - s_lat) * t, s_lon + (e_lon - s_lon) * t)
                last_lat, last_lon = dense[-1]
                last_dn, last_de = self._gps_delta_m(last_lat, last_lon, p[0], p[1])
                if k == steps or math.hypot(last_dn, last_de) >= self.PATH_DENSIFY_MIN_SEG_M:
                    dense.append(p)

            # This index represents an original map waypoint/corner. Only these
            # points get the stop-and-align behavior; dense breadcrumbs flow through.
            corner_indices.add(len(dense) - 1)

        return dense, corner_indices

    def _filter_close_user_points(self, points):
        if len(points) <= 2:
            return list(points)

        min_spacing = max(0.0, self.PATH_MIN_USER_POINT_SPACING_M)
        duplicate_eps = max(0.0, self.PATH_DUPLICATE_POINT_EPS_M)
        if min_spacing <= 0.0:
            return list(points)

        def is_intentional_corner(idx):
            if self.AUTO_FORCE_USER_WP_CORNERS:
                return True
            if idx <= 0 or idx >= len(points) - 1:
                return True
            prev_lat, prev_lon = points[idx - 1]
            cur_lat, cur_lon = points[idx]
            next_lat, next_lon = points[idx + 1]
            in_n, in_e = self._gps_delta_m(prev_lat, prev_lon, cur_lat, cur_lon)
            out_n, out_e = self._gps_delta_m(cur_lat, cur_lon, next_lat, next_lon)
            in_len = math.hypot(in_n, in_e)
            out_len = math.hypot(out_n, out_e)
            if in_len < self.PATH_DENSIFY_MIN_SEG_M or out_len < self.PATH_DENSIFY_MIN_SEG_M:
                return True
            dot = max(-1.0, min(1.0, (in_n * out_n + in_e * out_e) / (in_len * out_len)))
            return math.acos(dot) >= self.WAYPOINT_TURN_ANGLE_RAD

        filtered = [points[0]]
        skipped = 0
        for idx, point in enumerate(points[1:], start=1):
            last = filtered[-1]
            dn, de = self._gps_delta_m(last[0], last[1], point[0], point[1])
            dist_m = math.hypot(dn, de)
            is_final = idx == len(points) - 1
            if dist_m < duplicate_eps and not is_final:
                skipped += 1
                continue
            if dist_m < min_spacing and not is_final and not is_intentional_corner(idx):
                skipped += 1
                continue
            if dist_m < max(duplicate_eps, min_spacing * 0.35) and is_final and len(filtered) > 1:
                filtered[-1] = point
                skipped += 1
                continue
            filtered.append(point)

        if skipped > 0:
            self.get_logger().warn(
                f"⚠️ Path upload skipped {skipped} point(s) closer than {min_spacing:.2f}m"
            )
        return filtered

    def _is_corner_waypoint(self, idx):
        return idx in self.path_corner_indices

    def _snap_start_idx_to_nearest_path(self):
        """Start AUTO near the closest segment instead of blindly chasing WP1."""
        if not self.AUTO_START_SNAP_ENABLE:
            return
        if self.gps_x is None or self.gps_y is None or len(self.path_points) < 2:
            return
        if self.current_idx != 0:
            return

        first_lat, first_lon = self.path_points[0]
        first_n, first_e = self.meter_error_to_waypoint(first_lat, first_lon)
        first_dist = math.hypot(first_n, first_e)

        best_idx = 0
        best_cross = float("inf")
        best_along = 0.0
        for i in range(len(self.path_points) - 1):
            a_lat, a_lon = self.path_points[i]
            b_lat, b_lon = self.path_points[i + 1]
            seg_n, seg_e = self._gps_delta_m(a_lat, a_lon, b_lat, b_lon)
            seg_len = math.hypot(seg_n, seg_e)
            if seg_len < 0.25:
                continue
            rob_n, rob_e = self._gps_delta_m(a_lat, a_lon, self.gps_x, self.gps_y)
            along = max(0.0, min(seg_len, (rob_n * seg_n + rob_e * seg_e) / seg_len))
            cross = abs(rob_n * seg_e - rob_e * seg_n) / seg_len
            if cross < best_cross:
                best_cross = cross
                best_along = along
                best_idx = i

        should_snap = (
            best_cross <= self.AUTO_START_SNAP_MAX_CROSSTRACK_M
            and (
                (first_dist - best_cross) >= self.AUTO_START_SNAP_MIN_GAIN_M
                or best_along >= self.AUTO_START_SNAP_MIN_ALONG_M
            )
        )
        if not should_snap:
            return

        # Target the next breadcrumb on that segment. If already near the segment
        # end, advance once more so it does not pivot back to a point behind it.
        new_idx = min(len(self.path_points) - 1, best_idx + 1)
        seg_a = self.path_points[best_idx]
        seg_b = self.path_points[best_idx + 1]
        seg_n, seg_e = self._gps_delta_m(seg_a[0], seg_a[1], seg_b[0], seg_b[1])
        seg_len = max(0.001, math.hypot(seg_n, seg_e))
        if best_along > (seg_len * 0.75) and new_idx < (len(self.path_points) - 1):
            new_idx += 1

        self.current_idx = new_idx
        self.wp_align_active = False
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self._reset_motion_progress_state()
        self.pid_yaw.reset()
        self.steer_cmd_filt = 0.0
        # Sync DR to GPS so accumulated encoder distance doesn't cascade-handoff
        # past multiple dense sub-waypoints the instant we snap.
        if self.gps_x is not None and self.gps_y is not None:
            self.dr_lat = self.gps_x
            self.dr_lon = self.gps_y
        self.get_logger().warn(
            f"🎯 AUTO start snapped to path idx {self.current_idx + 1}/{len(self.path_points)} "
            f"xtrk={best_cross:.2f}m first={first_dist:.2f}m"
        )

    def waypoint_cb(self, msg):
        if len(msg.data) % 2 != 0:
            self.get_logger().warn(f"⚠️ Invalid waypoint length: {len(msg.data)} (must be even)")
            return

        uploaded_points = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
        self.raw_path_points = self._filter_close_user_points(uploaded_points)
        self.path_points, self.path_corner_indices = self._densify_path_points(self.raw_path_points)
        self._base_path_points = list(self.path_points)
        self._base_path_corner_indices = set(self.path_corner_indices)
        self.current_idx = 0
        self._auto_start_gps = None
        self._home_gps = None
        self._return_home_active = False
        self.gps_start_only_active = False
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_cooldown_until = 0.0
        self.turn_boost_pwm = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1
        self._auto_bypass_last_ts = 0.0
        self.obstacle_memory.clear()
        self.obstacle_memory_active = False
        self.obstacle_memory_min_m = float('inf')
        self.obstacle_memory_turn = 0.0
        self.steer_cmd_filt = 0.0
        # Uploading new path must not auto-run from stale START state.
        # Operator must press START explicitly after upload.
        self.mode_web = "STOP"
        self.send_pwm(0, 0)
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self.get_logger().info(
            f"▶ WEB CMD: WAYPOINT raw={len(self.raw_path_points)} dense={len(self.path_points)} "
            f"spacing={self.PATH_DENSIFY_SPACING_M:.2f}m corners={len(self.path_corner_indices)}"
        )

    def return_home(self):
        home = self._home_gps
        if home is None and self._auto_start_gps is not None:
            home = self._auto_start_gps
        if home is None and self.raw_path_points:
            home = self.raw_path_points[0]
        if home is None:
            self.get_logger().warn("🏠 RETURN_HOME ignored: no home point recorded yet")
            self.send_pwm(0, 0)
            return

        if self.gps_x is None or self.gps_y is None or not self.gps_ready:
            self.get_logger().warn("🏠 RETURN_HOME ignored: GPS not ready")
            self.send_pwm(0, 0)
            return

        self._home_gps = home
        self.raw_path_points = [home]
        self.path_points = [home]
        self.path_corner_indices = {0}
        self._base_path_points = list(self.path_points)
        self._base_path_corner_indices = set(self.path_corner_indices)
        self.current_idx = 0
        self.web_run_mode = "AUTO"
        self.mode_web = "START"
        self.gps_start_only_active = False
        self._return_home_active = True
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_cooldown_until = 0.0
        self._reset_motion_progress_state()
        self.turn_boost_pwm = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1
        self._auto_start_gps = (self.gps_x, self.gps_y)
        self.steer_cmd_filt = 0.0
        self._auto_steer_mix_prev = 0.0
        self._reset_turn_direction_state()
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self.send_pwm(0, 0)
        self.get_logger().warn(
            f"🏠 RETURN_HOME: target lat={home[0]:.7f} lon={home[1]:.7f} safety={'ON' if self.lidar_safety_enabled else 'OFF'}"
        )

    def safety_distance_cb(self, msg):
        cm = float(msg.data)
        cm = max(0.0, min(400.0, cm))
        self.safety_distance_cm = cm
        self.safety_distance_m = cm / 100.0
        self.lidar_hard_pivot_m = max(self.lidar_hard_pivot_m, self.front_stop_distance_m)
        self.lidar_corridor_stop_m = max(
            self.lidar_corridor_stop_m,
            self.safety_distance_m,
            self.lidar_hard_pivot_m,
            self.front_stop_distance_m,
        )
        self.get_logger().info(f"🛡️ Safety distance set: {self.safety_distance_cm:.0f} cm")

    def yaw_offset_cb(self, msg):
        # Disabled by request: keep direct Pixhawk yaw without any +/- compensation.
        self.yaw_offset_deg = 0.0
        self.yaw_offset_rad = 0.0
        self.get_logger().warn("🧭 Ignored /yaw_offset_deg (yaw compensation disabled)")

    def yaw_offset_adjust_cb(self, msg):
        # Disabled by request: keep direct Pixhawk yaw without any +/- compensation.
        self.yaw_offset_deg = 0.0
        self.yaw_offset_rad = 0.0
        self.get_logger().warn("🧭 Ignored /yaw_offset_adjust_deg (yaw compensation disabled)")

    def _load_persisted_yaw_offset(self):
        # Disabled by request.
        return

    def _save_persisted_yaw_offset(self):
        # Disabled by request.
        return

    def lidar_safety_enable_cb(self, msg):
        self.lidar_safety_enabled = bool(msg.data)
        state = "ON" if self.lidar_safety_enabled else "OFF"
        self.get_logger().info(f"🛡️ LIDAR safety {state}")

    def lidar_max_use_cb(self, msg):
        v = float(msg.data)
        if not math.isfinite(v):
            return
        # 0.0 => unlimited (use LiDAR range_max)
        if v < 0.0:
            v = 0.0
        self.lidar_max_use_m = v
        label = "MAX(range_max)" if self.lidar_max_use_m <= 0.0 else f"{self.lidar_max_use_m:.2f}m"
        self.get_logger().info(f"🛰️ LIDAR detect range: {label}")

    def camera_lane_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        now = time.time()
        ok = bool(data.get("ok", False))
        conf = float(data.get("confidence", 0.0))
        bias = float(data.get("bias_rad", 0.0))
        if not math.isfinite(conf) or not math.isfinite(bias):
            return
        self.camera_lane_ok = ok
        self.camera_lane_conf = max(0.0, min(1.0, conf))
        self.camera_lane_bias_rad = max(
            -self.camera_lane_max_bias_rad,
            min(self.camera_lane_max_bias_rad, bias)
        )
        self.camera_lane_ts = now
        self.camera_lane_debug = data

    def _camera_lane_heading_bias(self, obstacle_avoid_active):
        if not self.camera_lane_assist_enable:
            return 0.0, "CAM_OFF"
        now = time.time()
        if not self.camera_lane_ok or (now - self.camera_lane_ts) > self.camera_lane_max_age_s:
            return 0.0, "CAM_STALE"
        if self.camera_lane_conf < self.camera_lane_min_conf:
            return 0.0, "CAM_LOW"
        bias = self.camera_lane_bias_rad
        src = "CAM"
        if obstacle_avoid_active:
            # LiDAR remains the safety authority. Camera can still softly prefer
            # the orchard row direction, but it cannot overrule obstacle escape.
            bias *= max(0.0, min(1.0, self.camera_lane_obstacle_scale))
            src = "CAM_SOFT"
        return bias, src

    def _load_footprint_polygon(self):
        """Top-view collision polygon in robot frame: x forward, y left."""
        default_poly = [
            [0.98, 0.42],   # front cutting/deck edge
            [0.56, 0.42],
            [0.42, 0.34],   # body narrows behind deck
            [-0.78, 0.34],
            [-0.78, -0.34],
            [0.42, -0.34],
            [0.56, -0.42],
            [0.98, -0.42],
        ]
        raw = os.getenv("AUTO_FOOTPRINT_POLYGON_JSON", "").strip()
        poly = default_poly
        if raw:
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, list) and len(parsed) >= 3:
                    poly = parsed
            except Exception as e:
                self.get_logger().warn(f"⚠️ Invalid AUTO_FOOTPRINT_POLYGON_JSON, using default: {e}")

        clean = []
        for p in poly:
            try:
                x = float(p[0])
                y = float(p[1])
                if math.isfinite(x) and math.isfinite(y):
                    clean.append((x, y))
            except Exception:
                continue
        if len(clean) < 3:
            clean = [(float(x), float(y)) for x, y in default_poly]
        return clean

    def _point_in_polygon(self, x, y, poly):
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            xi, yi = poly[i]
            xj, yj = poly[j]
            if ((yi > y) != (yj > y)):
                x_cross = (xj - xi) * (y - yi) / max(1e-9, (yj - yi)) + xi
                if x < x_cross:
                    inside = not inside
            j = i
        return inside

    def _point_segment_distance_m(self, px, py, ax, ay, bx, by):
        abx = bx - ax
        aby = by - ay
        denom = abx * abx + aby * aby
        if denom <= 1e-9:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * abx + (py - ay) * aby) / denom
        t = max(0.0, min(1.0, t))
        cx = ax + t * abx
        cy = ay + t * aby
        return math.hypot(px - cx, py - cy)

    def _point_hits_footprint(self, x, y, buffer_m=None):
        if not self.footprint_collision_enable:
            half_width = max(0.20, (self.robot_width_m * 0.5) + self.lidar_footprint_buffer_m)
            return (-0.80 <= x <= 1.00) and abs(y) <= half_width
        poly = self.footprint_polygon
        if self._point_in_polygon(x, y, poly):
            return True
        buf = self.footprint_collision_buffer_m if buffer_m is None else float(buffer_m)
        if buf <= 0.0:
            return False
        for i in range(len(poly)):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % len(poly)]
            if self._point_segment_distance_m(x, y, ax, ay, bx, by) <= buf:
                return True
        return False

    def _path_footprint_collision_distance(self, points, theta, lookahead, step=None):
        """Sweep the collision polygon along a candidate heading."""
        c = math.cos(theta)
        s = math.sin(theta)
        sample_step = max(0.08, min(0.35, step if step is not None else self.local_costmap_sample_step_m))
        best_collision = float('inf')
        side_pressure = 0.0
        d = 0.0
        while d <= lookahead:
            for forward_m, left_m, _, _ in points:
                # Transform point into candidate-path coordinates, then into
                # the robot footprint centered at a future pose d meters ahead.
                along = forward_m * c + left_m * s
                lateral = -forward_m * s + left_m * c
                if along < (d - 1.15) or along > (d + 1.20):
                    continue
                rel_x = along - d
                rel_y = lateral
                if self._point_hits_footprint(rel_x, rel_y):
                    best_collision = min(best_collision, d)
                else:
                    # Soft pressure for points near the polygon; keeps it from
                    # choosing a path that barely scrapes rows of trees.
                    near = abs(rel_y) - ((self.robot_width_m * 0.5) + self.footprint_collision_buffer_m)
                    if -0.35 <= rel_x <= 1.05 and near < 0.55:
                        side_pressure += 1.0 / max(0.18, near + 0.58)
            if math.isfinite(best_collision):
                break
            d += sample_step
        return best_collision, side_pressure

    def _orchard_candidate_angles(self):
        max_deg = max(15.0, min(85.0, self.orchard_candidate_max_deg))
        step_deg = max(3.0, min(20.0, self.orchard_candidate_step_deg))
        angles = [0.0]
        n = int(math.ceil(max_deg / step_deg))
        for i in range(1, n + 1):
            deg = min(max_deg, i * step_deg)
            angles.extend([math.radians(deg), math.radians(-deg)])
        return angles

    def _score_orchard_candidate(self, theta, clear_m, blocked, side_pressure, lookahead):
        clear_ratio = max(0.0, min(1.0, clear_m / max(0.1, lookahead)))
        score = self.orchard_gap_score_bias * clear_ratio
        if not blocked:
            score += 4.5
        # Prefer staying near the path heading unless an actual gap forces a side move.
        score += self.orchard_path_bias * max(0.0, math.cos(theta))
        score -= 0.22 * (abs(theta) / max(math.radians(1.0), self.lidar_avoid_bias_max))
        score -= 0.018 * side_pressure
        if self.lidar_avoid_commit_sign != 0.0 and abs(theta) > math.radians(2.0):
            sign = 1.0 if theta > 0.0 else -1.0
            if sign != self.lidar_avoid_commit_sign:
                score -= self.orchard_side_change_penalty
        if clear_m < self.orchard_min_clear_ahead_m:
            score -= (self.orchard_min_clear_ahead_m - clear_m) * 0.65
        return score

    def _plan_lidar_footprint(self, points, lookahead_m):
        self.lidar_footprint_points = list(points)
        self.lidar_footprint_best_angle = 0.0
        self.lidar_footprint_best_clear_m = float('inf')
        self.lidar_footprint_blocked = False
        self.lidar_footprint_min_collision_m = float('inf')
        if not self.lidar_footprint_enable:
            return

        lookahead = max(self.safety_distance_m, min(max(0.8, lookahead_m), self.lidar_footprint_lookahead_m))
        if self.lidar_raw_hard_stop_enable and math.isfinite(self.lidar_raw_hard_stop_m):
            lookahead = max(lookahead, min(self.front_stop_distance_m, self.lidar_raw_hard_stop_m + 0.20))
        half_width = max(0.20, (self.robot_width_m * 0.5) + self.lidar_footprint_buffer_m)
        candidates = self._orchard_candidate_angles() if self.orchard_avoid_enable else [
            math.radians(d) for d in [0, 18, -18, 36, -36, 55, -55, 72, -72]
        ]
        best_score = -1e9
        best_angle = 0.0
        best_clear = 0.0
        best_blocked = True
        best_collision = float('inf')

        for theta in candidates:
            if self.footprint_collision_enable:
                min_collision, side_pressure = self._path_footprint_collision_distance(points, theta, lookahead)
            else:
                c = math.cos(theta)
                s = math.sin(theta)
                min_collision = float('inf')
                side_pressure = 0.0

                for forward_m, left_m, _, _ in points:
                    along = forward_m * c + left_m * s
                    lateral = -forward_m * s + left_m * c
                    if along < -0.10 or along > lookahead:
                        continue
                    lateral_clear = abs(lateral) - half_width
                    if lateral_clear <= 0.0:
                        min_collision = min(min_collision, max(0.0, along))
                    elif along > 0.0:
                        side_pressure += 1.0 / max(0.15, lateral_clear + 0.25 * along)

            blocked = math.isfinite(min_collision)
            clear_m = min_collision if blocked else lookahead
            if self.orchard_avoid_enable:
                score = self._score_orchard_candidate(theta, clear_m, blocked, side_pressure, lookahead)
            else:
                # Prefer non-colliding paths, then forward-ish paths, while still
                # allowing a strong side choice when the body corridor is blocked.
                score = clear_m
                if not blocked:
                    score += 4.0
                score += 0.55 * math.cos(theta)
                score -= 0.012 * abs(math.degrees(theta))
                score -= 0.03 * side_pressure

            if score > best_score:
                best_score = score
                best_angle = theta
                best_clear = clear_m
                best_blocked = blocked
                best_collision = min_collision

        self.lidar_footprint_best_angle = best_angle
        self.lidar_footprint_best_clear_m = best_clear
        self.lidar_footprint_blocked = best_blocked
        self.lidar_footprint_min_collision_m = best_collision

        # If the straight body corridor is already blocked, make the chosen
        # direction visible to the old corridor planner too.
        straight_collision = float('inf')
        for forward_m, left_m, _, _ in points:
            if self.footprint_collision_enable:
                hit = 0.0 <= forward_m <= lookahead and self._point_hits_footprint(forward_m, left_m)
            else:
                hit = 0.0 <= forward_m <= lookahead and abs(left_m) <= half_width
            if hit:
                straight_collision = min(straight_collision, forward_m)
        if math.isfinite(straight_collision):
            self.lidar_footprint_blocked = True
            self.lidar_footprint_min_collision_m = min(self.lidar_footprint_min_collision_m, straight_collision)

    def _memory_points_robot_frame(self):
        if (
            not self.obstacle_memory_enable
            or self.gps_x is None
            or self.gps_y is None
        ):
            return []

        now = time.time()
        self._purge_obstacle_memory(now)
        size_m = max(2.0, self.local_costmap_size_m)
        back_m = max(0.0, min(size_m * 0.45, self.local_costmap_back_m))
        ahead_m = max(0.5, size_m - back_m)
        half_side_m = size_m * 0.5
        points = []
        for p in self.obstacle_memory:
            d_north, d_east = self._gps_delta_m(self.gps_x, self.gps_y, p["lat"], p["lon"])
            forward_m, left_m = self._world_to_robot_local_m(d_north, d_east)
            if -back_m <= forward_m <= ahead_m and abs(left_m) <= half_side_m:
                r = math.hypot(forward_m, left_m)
                a = math.atan2(left_m, forward_m)
                points.append((forward_m, left_m, r, a))
        return points

    def _update_local_costmap(self, live_points):
        self.local_costmap_active = False
        self.local_costmap_blocked = False
        self.local_costmap_best_angle = 0.0
        self.local_costmap_best_clear_m = float('inf')
        self.local_costmap_min_collision_m = float('inf')
        self.local_costmap_grid = []
        self.local_costmap_points = []
        if not self.local_costmap_enable:
            return

        size_m = max(2.0, self.local_costmap_size_m)
        res = max(0.05, min(0.50, self.local_costmap_resolution_m))
        cells = max(16, min(180, int(round(size_m / res))))
        back_m = max(0.0, min(size_m * 0.45, self.local_costmap_back_m))
        ahead_m = max(0.5, size_m - back_m)
        half_side_m = size_m * 0.5
        inflate_cells = max(1, int(math.ceil(max(0.05, self.local_costmap_inflate_m) / res)))

        points = list(live_points or [])
        points.extend(self._memory_points_robot_frame())
        points = [
            p for p in points
            if -back_m <= p[0] <= ahead_m and abs(p[1]) <= half_side_m
        ]
        self.local_costmap_points = points
        if not points:
            return

        occ = [[0] * cells for _ in range(cells)]

        def to_cell(forward_m, left_m):
            gx = int((forward_m + back_m) / res)
            gy = int((left_m + half_side_m) / res)
            if gx < 0 or gx >= cells or gy < 0 or gy >= cells:
                return None
            return gx, gy

        for forward_m, left_m, _, _ in points:
            cell = to_cell(forward_m, left_m)
            if cell is None:
                continue
            cx, cy = cell
            x0 = max(0, cx - inflate_cells)
            x1 = min(cells - 1, cx + inflate_cells)
            y0 = max(0, cy - inflate_cells)
            y1 = min(cells - 1, cy + inflate_cells)
            for gx in range(x0, x1 + 1):
                dx = (gx - cx) * res
                for gy in range(y0, y1 + 1):
                    dy = (gy - cy) * res
                    if (dx * dx + dy * dy) <= (self.local_costmap_inflate_m * self.local_costmap_inflate_m):
                        occ[gx][gy] = 100

        self.local_costmap_grid = occ

        def occupied(forward_m, left_m):
            cell = to_cell(forward_m, left_m)
            if cell is None:
                return False
            return occ[cell[0]][cell[1]] >= 50

        lookahead = max(self.safety_distance_m, min(ahead_m, self.local_costmap_lookahead_m))
        step = max(0.06, min(0.50, self.local_costmap_sample_step_m))
        candidates = self._orchard_candidate_angles() if self.orchard_avoid_enable else [
            math.radians(d) for d in [0, 10, -10, 22, -22, 38, -38, 55, -55, 72, -72]
        ]
        best_score = -1e9
        best_angle = 0.0
        best_clear = 0.0
        best_blocked = True
        best_collision = float('inf')

        for theta in candidates:
            if self.footprint_collision_enable:
                collision, side_cost = self._path_footprint_collision_distance(points, theta, lookahead, step)
            else:
                collision = float('inf')
                side_cost = 0.0
                d = 0.0
                while d <= lookahead:
                    f = d * math.cos(theta)
                    l = d * math.sin(theta)
                    if occupied(f, l):
                        collision = d
                        break
                    # small cost for brushing near non-inflated points
                    for pf, pl, _, _ in points:
                        df = pf - f
                        dl = pl - l
                        dist = math.hypot(df, dl)
                        if dist < (self.local_costmap_inflate_m + 0.45):
                            side_cost += 1.0 / max(0.20, dist)
                    d += step

            blocked = math.isfinite(collision)
            clear_m = collision if blocked else lookahead
            if self.orchard_avoid_enable:
                score = self._score_orchard_candidate(theta, clear_m, blocked, side_cost, lookahead)
            else:
                score = clear_m
                if not blocked:
                    score += 4.0
                score += 0.45 * math.cos(theta)
                score -= 0.010 * abs(math.degrees(theta))
                score -= 0.006 * side_cost
            if score > best_score:
                best_score = score
                best_angle = theta
                best_clear = clear_m
                best_blocked = blocked
                best_collision = collision

        self.local_costmap_active = True
        self.local_costmap_best_angle = best_angle
        self.local_costmap_best_clear_m = best_clear
        self.local_costmap_blocked = bool(
            occupied(0.0, 0.0)
            or (best_blocked and best_clear < max(self.orchard_no_gap_stop_m, self.safety_distance_m))
        )
        self.local_costmap_min_collision_m = best_collision

    def lidar_cb(self, msg):
        half_sector = math.radians(self.lidar_front_sector_deg * 0.5)
        raw_min_front = float('inf')
        min_front = float('inf')
        front_threat = 0.0
        left_threat = 0.0
        right_threat = 0.0
        close_points = 0
        close_cluster = 0
        close_beams = []
        center_threat = 0.0
        center_left_threat = 0.0
        center_right_threat = 0.0
        angle_step = abs(float(msg.angle_increment)) if msg.angle_increment else 0.0
        avoid_distance_m = max(self.safety_distance_m, self.lidar_avoid_distance_m)
        escape_front_rad = math.radians(max(5.0, min(90.0, self.lidar_escape_front_deg)))
        corridor_n = max(5, int(self.lidar_corridor_sectors))
        corridor_clear = [float('inf')] * corridor_n
        corridor_threat = [0.0] * corridor_n
        footprint_points = []
        raw_hard_min = float('inf')
        raw_hard_hits = 0
        raw_hard_half_width = max(0.20, (self.robot_width_m * 0.5) + self.lidar_footprint_buffer_m)
        raw_hard_lookahead = max(self.safety_distance_m, self.front_stop_distance_m)

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if self.lidar_max_use_m > 0.0 and r > self.lidar_max_use_m:
                continue

            a_raw = msg.angle_min + i * msg.angle_increment
            # Align LiDAR frame to robot-forward (fixes front/back reversed mounting).
            a = normalize_angle(a_raw + self.lidar_angle_offset_rad)
            # Use front sector only (default 240 deg = +/-120 deg)
            if abs(a) > half_sector:
                continue

            raw_min_front = min(raw_min_front, r)
            forward_m = r * math.cos(a)
            left_m = r * math.sin(a)
            if self.footprint_collision_enable:
                collision_d, _ = self._path_footprint_collision_distance(
                    [(forward_m, left_m, r, a)],
                    0.0,
                    raw_hard_lookahead,
                    self.local_costmap_sample_step_m,
                )
                raw_hard_hit = (
                    self.lidar_raw_hard_stop_enable
                    and math.isfinite(collision_d)
                )
            else:
                raw_hard_hit = (
                    self.lidar_raw_hard_stop_enable
                    and 0.0 <= forward_m <= raw_hard_lookahead
                    and abs(left_m) <= raw_hard_half_width
                )
            if raw_hard_hit:
                raw_hard_hits += 1
                raw_hard_min = min(raw_hard_min, forward_m)
            if r < avoid_distance_m:
                closeness = (avoid_distance_m - r) / max(avoid_distance_m, 1e-6)
                front_weight = max(0.0, math.cos(a))
                threat = closeness * (0.3 + 0.7 * front_weight)
                close_beams.append((i, r, a, threat))

        def corridor_idx(angle_rad):
            ratio = (angle_rad + half_sector) / max(1e-6, 2.0 * half_sector)
            return max(0, min(corridor_n - 1, int(ratio * corridor_n)))

        def commit_cluster(cluster):
            nonlocal min_front, front_threat, left_threat, right_threat, close_points, close_cluster
            nonlocal center_threat, center_left_threat, center_right_threat
            if not cluster:
                return

            ranges = sorted(p[1] for p in cluster)
            mid = len(ranges) // 2
            median_r = ranges[mid] if len(ranges) % 2 else 0.5 * (ranges[mid - 1] + ranges[mid])
            span_rad = max(angle_step, abs(cluster[-1][0] - cluster[0][0]) * angle_step)
            physical_width_m = median_r * span_rad

            # Dust/grass usually appears as one or two beams, or a tiny close
            # speck. Only accept a blob that has enough angular width to be a
            # real obstacle/person.
            if (
                len(cluster) < self.lidar_min_cluster_points
                or physical_width_m < self.lidar_min_cluster_width_m
            ):
                return

            self._remember_lidar_cluster(cluster, median_r)
            close_points += len(cluster)
            close_cluster = max(close_cluster, len(cluster))
            min_front = min(min_front, min(p[1] for p in cluster))

            for _, _, a, threat in cluster:
                front_threat += threat
                ci = corridor_idx(a)
                corridor_threat[ci] += threat
                if self.lidar_swap_lr:
                    if a >= 0.0:
                        right_threat += threat
                        if abs(a) <= escape_front_rad:
                            center_right_threat += threat
                    else:
                        left_threat += threat
                        if abs(a) <= escape_front_rad:
                            center_left_threat += threat
                else:
                    if a >= 0.0:
                        left_threat += threat
                        if abs(a) <= escape_front_rad:
                            center_left_threat += threat
                    else:
                        right_threat += threat
                        if abs(a) <= escape_front_rad:
                            center_right_threat += threat
                if abs(a) <= escape_front_rad:
                    center_threat += threat
            for _, r, a, _ in cluster:
                footprint_points.append((r * math.cos(a), r * math.sin(a), r, a))
                ci = corridor_idx(a)
                corridor_clear[ci] = min(corridor_clear[ci], r)

        cluster = []
        prev = None
        for beam in close_beams:
            if prev is not None:
                idx_gap = beam[0] - prev[0]
                range_jump = abs(beam[1] - prev[1])
                max_jump = max(self.lidar_cluster_max_range_jump_m, 0.25 * min(beam[1], prev[1]))
                same_blob = idx_gap <= self.lidar_cluster_max_gap and range_jump <= max_jump
            else:
                same_blob = True

            if same_blob:
                cluster.append(beam)
            else:
                commit_cluster(cluster)
                cluster = [beam]
            prev = beam
        commit_cluster(cluster)

        self.lidar_min_front_m = min_front
        self.lidar_front_threat = front_threat
        self.lidar_left_threat = left_threat
        self.lidar_right_threat = right_threat
        self.lidar_close_points = close_points
        self.lidar_close_cluster = close_cluster
        self.lidar_raw_hard_stop_m = raw_hard_min
        self.lidar_raw_hard_stop_active = (
            self.lidar_raw_hard_stop_enable
            and raw_hard_hits >= self.lidar_raw_hard_stop_points
            and math.isfinite(raw_hard_min)
        )
        self._plan_lidar_footprint(footprint_points, avoid_distance_m)
        self._update_local_costmap(footprint_points)

        # Pick the safest local corridor. A sector with no close cluster is
        # treated as clear to avoid_distance_m. The score prefers forward gaps
        # but will choose side gaps when the center is blocked by a real cluster.
        self.lidar_corridor_sector_clear = list(corridor_clear)
        self.lidar_corridor_blocked = False
        self.lidar_corridor_best_angle = 0.0
        self.lidar_corridor_best_clear_m = avoid_distance_m
        if self.lidar_corridor_enable:
            best_score = -1e9
            best_angle = 0.0
            best_clear = 0.0
            viable = False
            for ci in range(corridor_n):
                frac = (ci + 0.5) / corridor_n
                angle = -half_sector + frac * (2.0 * half_sector)
                clear_m = corridor_clear[ci] if math.isfinite(corridor_clear[ci]) else avoid_distance_m
                front_preference = max(0.0, math.cos(angle))
                side_penalty = abs(angle) / max(half_sector, 1e-6)
                score = (
                    (clear_m / max(avoid_distance_m, 1e-6))
                    + self.lidar_corridor_prefer_front * front_preference
                    - 0.35 * side_penalty
                    - 0.08 * corridor_threat[ci]
                )
                if clear_m >= self.lidar_corridor_min_clear_m:
                    viable = True
                if score > best_score:
                    best_score = score
                    best_angle = angle
                    best_clear = clear_m

            self.lidar_corridor_best_angle = best_angle
            self.lidar_corridor_best_clear_m = best_clear
            self.lidar_corridor_blocked = bool(close_points > 0 and not viable)
            if self.lidar_footprint_enable and footprint_points:
                self.lidar_corridor_best_angle = self.lidar_footprint_best_angle
                self.lidar_corridor_best_clear_m = min(best_clear, self.lidar_footprint_best_clear_m)
                self.lidar_corridor_blocked = self.lidar_corridor_blocked or self.lidar_footprint_blocked
            if self.local_costmap_enable and self.local_costmap_active:
                self.lidar_corridor_best_angle = self.local_costmap_best_angle
                self.lidar_corridor_best_clear_m = min(self.lidar_corridor_best_clear_m, self.local_costmap_best_clear_m)
                self.lidar_corridor_blocked = self.lidar_corridor_blocked or self.local_costmap_blocked

        now = time.time()
        # Outdoor dust/grass can create single-frame close spikes. Require a small
        # cluster and a short hold before making safety active, then hold clear a
        # bit longer to avoid chatter.
        candidate = (
            math.isfinite(min_front)
            and min_front < max(0.0, max(self.safety_distance_m, self.lidar_avoid_distance_m) - self.lidar_noise_margin_m)
            and close_points >= self.lidar_min_close_points
            and close_cluster >= self.lidar_min_close_cluster
            and front_threat >= self.lidar_min_threat
        ) or self.lidar_raw_hard_stop_active
        clear_radius_m = max(self.safety_distance_m, self.lidar_avoid_distance_m) + max(0.0, self.lidar_avoid_clear_margin_m)
        clear_enough = (
            (not math.isfinite(min_front))
            or (
                min_front >= clear_radius_m
                and (close_points == 0 or close_cluster < self.lidar_min_close_cluster or front_threat < (self.lidar_min_threat * 0.35))
            )
        )
        if candidate:
            if not self.lidar_obstacle_candidate:
                self.lidar_obstacle_candidate_since = now
            self.lidar_obstacle_candidate = True
            self.lidar_clear_candidate_since = 0.0
        else:
            self.lidar_obstacle_candidate = False
            self.lidar_obstacle_candidate_since = 0.0
            if self.lidar_obstacle_active and self.lidar_clear_candidate_since <= 0.0:
                self.lidar_clear_candidate_since = now

        if self.lidar_obstacle_active:
            # Stay in avoid mode until the obstacle has left the configured
            # clearance radius for long enough. This prevents snapping back to
            # the yellow path and brushing past a person/object.
            if clear_enough and self.lidar_clear_candidate_since > 0.0 and (now - self.lidar_clear_candidate_since) >= self.lidar_obstacle_exit_s:
                self.lidar_obstacle_active = False
                self.lidar_confirmed_min_front_m = float('inf')
                self.lidar_avoid_commit_until = now + self.lidar_avoid_hold_s
        else:
            if candidate and (now - self.lidar_obstacle_candidate_since) >= self.lidar_obstacle_enter_s:
                self.lidar_obstacle_active = True
                self.lidar_confirmed_min_front_m = min_front

        if self.lidar_obstacle_active and math.isfinite(min_front):
            self.lidar_confirmed_min_front_m = min_front
        if self.lidar_raw_hard_stop_active and math.isfinite(raw_hard_min):
            self.lidar_confirmed_min_front_m = min(self.lidar_confirmed_min_front_m, raw_hard_min)

        # Positive = turn left, Negative = turn right. Once a real obstacle is
        # in front, pick one escape side and hold it. Without this latch a human
        # centered in front makes left/right clearances alternate and the mower
        # wiggles like a follow-me tracker instead of avoiding.
        turn_raw = (right_threat - left_threat)
        escape_needed = (
            (candidate or self.lidar_obstacle_active)
            and center_threat >= max(self.lidar_min_threat * 0.35, 0.08)
        )
        if escape_needed:
            if self.lidar_avoid_commit_sign != 0.0 and now < self.lidar_avoid_commit_until:
                escape_sign = self.lidar_avoid_commit_sign
            else:
                diff = center_right_threat - center_left_threat
                if abs(diff) >= self.lidar_escape_threat_bias:
                    escape_sign = 1.0 if diff > 0.0 else -1.0
                elif abs(self.lidar_corridor_best_angle) > math.radians(4.0):
                    escape_sign = 1.0 if self.lidar_corridor_best_angle > 0.0 else -1.0
                elif abs(turn_raw) > self.lidar_escape_threat_bias:
                    escape_sign = 1.0 if turn_raw > 0.0 else -1.0
                else:
                    escape_sign = 1.0
            desired_turn = escape_sign * max(self.lidar_escape_min_turn_rad, abs(self.lidar_corridor_best_angle))
            desired_turn = max(-self.lidar_avoid_bias_max, min(self.lidar_avoid_bias_max, desired_turn))
        elif self.lidar_corridor_enable and (candidate or self.lidar_obstacle_active):
            desired_turn = max(
                -self.lidar_avoid_bias_max,
                min(self.lidar_avoid_bias_max, self.lidar_corridor_best_angle)
            )
        else:
            desired_turn = max(-self.lidar_avoid_bias_max, min(self.lidar_avoid_bias_max, 1.2 * turn_raw))
        desired_sign = 0.0
        if desired_turn > math.radians(2.0):
            desired_sign = 1.0
        elif desired_turn < -math.radians(2.0):
            desired_sign = -1.0

        if (self.lidar_obstacle_active or escape_needed) and desired_sign != 0.0:
            if self.lidar_avoid_commit_sign == 0.0:
                self.lidar_avoid_commit_sign = desired_sign
                self.lidar_avoid_commit_until = now + self.lidar_avoid_hold_s
            elif desired_sign != self.lidar_avoid_commit_sign:
                if now < self.lidar_avoid_commit_until:
                    desired_turn = self.lidar_avoid_commit_sign * max(abs(desired_turn), math.radians(4.0))
                else:
                    self.lidar_avoid_commit_sign = desired_sign
                    self.lidar_avoid_commit_until = now + self.lidar_avoid_hold_s
            else:
                self.lidar_avoid_commit_until = now + self.lidar_avoid_hold_s
        elif self.lidar_obstacle_active and (not clear_enough) and self.lidar_avoid_commit_sign != 0.0:
            # Keep arcing around the same side while the obstacle is still within
            # the clearance radius, even if current scan points are briefly sparse.
            self.lidar_avoid_commit_until = now + self.lidar_avoid_hold_s
            desired_turn = self.lidar_avoid_commit_sign * max(abs(desired_turn), math.radians(6.0))
        else:
            if (not self.lidar_obstacle_active) and now >= self.lidar_avoid_commit_until:
                self.lidar_avoid_commit_sign = 0.0

        if self.lidar_avoid_commit_sign != 0.0 and now < self.lidar_avoid_commit_until:
            release_mag = self.lidar_avoid_release_ratio * self.lidar_avoid_bias_max
            if self.orchard_avoid_enable and self.local_costmap_enable and self.local_costmap_active:
                gap_mag = abs(self.local_costmap_best_angle) + math.radians(self.orchard_gap_turn_boost_deg)
                release_mag = min(release_mag, max(math.radians(10.0), gap_mag))
            desired_turn = self.lidar_avoid_commit_sign * max(abs(desired_turn), release_mag)

        alpha = self.lidar_avoid_filter_alpha
        self.lidar_avoid_turn_filt = (1.0 - alpha) * self.lidar_avoid_turn_filt + alpha * desired_turn
        if (not self.lidar_obstacle_active) and abs(self.lidar_avoid_turn_filt) < math.radians(1.0):
            self.lidar_avoid_turn_filt = 0.0
        self.lidar_avoid_turn = self.lidar_avoid_turn_filt
        self.lidar_last_update_ts = now

    def emergency_cb(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().warn(f"[WEB EMERGENCY CMD] {cmd}")

        if cmd == "EMERGENCY":
            # Always re-send EMG so Arduino is forced into EMER loop
            # even after serial reconnect or MCU restart.
            was_latched = self.emergency_latched
            self.emergency_latched = True
            self._abort_mission_after_emergency()
            self.last_enc_delta = 0.0
            self.speed_mps = 0.0
            self._last_enc_update_ts = time.time()
            self.prev_enc = None
            if self.gps_x is not None and self.gps_y is not None:
                self.dr_lat = self.gps_x
                self.dr_lon = self.gps_y
            else:
                self.dr_lat = None
                self.dr_lon = None
            self.send_serial("EMG")
            if not was_latched:
                self.get_logger().error("🛑 EMERGENCY LATCHED - mission aborted, upload path again before START")
            else:
                self.get_logger().warn("🛑 EMERGENCY RE-ASSERT - mission remains aborted")

        elif cmd == "RESET":
            if self.emergency_latched:
                self.emergency_latched = False
                self._abort_mission_after_emergency()
                self.last_enc_delta = 0.0
                self.speed_mps = 0.0
                self._last_enc_update_ts = time.time()
                self.prev_enc = None
                if self.gps_x is not None and self.gps_y is not None:
                    self.dr_lat = self.gps_x
                    self.dr_lon = self.gps_y
                else:
                    self.dr_lat = None
                    self.dr_lon = None

        # ---------- PWM DEBUG ----------
                self.last_pwm_l = 0
                self.last_pwm_r = 0
                self.send_serial("RST")
                self.get_logger().info("🔄 EMERGENCY RESET - mission cleared, waiting for new path + START")
            else:
                self.get_logger().warn("ℹ️ RESET IGNORED (not in EMERGENCY)")

    # =================================================
    # MAIN LOOP
    # =================================================
    def update(self):
        self.read_arduino()
        self.ensure_arduino_link()
        if self.master is None:
            now = time.time()
            # Do not let slow/no-heartbeat Pixhawk startup starve Arduino mode
            # handshaking. The mower must learn the hardware switch first.
            arduino_ready_for_pixhawk = self._arduino_rx_seen or (now - self._startup_ts) > 20.0
            if arduino_ready_for_pixhawk and now - self._last_pixhawk_retry_ts > self._pixhawk_retry_interval_s:
                self._last_pixhawk_retry_ts = now
                self.start_pixhawk_connect_async()
        self.read_pixhawk()
        self._decay_motion_estimate()
        self.control_logic()
        self.publish_all()
        self.debug_log()

    def ensure_arduino_link(self):
        now = time.time()
        if self.arduino_connected and (now - self._last_arduino_rx_ts) <= self.arduino_rx_timeout_s:
            return

        if self.arduino_connected:
            self.arduino_connected = False
            # Keep the last confirmed hardware mode for telemetry/UI. The link
            # status and arduino_age show that the value is stale; forcing MANUAL
            # here made the web page appear stuck in MANUAL even after the switch
            # had already been parsed as AUTO. Control stays disabled because
            # arduino_connected is false and mode_web is stopped below.
            self.mode_web = "STOP"
            try:
                if getattr(self, "ser", None) is not None and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._arduino_rx_seen = False
            if now - self._last_arduino_state_warn_ts > 1.0:
                self._last_arduino_state_warn_ts = now
                self.get_logger().error("❌ Arduino link timeout/disconnected -> force STOP (keep last HW mode)")

        if now - self._last_arduino_retry_ts < 1.0:
            return
        self._last_arduino_retry_ts = now
        try:
            self.ser = self.connect_arduino(self.ENCODER_PORT_CANDIDATES)
            self.arduino_connected = True
            self._last_arduino_rx_ts = time.time()
            self._arduino_rx_seen = False
            self._mode_candidate = self.mode_hardware
            self._mode_candidate_count = 0
            self._mode_candidate_since = time.time()
            self.get_logger().info("✅ Arduino reconnect success")
        except Exception as e:
            if now - self._last_arduino_state_warn_ts > 1.0:
                self._last_arduino_state_warn_ts = now
                self.get_logger().warn(f"⚠️ Arduino reconnect pending: {e}")

    # =================================================
    # PIXHAWK
    # =================================================
    def _gps_delta_m(self, lat1, lon1, lat2, lon2):
        d_north = (lat2 - lat1) * 111139.0
        d_east = (lon2 - lon1) * (111139.0 * math.cos(math.radians(lat1)))
        return d_north, d_east

    def _meters_to_latlon(self, d_north, d_east, lat_ref):
        d_lat = d_north / 111139.0
        cos_lat = max(0.2, abs(math.cos(math.radians(lat_ref))))
        d_lon = d_east / (111139.0 * cos_lat)
        return d_lat, d_lon

    def _robot_local_to_world_m(self, forward_m, left_m):
        d_north = forward_m * math.cos(self.yaw) + left_m * math.sin(self.yaw)
        d_east = forward_m * math.sin(self.yaw) - left_m * math.cos(self.yaw)
        return d_north, d_east

    def _world_to_robot_local_m(self, d_north, d_east):
        forward_m = d_north * math.cos(self.yaw) + d_east * math.sin(self.yaw)
        left_m = d_north * math.sin(self.yaw) - d_east * math.cos(self.yaw)
        return forward_m, left_m

    def _purge_obstacle_memory(self, now=None):
        if now is None:
            now = time.time()
        ttl = max(0.1, self.obstacle_memory_ttl_s)
        self.obstacle_memory = [
            p for p in self.obstacle_memory
            if (now - p.get("ts", 0.0)) <= ttl
        ]
        if len(self.obstacle_memory) > self.obstacle_memory_max_points:
            self.obstacle_memory = self.obstacle_memory[-self.obstacle_memory_max_points:]

    def _remember_lidar_cluster(self, cluster, median_r):
        if (
            not self.obstacle_memory_enable
            or self.gps_x is None
            or self.gps_y is None
            or not cluster
            or not math.isfinite(median_r)
        ):
            return

        now = time.time()
        # Use the closest real beam in the cluster. It is conservative and keeps
        # the remembered point on the side that would touch the robot first.
        nearest = min(cluster, key=lambda p: p[1])
        r = float(nearest[1])
        a = float(nearest[2])
        forward_m = r * math.cos(a)
        left_m = r * math.sin(a)
        d_north, d_east = self._robot_local_to_world_m(forward_m, left_m)
        d_lat, d_lon = self._meters_to_latlon(d_north, d_east, self.gps_x)
        lat = self.gps_x + d_lat
        lon = self.gps_y + d_lon

        self._purge_obstacle_memory(now)
        merge_m = max(0.05, self.obstacle_memory_merge_m)
        for p in self.obstacle_memory:
            pn, pe = self._gps_delta_m(p["lat"], p["lon"], lat, lon)
            if math.hypot(pn, pe) <= merge_m:
                p["lat"] = 0.65 * p["lat"] + 0.35 * lat
                p["lon"] = 0.65 * p["lon"] + 0.35 * lon
                p["ts"] = now
                p["range_m"] = min(float(p.get("range_m", r)), r)
                return

        self.obstacle_memory.append({
            "lat": lat,
            "lon": lon,
            "ts": now,
            "range_m": r,
        })
        if len(self.obstacle_memory) > self.obstacle_memory_max_points:
            self.obstacle_memory = self.obstacle_memory[-self.obstacle_memory_max_points:]

    def _obstacle_memory_threat(self):
        self.obstacle_memory_active = False
        self.obstacle_memory_min_m = float('inf')
        self.obstacle_memory_turn = 0.0
        if (
            not self.obstacle_memory_enable
            or self.gps_x is None
            or self.gps_y is None
        ):
            return False, float('inf'), 0.0

        now = time.time()
        self._purge_obstacle_memory(now)
        best = None
        best_score = float('inf')
        half_width = max(0.15, self.obstacle_memory_corridor_half_m)
        lookahead = max(self.safety_distance_m, self.obstacle_memory_lookahead_m)
        behind = max(0.0, self.obstacle_memory_behind_m)

        for p in self.obstacle_memory:
            d_north, d_east = self._gps_delta_m(self.gps_x, self.gps_y, p["lat"], p["lon"])
            forward_m, left_m = self._world_to_robot_local_m(d_north, d_east)
            if forward_m < -behind or forward_m > lookahead:
                continue
            if abs(left_m) > half_width:
                continue
            # Prefer close/front obstacles; side obstacles just behind the nose
            # are still kept so the mower does not cut back into them.
            score = max(0.0, forward_m) + 0.35 * abs(left_m)
            if score < best_score:
                best_score = score
                best = (forward_m, left_m)

        if best is None:
            return False, float('inf'), 0.0

        forward_m, left_m = best
        # Obstacles slightly beside/behind the nose should keep a steering bias,
        # but should not trigger a hard pivot/stop by reporting zero distance.
        if forward_m <= 0.0:
            min_m = max(self.safety_distance_m, self.lidar_hard_pivot_m) + 0.10
        else:
            min_m = forward_m
        turn = -self.obstacle_memory_turn_bias_rad if left_m >= 0.0 else self.obstacle_memory_turn_bias_rad
        self.obstacle_memory_active = True
        self.obstacle_memory_min_m = min_m
        self.obstacle_memory_turn = turn
        return True, min_m, turn

    def _pulses_to_distance_m(self, pulses):
        return (pulses / self.WHEEL_PULSES_PER_REV) * self.WHEEL_CIRCUM_M

    def _update_yaw_filtered(self, yaw_enu):
        commanded_motion = (
            self.mode_web == "START"
            and self.mode_hardware in (HW_AUTO, HW_FOLLOW)
            and max(abs(self.last_pwm_l), abs(self.last_pwm_r)) > 20
        )
        moving = (self.speed_mps > self.enc_trust_speed_mps) or commanded_motion
        alpha = self.yaw_alpha_move if moving else self.yaw_alpha_stop
        s_new = math.sin(yaw_enu)
        c_new = math.cos(yaw_enu)
        if self._yaw_filt_s is None or self._yaw_filt_c is None:
            self._yaw_filt_s = s_new
            self._yaw_filt_c = c_new
        else:
            self._yaw_filt_s = (1.0 - alpha) * self._yaw_filt_s + alpha * s_new
            self._yaw_filt_c = (1.0 - alpha) * self._yaw_filt_c + alpha * c_new
        self.yaw = normalize_angle(math.atan2(self._yaw_filt_s, self._yaw_filt_c))

    def _blend_angles(self, a, b, w_b):
        w_b = max(0.0, min(1.0, w_b))
        w_a = 1.0 - w_b
        s = w_a * math.sin(a) + w_b * math.sin(b)
        c = w_a * math.cos(a) + w_b * math.cos(b)
        return normalize_angle(math.atan2(s, c))

    def _update_yaw_ui(self, yaw_norm):
        # Keep UI heading absolute so north reference (0 deg) does not drift.
        self.yaw_ui = normalize_angle(yaw_norm)
        self._yaw_ui_prev = self.yaw_ui

    def _select_ui_yaw(self, imu_yaw_with_offset):
        # Prefer compass heading for UI rendering when available and fresh.
        # This keeps map arrow stable without changing core control yaw logic.
        if self.ui_use_compass_heading and self.compass_heading_enu is not None:
            if (time.time() - self._compass_last_ts) <= self.ui_compass_max_age_s:
                return normalize_angle(self.compass_heading_enu)
        return normalize_angle(imu_yaw_with_offset)

    def _decay_motion_estimate(self):
        now = time.time()
        if self._last_enc_update_ts <= 0:
            return
        if now - self._last_enc_update_ts > 0.25:
            self.speed_mps *= 0.92
            if self.speed_mps < 0.01:
                self.speed_mps = 0.0

    def _apply_encoder_dead_reckoning(self, delta_pulses):
        now = time.time()
        if self.mode_hardware == HW_MANUAL or self.emergency_latched:
            self._last_enc_update_ts = now
            self.last_enc_delta = 0.0
            self.speed_mps = 0.0
            if self.gps_x is not None and self.gps_y is not None:
                self.dr_lat = self.gps_x
                self.dr_lon = self.gps_y
            return
        if self._last_enc_update_ts <= 0:
            self._last_enc_update_ts = now
            return

        dt = max(0.001, now - self._last_enc_update_ts)
        self._last_enc_update_ts = now

        if abs(delta_pulses) < self.ENC_DRIFT_PULSE_DEADBAND:
            delta_pulses = 0.0

        dist_m = self._pulses_to_distance_m(delta_pulses)
        inst_speed = abs(dist_m) / dt
        if abs(delta_pulses) < self.ENC_MOVING_PULSE_DEADBAND:
            inst_speed = 0.0
            dist_m = 0.0
        self.speed_mps = (
            (1.0 - self.speed_ema_alpha) * self.speed_mps
            + self.speed_ema_alpha * inst_speed
        )

        if self.gps_x is None or self.gps_y is None:
            return
        if self.dr_lat is None or self.dr_lon is None:
            self.dr_lat = self.gps_x
            self.dr_lon = self.gps_y

        # ENU yaw convention used in this node: north=cos(yaw), east=sin(yaw)
        d_north = dist_m * math.cos(self.yaw)
        d_east = dist_m * math.sin(self.yaw)
        d_lat, d_lon = self._meters_to_latlon(d_north, d_east, self.dr_lat)
        self.dr_lat += d_lat
        self.dr_lon += d_lon

        # Keep encoder dead-reckoning for speed/stall estimation always,
        # but only let it drive position in explicit START_ONLY mode.
        start_only_active = (
            self.gps_control_mode == "START_ONLY"
            and self.gps_start_only_active
            and self.mode_web == "START"
            and self.mode_hardware in (HW_AUTO, HW_FOLLOW)
        )
        if (
            start_only_active
            and self.gps_ready
            and abs(delta_pulses) >= self.ENC_MOVING_PULSE_DEADBAND
        ):
            self.gps_x = self.dr_lat
            self.gps_y = self.dr_lon

    def _fuse_gps_measurement(self, lat, lon):
        if self.gps_x is None or self.gps_y is None:
            self.gps_x = lat
            self.gps_y = lon
            return

        commanded_motion = (
            self.mode_web == "START"
            and self.mode_hardware in (HW_AUTO, HW_FOLLOW)
            and max(abs(self.last_pwm_l), abs(self.last_pwm_r)) > 20
        )
        moving = (self.speed_mps > self.enc_trust_speed_mps) or commanded_motion
        alpha = self.gps_corr_alpha_move if moving else self.gps_corr_alpha_stop
        max_step = self.gps_corr_max_step_move_m if moving else self.gps_corr_max_step_stop_m

        d_north, d_east = self._gps_delta_m(self.gps_x, self.gps_y, lat, lon)
        err_m = math.hypot(d_north, d_east)
        if err_m > max_step and err_m > 1e-6:
            scale = max_step / err_m
            d_north *= scale
            d_east *= scale
            err_m = max_step

        d_north *= alpha
        d_east *= alpha
        self._fusion_last_corr_m = math.hypot(d_north, d_east)

        d_lat, d_lon = self._meters_to_latlon(d_north, d_east, self.gps_x)
        self.gps_x += d_lat
        self.gps_y += d_lon

        # Keep dead-reckoning origin close to fused state to limit long drift.
        if self.dr_lat is not None and self.dr_lon is not None:
            blend = 0.7
            d_lat_dr, d_lon_dr = self._meters_to_latlon(d_north * blend, d_east * blend, self.dr_lat)
            self.dr_lat += d_lat_dr
            self.dr_lon += d_lon_dr

    def _active_path_unit_vector(self):
        if not self.path_points or len(self.path_points) < 2:
            return None

        if self.current_idx <= 0:
            a_idx, b_idx = 0, 1
        else:
            a_idx = max(0, min(len(self.path_points) - 2, self.current_idx - 1))
            b_idx = a_idx + 1

        a_lat, a_lon = self.path_points[a_idx]
        b_lat, b_lon = self.path_points[b_idx]
        seg_n, seg_e = self._gps_delta_m(a_lat, a_lon, b_lat, b_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.25:
            return None
        return seg_n / seg_len, seg_e / seg_len

    def _fuse_live_auto_gps_measurement(self, lat, lon):
        """Fast along-path GPS correction, slow cross-path correction.

        Raw GPS can jump sideways by a few decimeters. If that full lateral jump
        is fed into path tracking immediately, yaw PID yanks the mower left/right
        and it snakes. Along-track motion is allowed to update faster because it
        mainly advances waypoint progress.
        """
        if self.gps_x is None or self.gps_y is None:
            self.gps_x = lat
            self.gps_y = lon
            if self.dr_lat is None or self.dr_lon is None:
                self.dr_lat = lat
                self.dr_lon = lon
            return

        d_north, d_east = self._gps_delta_m(self.gps_x, self.gps_y, lat, lon)
        unit = self._active_path_unit_vector()
        if unit is None:
            # No active segment yet: use conservative normal fusion.
            self._fuse_gps_measurement(lat, lon)
            return

        ux, uy = unit
        nx, ny = -uy, ux
        along = d_north * ux + d_east * uy
        lateral = d_north * nx + d_east * ny

        now = time.time()
        dt = now - self._last_gps_update_ts if self._last_gps_update_ts > 0 else 0.10
        dt = max(0.05, min(0.50, dt))
        damp_active = now < self._gps_live_jump_damp_until
        along *= max(0.0, min(1.0, self.gps_live_auto_along_alpha))
        lateral_alpha = self.gps_live_jump_damp_lateral_alpha if damp_active else self.gps_live_auto_lateral_alpha
        lateral_max_step = self.gps_live_jump_damp_lateral_max_step_m if damp_active else self.gps_live_auto_lateral_max_step_m
        lateral *= max(0.0, min(1.0, lateral_alpha))
        speed_limited_step = max(
            self.gps_live_auto_along_min_step_m,
            (max(0.0, self.speed_mps) + self.gps_live_auto_along_speed_margin_mps) * dt
        )
        along_step_cap = min(self.gps_live_auto_along_max_step_m, speed_limited_step)
        along = max(-along_step_cap, min(along_step_cap, along))
        lateral = max(-lateral_max_step, min(lateral_max_step, lateral))

        corr_n = along * ux + lateral * nx
        corr_e = along * uy + lateral * ny
        self._fusion_last_corr_m = math.hypot(corr_n, corr_e)

        d_lat, d_lon = self._meters_to_latlon(corr_n, corr_e, self.gps_x)
        self.gps_x += d_lat
        self.gps_y += d_lon

        if self.dr_lat is None or self.dr_lon is None:
            self.dr_lat = self.gps_x
            self.dr_lon = self.gps_y
        else:
            # Keep DR close, but lateral correction remains damped.
            d_lat_dr, d_lon_dr = self._meters_to_latlon(corr_n * 0.7, corr_e * 0.7, self.dr_lat)
            self.dr_lat += d_lat_dr
            self.dr_lon += d_lon_dr

    def _update_gps_filtered(self, lat, lon, fix_type):
        now = time.time()
        self.gps_raw_x = lat
        self.gps_raw_y = lon
        self.gps_fix = fix_type
        relaxed_tracking = (self.mode_hardware in (HW_MANUAL, HW_FOLLOW)) or (self.mode_web != "START")

        if self._gps_invalid(lat, lon):
            return

        # Fast display fallback so UI is not stuck at None while warming up.
        if self.gps_allow_fix1_for_display and self.gps_x is None and fix_type >= 1:
            self.gps_x = lat
            self.gps_y = lon

        self._process_gps_calibration(lat, lon, fix_type)
        if self.gps_calibrating:
            return

        if fix_type < self.gps_warmup_min_fix:
            self.gps_ready = False
            self.gps_init_samples.clear()
            return

        # Warmup gate: require stable cluster before using GPS for navigation.
        if not self.gps_ready:
            self.gps_init_samples.append((lat, lon))
            if len(self.gps_init_samples) < self.gps_init_min_samples:
                # In MANUAL/FOLLOW, keep UI position alive while warmup is running.
                if relaxed_tracking and fix_type >= 1:
                    if self.gps_x is None or self.gps_y is None:
                        self.gps_x = lat
                        self.gps_y = lon
                    else:
                        a = 0.35
                        self.gps_x += a * (lat - self.gps_x)
                        self.gps_y += a * (lon - self.gps_y)
                    self.dr_lat = self.gps_x
                    self.dr_lon = self.gps_y
                    self.last_gps_time = now
                    self._last_gps_update_ts = now
                    self.gps_from_pixhawk = True
                return

            lats = [p[0] for p in self.gps_init_samples]
            lons = [p[1] for p in self.gps_init_samples]
            med_lat = statistics.median(lats)
            med_lon = statistics.median(lons)

            max_spread_m = 0.0
            for s_lat, s_lon in self.gps_init_samples:
                dn, de = self._gps_delta_m(med_lat, med_lon, s_lat, s_lon)
                max_spread_m = max(max_spread_m, math.hypot(dn, de))
            self._gps_init_spread_m = max_spread_m

            if max_spread_m <= self.gps_init_max_spread_m and fix_type >= self.gps_ready_min_fix:
                self.gps_x = med_lat
                self.gps_y = med_lon
                self.dr_lat = med_lat
                self.dr_lon = med_lon
                self.last_gps_time = now
                self._last_gps_update_ts = now
                self.gps_from_pixhawk = True
                self.gps_ready = True
                self.get_logger().info(
                    f"✅ GPS ready: n={len(self.gps_init_samples)} spread={max_spread_m:.2f}m fix={fix_type}"
                )
            elif relaxed_tracking and fix_type >= 1:
                # Even if not ready yet, MANUAL/FOLLOW should show live movement.
                if self.gps_x is None or self.gps_y is None:
                    self.gps_x = med_lat
                    self.gps_y = med_lon
                else:
                    a = 0.30
                    self.gps_x += a * (med_lat - self.gps_x)
                    self.gps_y += a * (med_lon - self.gps_y)
                self.dr_lat = self.gps_x
                self.dr_lon = self.gps_y
                self.last_gps_time = now
                self._last_gps_update_ts = now
                self.gps_from_pixhawk = True
            return

        start_only_locked = (
            self.gps_control_mode == "START_ONLY"
            and self.gps_start_only_active
            and self.mode_web == "START"
            and self.mode_hardware in (HW_AUTO, HW_FOLLOW)
        )
        if start_only_locked:
            self.last_gps_time = now
            self._last_gps_update_ts = now
            self.gps_from_pixhawk = True
            return

        # AUTO/FOLLOW LIVE mode must update quickly while driving.
        # If we run this through the conservative fusion/drop path, encoder backlash
        # can make the estimator look frozen until STOP. Use Pixhawk GPS directly
        # with light smoothing, while still rejecting absurd jumps.
        live_auto_tracking = (
            self.gps_control_mode == "LIVE"
            and self.mode_web == "START"
            and self.mode_hardware in (HW_AUTO, HW_FOLLOW)
            and fix_type >= self.gps_ready_min_fix
        )
        if live_auto_tracking:
            jump_m = 0.0
            if self.gps_x is not None and self.gps_y is not None:
                dn_live, de_live = self._gps_delta_m(self.gps_x, self.gps_y, lat, lon)
                jump_m = math.hypot(dn_live, de_live)
            if self.gps_live_auto_direct:
                if jump_m > self.gps_live_auto_max_jump_m and now - self._last_gps_drop_log_ts > 1.0:
                    self._last_gps_drop_log_ts = now
                    self.get_logger().warn(
                        f"⚠️ GPS AUTO direct accepted large catch-up: {jump_m:.2f}m "
                        f"(limit {self.gps_live_auto_max_jump_m:.2f}m, fix={fix_type})"
                    )
                self.gps_x = lat
                self.gps_y = lon
                self.dr_lat = lat
                self.dr_lon = lon
                self.last_gps_time = now
                self._last_gps_update_ts = now
                self.gps_from_pixhawk = True
                self._fusion_last_corr_m = jump_m
                return

            if jump_m <= self.gps_live_auto_max_jump_m:
                if jump_m >= self.gps_live_jump_damp_threshold_m:
                    self._gps_live_jump_damp_until = max(
                        self._gps_live_jump_damp_until,
                        now + self.gps_live_jump_damp_hold_s
                    )
                    if now - self._last_gps_drop_log_ts > 1.0:
                        self._last_gps_drop_log_ts = now
                        self.get_logger().warn(
                            f"⚠️ GPS jump damped: {jump_m:.2f}m (hold {self.gps_live_jump_damp_hold_s:.1f}s)"
                        )
                self._fuse_live_auto_gps_measurement(lat, lon)
                self.last_gps_time = now
                self._last_gps_update_ts = now
                self.gps_from_pixhawk = True
                self._fusion_last_corr_m = jump_m
                return

            self._gps_jump_drop_count += 1
            if now - self._last_gps_drop_log_ts > 1.0:
                self._last_gps_drop_log_ts = now
                self.get_logger().warn(
                    f"⚠️ GPS live jump dropped: {jump_m:.2f}m > {self.gps_live_auto_max_jump_m:.2f}m (fix={fix_type})"
                )
            return

        # In MANUAL/FOLLOW/non-start web mode, prioritize responsive live tracking over strict jump rejection.
        if relaxed_tracking:
            if self.gps_x is None or self.gps_y is None:
                self.gps_x = lat
                self.gps_y = lon
            else:
                a = 0.25
                self.gps_x += a * (lat - self.gps_x)
                self.gps_y += a * (lon - self.gps_y)
            self.dr_lat = self.gps_x
            self.dr_lon = self.gps_y
            self.last_gps_time = now
            self._last_gps_update_ts = now
            self.gps_from_pixhawk = True
            return

        dt = now - self._last_gps_update_ts if self._last_gps_update_ts > 0 else 0.1
        dt = max(0.02, min(1.0, dt))
        ref_lat = self.dr_lat if self.dr_lat is not None else self.gps_x
        ref_lon = self.dr_lon if self.dr_lon is not None else self.gps_y
        d_north, d_east = self._gps_delta_m(ref_lat, ref_lon, lat, lon)
        step_m = math.hypot(d_north, d_east)
        max_step_m = max(self.gps_min_step_m, self.gps_max_speed_mps * dt)

        # Drop obvious outlier jump
        if step_m > max_step_m:
            self._gps_jump_drop_count += 1
            # Keep the time base fresh so a single rejected sample does not make
            # the estimator look frozen during AUTO START.
            self.last_gps_time = now
            self._last_gps_update_ts = now
            self.gps_from_pixhawk = True
            if now - self._last_gps_drop_log_ts > 1.0:
                self._last_gps_drop_log_ts = now
                self.get_logger().warn(
                    f"⚠️ GPS jump dropped: {step_m:.2f}m > {max_step_m:.2f}m (fix={fix_type})"
                )
            return

        self._fuse_gps_measurement(lat, lon)
        self.last_gps_time = now
        self._last_gps_update_ts = now
        self.gps_from_pixhawk = True

    def read_pixhawk(self):
        if self.master is None:
            return
        while True:
            try:
                msg = self.master.recv_match(blocking=False)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Pixhawk read error: {e}")
                self.master = None
                return
            if not msg:
                break

            if msg.get_type() == "ATTITUDE":
                yaw_ned = msg.yaw
                # Use MAVLink attitude yaw directly as heading convention used by controller:
                # 0 = north, +pi/2 = east.
                # (Previous transform added +pi and could flip heading by ~180 deg.)
                yaw_heading = normalize_angle(yaw_ned + self.body_yaw_offset_rad)
                self.last_imu_yaw_rate = float(msg.yawspeed)
                use_attitude_heading = True
                if self.control_use_global_heading and self.compass_heading_enu is not None:
                    if (time.time() - self._compass_last_ts) <= self.ui_compass_max_age_s:
                        use_attitude_heading = False

                if self.use_raw_imu_yaw:
                    # Use IMU yaw directly every ATTITUDE frame
                    # (no compass blend / no additional hold logic).
                    yaw_now = normalize_angle(yaw_heading)
                    if use_attitude_heading:
                        if not self._imu_yaw_initialized:
                            self.yaw = yaw_now
                            self._imu_yaw_initialized = True
                        else:
                            self.yaw = yaw_now
                    self._update_yaw_ui(self._select_ui_yaw(self.yaw))
                else:
                    if self.compass_heading_enu is not None:
                        moving = self.speed_mps > self.enc_trust_speed_mps
                        w_compass = self.yaw_compass_weight_move if moving else self.yaw_compass_weight_stop
                        yaw_heading = self._blend_angles(yaw_heading, self.compass_heading_enu, w_compass)
                    self._update_yaw_filtered(normalize_angle(yaw_heading))
                    self._update_yaw_ui(self._select_ui_yaw(self.yaw))

            elif msg.get_type() == "GLOBAL_POSITION_INT":
                # Compass heading in centi-degrees, 65535 means unknown.
                if msg.hdg != 65535:
                    hdg_deg = msg.hdg * 0.01
                    self.compass_heading_enu = normalize_angle(math.radians(hdg_deg) + self.body_yaw_offset_rad)
                    self._compass_last_ts = time.time()
                    if self.control_use_global_heading:
                        self.yaw = self.compass_heading_enu
                        self._imu_yaw_initialized = True
                    self._update_yaw_ui(self._select_ui_yaw(self.yaw))
                    # When stationary, pin heading to compass to kill idle yaw drift.
                    if (not self.use_raw_imu_yaw) and self.speed_mps <= self.enc_trust_speed_mps:
                        self._update_yaw_filtered(self.compass_heading_enu)
                        self._update_yaw_ui(self.yaw)

            elif msg.get_type() == "GPS_RAW_INT":
                self.gps_num_sats = int(getattr(msg, "satellites_visible", 0) or 0)
                self._update_gps_filtered(msg.lat * 1e-7, msg.lon * 1e-7, msg.fix_type)

    # =================================================
    # ARDUINO
    # =================================================
    def _parse_arduino_line(self, line):
        # Parse Arduino CSV status line.
        # Old format: mode,enc1,enc2
        # Mid format: mode,enc1,enc2,pump,blade,manual_led,auto_lamp,emg
        # New format: mode,enc1,enc2,pump,blade,manual_led,auto_lamp,emg,can_soc,can_voltage,can_current,can_temp,can_status
        # Speed format: same 13 fields + speed_l_mps,speed_r_mps,speed_avg_mps
        # PWM format: same 16 fields + pwm_l,pwm_r from Arduino hardware output
        s = line.strip()
        if not s or s.startswith("#"):
            return None
        try:
            parts = [p.strip() for p in s.split(",")]
            if len(parts) < 3:
                return None
            if len(parts) == 3 and not self.ARDUINO_ACCEPT_OLD_3_FIELD:
                return None
            if len(parts) not in (3, 8, 13, 16, 18) and len(parts) < 18:
                return None
            mode = int(parts[0])
            enc_l = float(parts[1])
            enc_r = float(parts[2])
            payload = {
                "mode": mode,
                "enc_l": enc_l,
                "enc_r": enc_r,
                "has_io": False,
                "has_can": False,
                "pump": False,
                "blade": 0,
                "manual_led": False,
                "auto_lamp": False,
                "emg": False,
                "can_soc": -1.0,
                "can_voltage": float("nan"),
                "can_current": float("nan"),
                "can_temperature": float("nan"),
                "can_status": 2,
                "speed_l_mps": None,
                "speed_r_mps": None,
                "speed_avg_mps": None,
                "pwm_l": None,
                "pwm_r": None,
            }
            if len(parts) >= 8:
                payload["has_io"] = True
                payload["pump"] = int(float(parts[3])) != 0
                payload["blade"] = int(float(parts[4]))
                payload["manual_led"] = int(float(parts[5])) != 0
                payload["auto_lamp"] = int(float(parts[6])) != 0
                payload["emg"] = int(float(parts[7])) != 0
            if len(parts) >= 13:
                payload["has_can"] = True
                payload["can_soc"] = float(parts[8])
                payload["can_voltage"] = float(parts[9])
                payload["can_current"] = float(parts[10])
                payload["can_temperature"] = float(parts[11])
                payload["can_status"] = int(float(parts[12]))
            if len(parts) >= 16:
                payload["speed_l_mps"] = float(parts[13])
                payload["speed_r_mps"] = float(parts[14])
                payload["speed_avg_mps"] = float(parts[15])
            if len(parts) >= 18:
                payload["pwm_l"] = int(float(parts[16]))
                payload["pwm_r"] = int(float(parts[17]))
            return payload
        except Exception:
            return None

    def _update_can_cache(self, can_soc=None, can_voltage=None, can_current=None, can_temperature=None, can_status=None):
        """Smooth BMS telemetry so one dropped/default Arduino frame does not blink the UI."""
        now = time.time()
        parsed = {}

        try:
            v = float(can_voltage)
            if math.isfinite(v):
                parsed["voltage"] = v
        except Exception:
            pass

        try:
            c = float(can_current)
            if math.isfinite(c):
                parsed["current"] = c
        except Exception:
            pass

        try:
            t = float(can_temperature)
            if math.isfinite(t):
                parsed["temperature"] = t
        except Exception:
            pass

        try:
            st = int(float(can_status))
            if st in (0, 1, 2):
                parsed["status"] = st
        except Exception:
            pass

        try:
            soc = float(can_soc)
            if math.isfinite(soc) and 0.0 <= soc <= 100.0:
                parsed["soc"] = soc
        except Exception:
            pass

        voltage = parsed.get("voltage", float("nan"))
        current = parsed.get("current", float("nan"))
        temperature = parsed.get("temperature", float("nan"))
        status = parsed.get("status", 2)
        soc = parsed.get("soc", -1.0)

        voltage_zero = math.isfinite(voltage) and abs(voltage) < 0.01
        current_zero = math.isfinite(current) and abs(current) < 0.01
        temperature_zero = math.isfinite(temperature) and abs(temperature) < 0.01
        voltage_plausible = math.isfinite(voltage) and voltage >= 8.0
        # Arduino initializes CAN fields as 0,0,0,UNKNOWN. Some BMS/CAN gaps can
        # briefly produce a stray SOC like 2 while the rest of the frame is zero;
        # never let that lone value replace the last real battery telemetry.
        looks_like_default_frame = (
            status == 2
            and voltage_zero
            and current_zero
            and temperature_zero
        )

        have_recent_real_can = (
            self.arduino_can_last_valid_ts > 0.0
            and (now - self.arduino_can_last_valid_ts) <= self.arduino_can_invalid_hold_s
        )

        if looks_like_default_frame:
            if not have_recent_real_can and "status" in parsed:
                self.arduino_can_status = status
            return

        if (
            "soc" in parsed
            and have_recent_real_can
            and self.arduino_can_soc >= 0.0
            and abs(soc - self.arduino_can_soc) > self.arduino_can_soc_max_step
        ):
            parsed.pop("soc", None)

        if "voltage" in parsed:
            self.arduino_can_voltage = voltage
        if "current" in parsed:
            self.arduino_can_current = current
        if "temperature" in parsed:
            self.arduino_can_temperature = temperature
        if "status" in parsed:
            self.arduino_can_status = status
        if "soc" in parsed:
            self.arduino_can_soc = soc

        if (
            voltage_plausible
            or (status in (0, 1) and voltage_plausible)
            or (("soc" in parsed and soc > 0.0) and voltage_plausible)
        ):
            self.arduino_can_last_valid_ts = now

    def _parse_can_debug_line(self, line):
        # Arduino may also emit: # CAN {"soc":...,"voltage":...}
        # Older code skipped all # lines, which made web battery telemetry disappear.
        if not line.startswith("# CAN"):
            return False
        start = line.find("{")
        if start < 0:
            return False
        try:
            payload = json.loads(line[start:])
        except Exception:
            return False

        status_raw = str(payload.get("status", "UNKNOWN")).upper()
        if "NORMAL" in status_raw:
            status = 0
        elif "WARNING" in status_raw or "PROTECTION" in status_raw or "PROTECT" in status_raw:
            status = 1
        else:
            status = 2

        self._update_can_cache(
            can_soc=payload.get("soc", None),
            can_voltage=payload.get("voltage", None),
            can_current=payload.get("current", None),
            can_temperature=payload.get("temperature", None),
            can_status=status,
        )
        return True

    def _parse_can_tx_debug_line(self, line):
        # Arduino debug line:
        # # TX mode=0 ... can_soc=85 can_v=26.40 can_i=-3.20 can_t=31.0 can_st=0 spd_l=0.123 spd_r=0.120 spd_avg=0.122
        if not line.startswith("# TX"):
            return False
        fields = dict(re.findall(r"\b(can_soc|can_v|can_i|can_t|can_st|spd_l|spd_r|spd_avg)=([^\s]+)", line))
        if not fields:
            return False
        self._update_can_cache(
            can_soc=fields.get("can_soc", None),
            can_voltage=fields.get("can_v", None),
            can_current=fields.get("can_i", None),
            can_temperature=fields.get("can_t", None),
            can_status=fields.get("can_st", None),
        )
        self._update_arduino_speed_cache(
            fields.get("spd_l", None),
            fields.get("spd_r", None),
            fields.get("spd_avg", None),
        )
        return True

    def _update_arduino_speed_cache(self, speed_l=None, speed_r=None, speed_avg=None):
        try:
            v = float(speed_l)
            if math.isfinite(v):
                self.arduino_speed_l_mps = v
        except Exception:
            pass
        try:
            v = float(speed_r)
            if math.isfinite(v):
                self.arduino_speed_r_mps = v
        except Exception:
            pass
        try:
            v = float(speed_avg)
            if math.isfinite(v):
                self.arduino_speed_avg_mps = v
        except Exception:
            pass

    def read_arduino(self):
        try:
            if not self.arduino_connected:
                return
            # Match the older stable path: non-blocking read only when bytes are
            # already buffered. This keeps the ROS timer loop from poking a quiet
            # USB CDC port with readline() every tick.
            if hasattr(self.ser, "in_waiting") and self.ser.in_waiting <= 0:
                return

            waiting = int(getattr(self.ser, "in_waiting", 0) or 0)
            chunk = self.ser.read(waiting).decode(errors='ignore').replace("\x00", "")
            if not chunk:
                return
            self._serial_rx_buf += chunk
            if len(self._serial_rx_buf) > 8192:
                self._serial_rx_buf = self._serial_rx_buf[-4096:]

            pieces = self._serial_rx_buf.splitlines(True)
            complete_lines = []
            pending = ""
            for piece in pieces:
                if piece.endswith(("\n", "\r")):
                    complete_lines.append(piece.strip())
                else:
                    pending = piece
            self._serial_rx_buf = pending

            lines_processed = 0
            for line in complete_lines:
                if lines_processed >= 30:
                    break
                line = line.strip()
                if not line:
                    continue
                lines_processed += 1
                now = time.time()
                # Any incoming line means serial link is alive.
                self._last_arduino_rx_ts = now
                self._arduino_rx_seen = True
                self._arduino_write_timeout_count = 0

                if line.startswith("# CAN") and self._parse_can_debug_line(line):
                    continue
                if line.startswith("# TX") and self._parse_can_tx_debug_line(line):
                    continue

                if line.startswith("#"):
                    if (
                        (not self._warned_fw_old_log_format)
                        and line.startswith("# LOG")
                        and ("EMG:" not in line)
                    ):
                        self._warned_fw_old_log_format = True
                        self.get_logger().warn(
                            "⚠️ Arduino firmware looks old (no EMG in # LOG). Re-upload latest AMR/src/main.cpp"
                        )
                    # Mirror Arduino verbose logs into ROS console (throttled).
                    if now - self._last_arduino_verbose_log_ts > 0.25:
                        self._last_arduino_verbose_log_ts = now
                        self.get_logger().info(f"[ARDUINO] {line}")
                    continue

                parsed = self._parse_arduino_line(line)
                if parsed is None:
                    # Ignore fragmented/debug serial chunks from Arduino verbose logs.
                    # Examples: "AW 0,0", "RXRAW 0,0", trailing pieces from "# RXRAW ..."
                    if (
                        line.upper().startswith(("RXRAW", "AW ", "RAW "))
                        or re.match(r"^[A-Z_ ]*[-+]?\d+\s*,\s*[-+]?\d+\s*$", line.strip())
                    ):
                        continue
                    if now - self._last_serial_parse_warn_ts > 1.0:
                        self._last_serial_parse_warn_ts = now
                        self.get_logger().warn(f"⚠️ Ignored serial line (bad format): '{line}'")
                    continue

                self.enc_l_raw = int(parsed["enc_l"])
                self.enc_r_raw = int(parsed["enc_r"])
                if parsed.get("has_io", False):
                    self.arduino_pump_on = bool(parsed.get("pump", False))
                    self.arduino_blade_state = int(parsed.get("blade", 0))
                    self.arduino_manual_led = bool(parsed.get("manual_led", False))
                    self.arduino_auto_lamp = bool(parsed.get("auto_lamp", False))
                    self.arduino_emg_flag = bool(parsed.get("emg", False))
                if parsed.get("has_can", False):
                    self._update_can_cache(
                        can_soc=parsed.get("can_soc", -1.0),
                        can_voltage=parsed.get("can_voltage", float("nan")),
                        can_current=parsed.get("can_current", float("nan")),
                        can_temperature=parsed.get("can_temperature", float("nan")),
                        can_status=parsed.get("can_status", 2),
                    )
                self._update_arduino_speed_cache(
                    parsed.get("speed_l_mps", None),
                    parsed.get("speed_r_mps", None),
                    parsed.get("speed_avg_mps", None),
                )
                if parsed.get("pwm_l") is not None and parsed.get("pwm_r") is not None:
                    self.arduino_pwm_l = int(parsed.get("pwm_l", 0))
                    self.arduino_pwm_r = int(parsed.get("pwm_r", 0))
                    self.arduino_pwm_valid = True
                mode_in = int(parsed["mode"])
                enc_l = float(parsed["enc_l"])
                enc_r = float(parsed["enc_r"])
                if mode_in not in (HW_MANUAL, HW_AUTO, HW_FOLLOW):
                    if now - self._last_serial_parse_warn_ts > 1.0:
                        self._last_serial_parse_warn_ts = now
                        self.get_logger().warn(f"⚠️ Ignored serial mode out of range: mode={mode_in} line='{line}'")
                    continue

                # If the Arduino status has IO lamps, use them as a sanity check.
                # A single noisy CSV field saying MANUAL while auto_lamp is still
                # on should not stop AUTO or split datalogs.
                if (
                    self.MODE_MANUAL_IO_CONFIRM_ENABLE
                    and parsed.get("has_io", False)
                    and mode_in == HW_MANUAL
                    and bool(parsed.get("auto_lamp", False))
                    and not bool(parsed.get("manual_led", False))
                ):
                    if now - self._last_mode_parse_debug_ts > 1.0:
                        self._last_mode_parse_debug_ts = now
                        self.get_logger().warn(
                            f"⚠️ Ignored MANUAL noise: auto_lamp=1 manual_led=0 line='{line}'"
                        )
                    continue

                if mode_in == self._mode_candidate:
                    self._mode_candidate_count += 1
                else:
                    self._mode_candidate = mode_in
                    self._mode_candidate_count = 1
                    self._mode_candidate_since = now

                old = self.mode_hardware
                need_count = (
                    self.MODE_DEBOUNCE_COUNT_MANUAL
                    if mode_in == HW_MANUAL
                    else self.MODE_DEBOUNCE_COUNT_ACTIVE
                )
                need_time = (
                    self.MODE_DEBOUNCE_TIME_MANUAL_S
                    if mode_in == HW_MANUAL
                    else self.MODE_DEBOUNCE_TIME_ACTIVE_S
                )
                if (
                    mode_in != self.mode_hardware
                    and self._mode_candidate_count >= need_count
                    and (now - self._mode_candidate_since) >= need_time
                ):
                    self.mode_hardware = mode_in
                    self._mode_candidate_count = 0
                    self._mode_candidate_since = now

                enc_avg = (enc_l + enc_r) * 0.5
                if self._prev_enc_l is not None:
                    self.last_enc_delta_l = enc_l - self._prev_enc_l
                else:
                    self.last_enc_delta_l = 0.0
                if self._prev_enc_r is not None:
                    self.last_enc_delta_r = enc_r - self._prev_enc_r
                else:
                    self.last_enc_delta_r = 0.0
                if self.prev_enc is not None:
                    self.last_enc_delta = enc_avg - self.prev_enc
                    self._apply_encoder_dead_reckoning(self.last_enc_delta)
                self.prev_enc = enc_avg
                self._prev_enc_l = enc_l
                self._prev_enc_r = enc_r

                self._last_mode_rx_ts = now
                if now - self._last_mode_parse_debug_ts > 1.0:
                    self._last_mode_parse_debug_ts = now
                    self.get_logger().info(f"[HW MODE RX] parsed mode={mode_in} line='{line}'")
                if old != self.mode_hardware:
                    self.last_enc_delta = 0.0
                    self.last_enc_delta_l = 0.0
                    self.last_enc_delta_r = 0.0
                    self.speed_mps = 0.0
                    self._drive_stuck_since = 0.0
                    self._turn_stuck_since = 0.0
                    self._recovery_phase = ""
                    self._recovery_until = 0.0
                    self._recovery_cooldown_until = 0.0
                    self._last_enc_update_ts = now
                    self.prev_enc = enc_avg
                    self._prev_enc_l = enc_l
                    self._prev_enc_r = enc_r
                    if self.gps_x is not None and self.gps_y is not None:
                        self.dr_lat = self.gps_x
                        self.dr_lon = self.gps_y
                    else:
                        self.dr_lat = None
                        self.dr_lon = None
                    name = {0:"MANUAL",1:"AUTO",2:"FOLLOW"}.get(self.mode_hardware,"?")
                    self.get_logger().info(
                        f"[HW MODE] raw={mode_in} | {old} -> {self.mode_hardware} ({name})"
                    )
        except Exception as e:
            self.arduino_connected = False
            self._arduino_rx_seen = False
            try:
                if getattr(self, "ser", None) is not None and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            now = time.time()
            if now - self._last_serial_err_log_ts > 1.0:
                self._last_serial_err_log_ts = now
                self.get_logger().warn(f"⚠️ Arduino read/parse error: {e}")

    # =================================================
    # CONTROL
    # =================================================
    def manual_cb(self, msg):
        raw = msg.data.strip()
        cmd = raw.upper()

        if self.auto_manual_assist_enabled:
            if cmd.startswith("PWM,"):
                try:
                    parts = raw.split(",")
                    if len(parts) != 3:
                        return
                    l = int(float(parts[1]))
                    r = int(float(parts[2]))
                    l = max(-255, min(255, l))
                    r = max(-255, min(255, r))
                except Exception:
                    return
            else:
                if cmd not in ("FORWARD", "BACK", "LEFT", "RIGHT", "STOP"):
                    return
                assist_pwm = 170
                if cmd == "FORWARD":
                    l, r = assist_pwm, assist_pwm
                elif cmd == "BACK":
                    l, r = -assist_pwm, -assist_pwm
                elif cmd == "LEFT":
                    l, r = -assist_pwm, assist_pwm
                elif cmd == "RIGHT":
                    l, r = assist_pwm, -assist_pwm
                else:
                    l, r = 0, 0

            now = time.time()
            self.auto_manual_assist_drive_until = now + (0.45 if (l or r) else 0.0)
            self.send_pwm(l, r)
            self.get_logger().warn(
                f"[DRIVE-ASSIST-MANUAL] {cmd} logical=({l},{r}) raw=({self.last_pwm_l},{self.last_pwm_r})"
            )
            return

        if cmd == "STOP" and not self.emergency_latched:
            self.send_pwm(0, 0)
            self.get_logger().warn("[WEB-MANUAL] Ignored standalone STOP outside emergency/assist")
            return

        # Web joystick should always work: auto-latch emergency on first manual press.
        if not self.emergency_latched:
            self.emergency_latched = True
            self.get_logger().warn("🛑 Auto-enter EMER by web joystick")

        # Re-assert EMG before manual passthrough so Arduino stays in EMER loop.
        self.send_serial("EMG")

        if cmd.startswith("PWM,"):
            try:
                parts = raw.split(",")
                if len(parts) != 3:
                    return
                l = int(float(parts[1]))
                r = int(float(parts[2]))
                l = max(-255, min(255, l))
                r = max(-255, min(255, r))
            except Exception:
                return
            l_out, r_out = self._map_jetson_pwm_output(l, r)
            self.send_serial(f"PWM,{l_out},{r_out}")
            self.last_pwm_l, self.last_pwm_r = l_out, r_out
            self.get_logger().warn(f"[EMER-MANUAL] PWM {l},{r} -> {l_out},{r_out}")
            return

        if cmd not in ("FORWARD", "BACK", "LEFT", "RIGHT", "STOP"):
            return

        emg_pwm = 170
        if cmd == "FORWARD":
            l, r = emg_pwm, emg_pwm
        elif cmd == "BACK":
            l, r = -emg_pwm, -emg_pwm
        elif cmd == "LEFT":
            l, r = -emg_pwm, emg_pwm
        elif cmd == "RIGHT":
            l, r = emg_pwm, -emg_pwm
        else:
            l, r = 0, 0
        l_out, r_out = self._map_jetson_pwm_output(l, r)
        self.send_serial(f"PWM,{l_out},{r_out}")
        self.last_pwm_l, self.last_pwm_r = l_out, r_out
        self.get_logger().warn(f"[EMER-MANUAL] {cmd} PWM {l},{r} -> {l_out},{r_out}")

    def follow_cmd_cb(self, msg):
        raw = msg.data.strip()
        cmd = raw.upper()
        if cmd.startswith("PWM,"):
            try:
                parts = raw.split(",")
                if len(parts) != 3:
                    return
                l = int(float(parts[1]))
                r = int(float(parts[2]))
                l = max(-255, min(255, l))
                r = max(-255, min(255, r))
                self.follow_pwm_override_l = l
                self.follow_pwm_override_r = r
                self.follow_cmd = "PWM"
                self.follow_cmd_ts = time.time()
            except Exception:
                return
            return

        if cmd not in ("FORWARD", "BACK", "LEFT", "RIGHT", "STOP"):
            return
        self.follow_cmd = cmd
        self.follow_cmd_ts = time.time()

    def control_logic(self):

        if (not self.arduino_connected) or (not self._arduino_rx_seen):
            # Do not continue auto behavior until the Arduino has actually sent
            # at least one status frame. A port can open while USB CDC is still
            # wedged; sending PWM into that state causes repeated write timeouts.
            now = time.time()
            if self.mode_web == "START" and now - self._last_start_blocked_warn_ts > 1.0:
                self._last_start_blocked_warn_ts = now
                reason = "disconnected" if not self.arduino_connected else "open but no RX yet"
                self.get_logger().warn(f"⛔ START blocked: Arduino serial link is {reason}; PWM stays 0,0")
            return

        if self.emergency_latched:
            # EMERGENCY: Arduino handles manual loop, Jetson does not send auto PWM
            return

        if self.gps_calibrating:
            self.send_pwm(0, 0)
            return

        if self.auto_manual_assist_enabled:
            assist_idle = time.time() >= self.auto_manual_assist_drive_until
            if assist_idle:
                self.send_pwm(0, 0)

            if self.web_run_mode == "FOLLOW":
                return

            if not self.gps_from_pixhawk or not self.gps_ready:
                now = time.time()
                if now - self._last_gps_ready_warn_ts > 1.0:
                    self._last_gps_ready_warn_ts = now
                    self.get_logger().warn(
                        f"⚠️ DRIVE ASSIST: joystick enabled, waypoint confirm waiting GPS stable... "
                        f"fix={self.gps_fix} warmup_n={len(self.gps_init_samples)} spread={self._gps_init_spread_m:.2f}m"
                    )
                return
            if not self.path_points:
                now = time.time()
                if now - self._last_wp_warn_ts > 2.0:
                    self._last_wp_warn_ts = now
                    self.get_logger().warn("⚠️ DRIVE ASSIST: joystick enabled, no waypoint to confirm")
                return
            self._auto_manual_assist_tick()
            return

        # FOLLOW tracker path: does not require GPS/waypoint lock.
        if (
            self.follow_tracker_enabled
            and self.mode_hardware == HW_FOLLOW
            and self.mode_web == "START"
            and self.web_run_mode == "FOLLOW"
        ):
            self.follow_tracker_drive()
            return

        if self.mode_web == "PAUSE":
            self.send_pwm(0, 0)
            return

        if self.mode_web == "START" and (
            (self.web_run_mode == "AUTO" and self.mode_hardware != HW_AUTO)
            or (self.web_run_mode == "FOLLOW" and self.mode_hardware != HW_FOLLOW)
        ):
            self.send_pwm(0, 0)
            return

        if not self.gps_from_pixhawk or not self.gps_ready:
            now = time.time()
            if now - self._last_gps_ready_warn_ts > 1.0:
                self._last_gps_ready_warn_ts = now
                self.get_logger().warn(
                    f"⚠️ Waiting GPS stable... fix={self.gps_fix} warmup_n={len(self.gps_init_samples)} spread={self._gps_init_spread_m:.2f}m"
                )
            self.send_pwm(0, 0)
            return

        if self.mode_hardware == HW_AUTO and self.mode_web == "START" and self.web_run_mode == "AUTO":
            if not self.path_points:
                now = time.time()
                if now - self._last_wp_warn_ts > 2.0:
                    self._last_wp_warn_ts = now
                    self.get_logger().warn("⚠️ START but no waypoint. Waiting for /waypoint")
                self.send_pwm(0, 0)
                return

            if self.mode_hardware == HW_FOLLOW and self.lidar_safety_active():
                self.send_pwm(0, 0)
                return
            self.auto_drive()
        else:
            self.send_pwm(0, 0)

    def follow_tracker_drive(self):
        now = time.time()
        if (now - self.follow_cmd_ts) > self.follow_cmd_timeout_s:
            if now - self._last_follow_cmd_wait_warn_ts > 1.0:
                self._last_follow_cmd_wait_warn_ts = now
                self.get_logger().warn("FOLLOW WAIT TRACKER: no /follow_cmd received")
                self.follow_status_pub.publish(String(data="FOLLOW WAIT TRACKER: no /follow_cmd"))
            self.send_pwm(0, 0)
            return

        cmd = self.follow_cmd
        fwd = int(self.follow_forward_pwm)
        trn = int(self.follow_turn_pwm)
        if cmd == "PWM":
            l, r = self.follow_pwm_override_l, self.follow_pwm_override_r
        elif cmd == "FORWARD":
            l, r = fwd, fwd
        elif cmd == "BACK":
            l, r = -fwd, -fwd
        elif cmd == "LEFT":
            l, r = -trn, trn
        elif cmd == "RIGHT":
            l, r = trn, -trn
        else:
            l, r = 0, 0

        # If FOLLOW commands movement but encoders do not move beyond drivetrain
        # backlash, automatically add torque instead of staying under deadzone.
        dt = max(0.01, min(0.25, now - self._last_follow_boost_ts))
        self._last_follow_boost_ts = now
        movement_cmd = (abs(l) >= 20 or abs(r) >= 20)
        encoder_moving = max(abs(self.last_enc_delta_l), abs(self.last_enc_delta_r)) > self.ENC_MOVING_PULSE_DEADBAND
        if cmd == "PWM" and movement_cmd and not encoder_moving:
            self.follow_stall_boost_pwm = min(
                self.follow_stall_boost_max,
                self.follow_stall_boost_pwm + self.follow_stall_boost_ramp * dt
            )
        else:
            self.follow_stall_boost_pwm = max(
                0.0,
                self.follow_stall_boost_pwm - self.follow_stall_boost_decay * dt
            )

        if self.follow_stall_boost_pwm > 0.0:
            boost = int(round(self.follow_stall_boost_pwm))
            if l != 0:
                l = max(-255, min(255, l + (boost if l > 0 else -boost)))
            if r != 0:
                r = max(-255, min(255, r + (boost if r > 0 else -boost)))

        if self.follow_balance_enabled:
            l, r = self._apply_follow_straight_balance(l, r)
        else:
            self._follow_balance_corr_pwm = 0.0
        self.send_pwm(l, r)
        if now - self._last_follow_dbg_ts > 1.0:
            self._last_follow_dbg_ts = now
            bal_txt = f"{self._follow_balance_corr_pwm:+.0f}" if self.follow_balance_enabled else "OFF"
            self.get_logger().info(
                f"[FOLLOW TRACKER] cmd={cmd} logical=({l},{r}) raw=({self.last_pwm_l},{self.last_pwm_r}) "
                f"boost={self.follow_stall_boost_pwm:.0f} bal={bal_txt} "
                f"enc=({self.last_enc_delta_l:.1f},{self.last_enc_delta_r:.1f}) "
                f"lidar_min={self.lidar_min_front_m:.2f}m stop<{self.follow_lidar_stop_m:.2f}m"
            )

    def _apply_follow_straight_balance(self, l, r):
        """Trim FOLLOW straight drive using encoder side-speed mismatch."""
        if not self.follow_balance_enabled:
            return l, r

        same_direction = (l > 0 and r > 0) or (l < 0 and r < 0)
        straight_cmd = same_direction and abs(l - r) <= self.follow_balance_cmd_diff_max
        moving = max(abs(self.last_enc_delta_l), abs(self.last_enc_delta_r)) > self.ENC_MOVING_PULSE_DEADBAND
        if not straight_cmd or not moving:
            self._follow_balance_corr_pwm *= 0.80
            if abs(self._follow_balance_corr_pwm) < 0.5:
                self._follow_balance_corr_pwm = 0.0
            return l, r

        # Positive error means right encoder is moving more than left, so reduce
        # right PWM and/or help left PWM. This handles small motor mismatch without
        # needing a fixed hand-tuned trim.
        err = abs(self.last_enc_delta_r) - abs(self.last_enc_delta_l)
        if abs(err) <= self.follow_balance_deadband_pulse:
            target_corr = 0.0
        else:
            target_corr = max(
                -self.follow_balance_max_pwm,
                min(self.follow_balance_max_pwm, err * self.follow_balance_kp)
            )

        a = max(0.01, min(1.0, self.follow_balance_alpha))
        self._follow_balance_corr_pwm = (1.0 - a) * self._follow_balance_corr_pwm + a * target_corr
        corr = int(round(self._follow_balance_corr_pwm))
        if corr == 0:
            return l, r

        sign = 1 if (l + r) >= 0 else -1
        l_mag = max(0, abs(l) + corr)
        r_mag = max(0, abs(r) - corr)
        return (
            max(-255, min(255, sign * int(l_mag))),
            max(-255, min(255, sign * int(r_mag))),
        )

    def lidar_safety_active(self):
        if not self.lidar_safety_enabled:
            return False
        if (time.time() - self.lidar_last_update_ts) > 0.7:
            return False
        return (
            self.lidar_raw_hard_stop_active
            or self.lidar_obstacle_active
            or (self.lidar_footprint_enable and self.lidar_footprint_blocked)
            or (self.local_costmap_enable and self.local_costmap_active and self.local_costmap_blocked)
        )

    def _reset_auto_lidar_avoid_state(self):
        self.lidar_obstacle_candidate = False
        self.lidar_obstacle_candidate_since = 0.0
        self.lidar_clear_candidate_since = 0.0
        self.lidar_obstacle_active = False
        self.lidar_confirmed_min_front_m = float('inf')
        self.lidar_avoid_commit_sign = 0.0
        self.lidar_avoid_commit_until = 0.0
        self.lidar_avoid_turn = 0.0
        self.lidar_avoid_turn_filt = 0.0
        self.lidar_footprint_blocked = False
        self.lidar_footprint_min_collision_m = float('inf')
        self.local_costmap_blocked = False
        self.local_costmap_min_collision_m = float('inf')
        self.obstacle_memory.clear()
        self.obstacle_memory_active = False
        self.obstacle_memory_min_m = float('inf')
        self.obstacle_memory_turn = 0.0

    def meter_error_to_waypoint(self, target_lat, target_lon):
        # Convert lat/lon diff to local meter frame: +x north, +y east
        d_north = (target_lat - self.gps_x) * 111139.0
        d_east = (target_lon - self.gps_y) * (111139.0 * math.cos(math.radians(self.gps_x)))
        return d_north, d_east

    def _point_from_local_m(self, d_north, d_east):
        d_lat, d_lon = self._meters_to_latlon(d_north, d_east, self.gps_x)
        return self.gps_x + d_lat, self.gps_y + d_lon

    def _restore_base_path(self, reset_idx=False):
        if self.raw_path_points:
            self.path_points, self.path_corner_indices = self._densify_path_points(self.raw_path_points)
            self._base_path_points = list(self.path_points)
            self._base_path_corner_indices = set(self.path_corner_indices)
        if reset_idx:
            self.current_idx = 0
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1

    def _latlon_from_origin_offset(self, origin_lat, origin_lon, d_north, d_east):
        d_lat, d_lon = self._meters_to_latlon(d_north, d_east, origin_lat)
        return origin_lat + d_lat, origin_lon + d_lon

    def _maybe_insert_auto_bypass(self):
        if not self.auto_bypass_enable:
            return False
        if not self.auto_lidar_avoid_enable:
            return False
        if self.mode_hardware != HW_AUTO or self.mode_web != "START":
            return False
        memory_active, memory_min_m, memory_turn = self._obstacle_memory_threat()
        live_lidar_active = self.auto_lidar_avoid_enable and self.lidar_safety_active()
        if not self.lidar_safety_enabled or not (live_lidar_active or memory_active):
            return False
        if self.gps_x is None or self.gps_y is None or not self.path_points:
            return False
        if self.current_idx >= len(self.path_points):
            return False

        now = time.time()
        if self._auto_bypass_inserted:
            if self.current_idx <= self._auto_bypass_end_idx:
                return False
            self._auto_bypass_inserted = False
            self._auto_bypass_end_idx = -1
        if (now - self._auto_bypass_last_ts) < self.auto_bypass_cooldown_s:
            return False

        if self.current_idx > 0:
            start_lat, start_lon = self.path_points[self.current_idx - 1]
            end_lat, end_lon = self.path_points[self.current_idx]
            seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
            robot_n, robot_e = self._gps_delta_m(start_lat, start_lon, self.gps_x, self.gps_y)
        else:
            start_lat, start_lon = self.gps_x, self.gps_y
            end_lat, end_lon = self.path_points[self.current_idx]
            seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
            robot_n, robot_e = 0.0, 0.0

        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < self.auto_bypass_min_remaining_m:
            return False

        ux, uy = seg_n / seg_len, seg_e / seg_len
        along = max(0.0, min(seg_len, robot_n * ux + robot_e * uy))
        remaining = seg_len - along
        if remaining < self.auto_bypass_min_remaining_m:
            return False

        # Positive side means left of the current path direction. Prefer the
        # planner/avoidance decision, then memory, then raw left/right threat.
        side = 0.0
        if live_lidar_active and abs(self.lidar_corridor_best_angle) > math.radians(2.0):
            side = 1.0 if self.lidar_corridor_best_angle > 0.0 else -1.0
        elif abs(memory_turn) > math.radians(2.0):
            side = 1.0 if memory_turn > 0.0 else -1.0
        elif self.lidar_avoid_commit_sign != 0.0:
            side = self.lidar_avoid_commit_sign
        if side == 0.0:
            side = 1.0 if self.lidar_right_threat >= self.lidar_left_threat else -1.0
        side = 1.0 if side >= 0.0 else -1.0
        nx, ny = -uy * side, ux * side

        obstacle_m = self.lidar_avoid_distance_m
        if live_lidar_active and math.isfinite(self.lidar_confirmed_min_front_m):
            obstacle_m = min(obstacle_m, self.lidar_confirmed_min_front_m)
        if memory_active and math.isfinite(memory_min_m):
            obstacle_m = min(obstacle_m, memory_min_m)
        pass_m = max(self.auto_bypass_pass_m, obstacle_m + 0.9)
        return_m = max(self.auto_bypass_return_m, pass_m + 0.9)
        raw_alongs = [
            along + self.auto_bypass_forward_m,
            along + pass_m,
            along + return_m,
        ]
        offsets = [
            self.auto_bypass_side_offset_m,
            self.auto_bypass_side_offset_m,
            0.0,
        ]

        points = []
        last_a = along
        for a, lateral in zip(raw_alongs, offsets):
            a = max(along + 0.35, min(seg_len - 0.15, a))
            if a <= last_a + 0.20:
                continue
            d_n = ux * a + nx * lateral
            d_e = uy * a + ny * lateral
            points.append(self._latlon_from_origin_offset(start_lat, start_lon, d_n, d_e))
            last_a = a

        if len(points) < 2:
            return False

        insert_at = self.current_idx
        shifted_corners = set()
        for idx in self.path_corner_indices:
            shifted_corners.add(idx + len(points) if idx >= insert_at else idx)
        self.path_points[insert_at:insert_at] = points
        self.path_corner_indices = shifted_corners
        self._auto_bypass_inserted = True
        self._auto_bypass_end_idx = insert_at + len(points) - 1
        self._auto_bypass_last_ts = now
        self.wp_align_active = False
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self.get_logger().warn(
            f"🌀 AUTO bypass inserted: side={'L' if side > 0 else 'R'} "
            f"pts={len(points)} offset={self.auto_bypass_side_offset_m:.1f}m "
            f"obs={obstacle_m:.2f}m idx={insert_at + 1}"
        )
        return True

    def _segment_lookahead_target(self, endpoint_lat, endpoint_lon, dist_to_endpoint):
        """Return a target on the active segment, not just the endpoint.

        The mower should actively rejoin the yellow line when GPS says it is
        beside the segment. Aim at the closest point on the segment plus a
        short lookahead, then fall back to the endpoint near the end.
        """
        if self.gps_x is None or self.gps_y is None:
            return endpoint_lat, endpoint_lon, dist_to_endpoint, 0.0
        if self.current_idx <= 0:
            if self._auto_start_gps is None:
                return endpoint_lat, endpoint_lon, dist_to_endpoint, 0.0
            start_lat, start_lon = self._auto_start_gps
        else:
            start_lat, start_lon = self.path_points[self.current_idx - 1]

        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, endpoint_lat, endpoint_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.5:
            return endpoint_lat, endpoint_lon, dist_to_endpoint, 0.0

        robot_n, robot_e = self._gps_delta_m(start_lat, start_lon, self.gps_x, self.gps_y)
        along = max(0.0, min(seg_len, (robot_n * seg_n + robot_e * seg_e) / seg_len))
        cross_track = abs(robot_n * seg_e - robot_e * seg_n) / seg_len
        if not self.AUTO_REJOIN_LINE_ENABLE:
            return endpoint_lat, endpoint_lon, dist_to_endpoint, cross_track

        # If the robot is outside the drawn line, use a shorter lookahead so it
        # turns back into the corridor. Once it is close, look farther ahead for
        # smoother tracking.
        lookahead_m = self.CROSSTRACK_LOOKAHEAD_M if cross_track >= self.GPS_CROSSTRACK_DEADBAND_M else self.SEGMENT_LOOKAHEAD_M
        lookahead_m = max(self.SEGMENT_LOOKAHEAD_MIN_M, min(self.SEGMENT_LOOKAHEAD_M, lookahead_m))
        if cross_track >= self.CROSSTRACK_TIGHTEN_M:
            lookahead_m = min(lookahead_m, max(0.35, self.CROSSTRACK_LOOKAHEAD_M * 0.55))
        target_along = min(seg_len, along + lookahead_m)

        # Near a waypoint/corner, keep targeting the endpoint so reach detection
        # and the stop-align behavior happen at the actual numbered point.
        if (seg_len - along) <= max(self.WAYPOINT_REACH_M, self.SEGMENT_LOOKAHEAD_MIN_M):
            return endpoint_lat, endpoint_lon, dist_to_endpoint, cross_track

        t = target_along / seg_len
        target_n = seg_n * t
        target_e = seg_e * t
        return self._latlon_from_origin_offset(start_lat, start_lon, target_n, target_e) + (
            math.hypot(target_n - robot_n, target_e - robot_e),
            cross_track,
        )

    def _auto_heading_target_yaw(self, endpoint_lat, endpoint_lon, dist_to_endpoint, cross_track_m):
        """Prefer IMU-held segment heading over noisy GPS lateral corrections.

        GPS is still used for waypoint distance/progress. While the mower is on
        a normal segment and not close to the endpoint, heading comes from the
        planned segment vector, so small GPS left/right jumps do not make it
        snake. If it is the first point, near the endpoint, or far off the path,
        fall back to direct waypoint bearing.
        """
        direct_n, direct_e = self.meter_error_to_waypoint(endpoint_lat, endpoint_lon)
        if time.time() < self._auto_force_direct_wp_until:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "NO_PROGRESS_DIRECT"
        if (
            not self.IMU_SEGMENT_HEADING_ENABLE
            or self.gps_x is None
            or self.gps_y is None
        ):
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS"
        if self.current_idx <= 0:
            if self._auto_start_gps is None:
                return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS"
            start_lat, start_lon = self._auto_start_gps
        else:
            start_lat, start_lon = self.path_points[self.current_idx - 1]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, endpoint_lat, endpoint_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.5:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS"
        robot_n, robot_e = self._gps_delta_m(start_lat, start_lon, self.gps_x, self.gps_y)
        along = (robot_n * seg_n + robot_e * seg_e) / max(seg_len, 1e-6)
        overshoot_m = along - seg_len
        if overshoot_m > self.AUTO_WP_OVERSHOOT_RECOVER_M:
            # We missed the waypoint radius. Do not keep driving parallel forever;
            # turn back toward the current waypoint until it is actually reached.
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "WP_RECOVER"

        if self.wp_align_active:
            target_yaw = math.atan2(seg_e, seg_n)
            return target_yaw, seg_n, seg_e, "ALIGN_SEG"

        if self.AUTO_WP_DIRECT_HEADING_ENABLE and dist_to_endpoint <= self.AUTO_WP_DIRECT_APPROACH_M:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS_END"

        if (
            self.AUTO_REJOIN_LINE_ENABLE
            and self.AUTO_REJOIN_HEADING_ENABLE
            and cross_track_m >= self.GPS_CROSSTRACK_DEADBAND_M
        ):
            if time.time() < self._gps_live_jump_damp_until and cross_track_m < self.gps_live_jump_damp_rejoin_xtrack_m:
                target_yaw = math.atan2(seg_e, seg_n)
                return target_yaw, seg_n, seg_e, "IMU_GPS_DAMP"
            rejoin_yaw = math.atan2(direct_e, direct_n)
            seg_yaw = math.atan2(seg_e, seg_n)
            rejoin_err_from_seg = abs(normalize_angle(rejoin_yaw - seg_yaw))
            if rejoin_err_from_seg > math.radians(self.AUTO_REJOIN_ARC_ONLY_YAW_DEG):
                # Do not keep driving parallel to the path when field GPS starts
                # beside the segment. Still aim at the rejoin target; the motion
                # layer will crawl in an arc instead of spinning in place.
                return rejoin_yaw, direct_n, direct_e, "REJOIN_HARD"
            return rejoin_yaw, direct_n, direct_e, "REJOIN"

        if self.AUTO_GPS_PATH_HEADING_ENABLE:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS_PATH"

        if (
            not self.AUTO_SEGMENT_HEADING_LOCK
            and (
                dist_to_endpoint <= self.IMU_SEGMENT_MIN_REMAIN_M
                or cross_track_m >= self.IMU_SEGMENT_MAX_CROSSTRACK_M
            )
        ):
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS"

        target_yaw = math.atan2(seg_e, seg_n)
        # Keep a forward vector for debug/scaling without using GPS sideways
        # bearing as the steering target.
        return target_yaw, seg_n, seg_e, "IMU_SEG"

    def _segment_turn_angle_rad(self, idx):
        if idx <= 0 or idx >= (len(self.path_points) - 1):
            return 0.0
        prev_lat, prev_lon = self.path_points[idx - 1]
        cur_lat, cur_lon = self.path_points[idx]
        next_lat, next_lon = self.path_points[idx + 1]
        in_n, in_e = self._gps_delta_m(prev_lat, prev_lon, cur_lat, cur_lon)
        out_n, out_e = self._gps_delta_m(cur_lat, cur_lon, next_lat, next_lon)
        in_len = math.hypot(in_n, in_e)
        out_len = math.hypot(out_n, out_e)
        if in_len < 0.35 or out_len < 0.35:
            return 0.0
        dot = max(-1.0, min(1.0, (in_n * out_n + in_e * out_e) / (in_len * out_len)))
        return math.acos(dot)

    def _adaptive_turn_policy(self, is_corner_target, turn_angle_rad, dist_to_wp, yaw_err, gps_jump_damped, obstacle_avoid_active):
        if not self.AUTO_TURN_POLICY_ENABLE:
            return {
                "name": "fixed",
                "enter_rad": self.TURN_IN_PLACE_ENTER_RAD,
                "exit_rad": self.TURN_IN_PLACE_EXIT_RAD,
                "heading_enter_rad": self.HEADING_COMMIT_ENTER_RAD,
                "heading_exit_rad": self.HEADING_COMMIT_EXIT_RAD,
                "align_tol_rad": self.wp_align_tol_rad,
                "corner_cap": self.AUTO_CORNER_APPROACH_PWM,
                "crawl_cap": None,
            }

        sharp_rad = math.radians(self.AUTO_TURN_POLICY_SHARP_DEG)
        uturn_rad = math.radians(self.AUTO_TURN_POLICY_UTURN_DEG)
        turn_abs_deg = abs(math.degrees(turn_angle_rad))
        yaw_abs = abs(yaw_err)
        slip_suspected = (
            max(abs(self.last_enc_delta_l), abs(self.last_enc_delta_r)) < self.ENC_MOVING_PULSE_DEADBAND
            and yaw_abs >= math.radians(self.AUTO_TURN_POLICY_SLIP_ENTER_DEG)
            and abs(self.last_imu_yaw_rate) < self.turn_stall_yaw_rate_rad_s
        )

        name = "line"
        enter_deg = math.degrees(self.TURN_IN_PLACE_ENTER_RAD)
        exit_deg = math.degrees(self.TURN_IN_PLACE_EXIT_RAD)
        heading_enter_deg = math.degrees(self.HEADING_COMMIT_ENTER_RAD)
        align_tol_deg = math.degrees(self.wp_align_tol_rad)
        corner_cap = self.AUTO_CORNER_APPROACH_PWM
        crawl_cap = None

        if is_corner_target and turn_angle_rad >= uturn_rad:
            name = "uturn"
            enter_deg = self.AUTO_TURN_POLICY_SHARP_ENTER_DEG
            exit_deg = self.AUTO_TURN_POLICY_UTURN_EXIT_DEG
            heading_enter_deg = max(45.0, self.AUTO_TURN_POLICY_SHARP_ENTER_DEG + 4.0)
            align_tol_deg = min(align_tol_deg, self.AUTO_TURN_POLICY_UTURN_EXIT_DEG)
            corner_cap = min(corner_cap, self.AUTO_TURN_POLICY_PREALIGN_PWM)
        elif is_corner_target and turn_angle_rad >= sharp_rad:
            name = "sharp"
            enter_deg = self.AUTO_TURN_POLICY_SHARP_ENTER_DEG
            exit_deg = self.AUTO_TURN_POLICY_EXIT_DEG
            heading_enter_deg = max(52.0, self.AUTO_TURN_POLICY_SHARP_ENTER_DEG + 6.0)
            align_tol_deg = min(align_tol_deg, self.AUTO_TURN_POLICY_EXIT_DEG)
            corner_cap = min(corner_cap, self.AUTO_TURN_POLICY_PREALIGN_PWM)
        elif is_corner_target and turn_angle_rad > math.radians(25.0):
            name = "medium"
            enter_deg = self.AUTO_TURN_POLICY_MEDIUM_ENTER_DEG
            exit_deg = self.AUTO_TURN_POLICY_EXIT_DEG
            heading_enter_deg = max(65.0, self.AUTO_TURN_POLICY_MEDIUM_ENTER_DEG + 8.0)

        if gps_jump_damped and not obstacle_avoid_active:
            name = f"{name}+gpsdamp"
            enter_deg = max(enter_deg, math.degrees(self.TURN_IN_PLACE_ENTER_RAD))
            heading_enter_deg = max(heading_enter_deg, math.degrees(self.HEADING_COMMIT_ENTER_RAD))
            crawl_cap = self.AUTO_TURN_POLICY_CRAWL_PWM

        if slip_suspected and not obstacle_avoid_active:
            name = f"{name}+slip"
            # On soft soil, entering pivot earlier when the wheels already slip
            # digs the mower in. Keep pivot thresholds high and let crawl/arc
            # recovery break traction instead.
            enter_deg = max(enter_deg, math.degrees(self.TURN_IN_PLACE_ENTER_RAD))
            heading_enter_deg = max(heading_enter_deg, math.degrees(self.HEADING_COMMIT_ENTER_RAD))
            crawl_cap = self.AUTO_TURN_POLICY_CRAWL_PWM

        if (
            is_corner_target
            and turn_angle_rad >= sharp_rad
            and dist_to_wp <= self.AUTO_TURN_POLICY_PREALIGN_DIST_M
            and yaw_abs >= math.radians(self.AUTO_TURN_POLICY_CRAWL_YAW_DEG)
            and not obstacle_avoid_active
        ):
            crawl_cap = self.AUTO_TURN_POLICY_CRAWL_PWM

        return {
            "name": name,
            "turn_deg": turn_abs_deg,
            "enter_rad": math.radians(enter_deg),
            "exit_rad": math.radians(exit_deg),
            "heading_enter_rad": math.radians(heading_enter_deg),
            "heading_exit_rad": self.HEADING_COMMIT_EXIT_RAD,
            "align_tol_rad": math.radians(align_tol_deg),
            "corner_cap": corner_cap,
            "crawl_cap": crawl_cap,
        }

    def _waypoint_handoff_radius_m(self):
        """Use a tighter reach radius at row ends/corners so AUTO does not start the U-turn early."""
        base_radius = self.WAYPOINT_REACH_M
        if self.current_idx <= 0 or self.current_idx >= (len(self.path_points) - 1):
            return base_radius
        prev_lat, prev_lon = self.path_points[self.current_idx - 1]
        cur_lat, cur_lon = self.path_points[self.current_idx]
        next_lat, next_lon = self.path_points[self.current_idx + 1]
        in_n, in_e = self._gps_delta_m(prev_lat, prev_lon, cur_lat, cur_lon)
        out_n, out_e = self._gps_delta_m(cur_lat, cur_lon, next_lat, next_lon)
        in_len = math.hypot(in_n, in_e)
        out_len = math.hypot(out_n, out_e)
        if in_len >= 0.4 and out_len >= 0.4:
            dot = max(-1.0, min(1.0, (in_n * out_n + in_e * out_e) / (in_len * out_len)))
            turn_angle = math.acos(dot)
            if turn_angle >= self.WAYPOINT_TURN_ANGLE_RAD:
                base_radius = min(base_radius, self.WAYPOINT_TURN_REACH_M)

        # If the current segment is short (common at U-turn bridge points), a
        # normal 0.8-0.9m radius can swallow the next point while still sitting
        # at the previous corner. Scale the handoff radius by segment length so
        # the mower must actually enter the short segment before advancing.
        if in_len > 0.05:
            short_radius = max(
                self.WAYPOINT_SHORT_SEGMENT_MIN_REACH_M,
                in_len * max(0.10, min(0.90, self.WAYPOINT_SHORT_SEGMENT_REACH_RATIO))
            )
            base_radius = min(base_radius, short_radius)
        return base_radius

    def _dr_segment_passed_current_waypoint(self, reach_radius_m):
        """Use encoder+IMU dead-reckoning to hand off sequential waypoints.

        GPS can drift sideways or lag behind the mower. For line/grid paths,
        advancing by mission order is safer than snapping to a neighboring row:
        once DR says we have travelled to/past the end of the current segment,
        move to the next waypoint even if GPS did not land exactly on the point.
        """
        if (
            not self.AUTO_DR_REACH_ENABLE
            or self.current_idx <= 0
            or self.current_idx >= len(self.path_points)
            or self.dr_lat is None
            or self.dr_lon is None
        ):
            return False

        start_lat, start_lon = self.path_points[self.current_idx - 1]
        end_lat, end_lon = self.path_points[self.current_idx]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.35:
            return False

        rob_n, rob_e = self._gps_delta_m(start_lat, start_lon, self.dr_lat, self.dr_lon)
        along = (rob_n * seg_n + rob_e * seg_e) / max(seg_len, 1e-6)
        cross = abs(rob_n * seg_e - rob_e * seg_n) / max(seg_len, 1e-6)
        pass_margin_m = max(reach_radius_m, self.AUTO_DR_PASS_MARGIN_M)
        max_cross_m = max(2.0, self.IMU_SEGMENT_MAX_CROSSTRACK_M)
        if self.current_idx >= len(self.path_points) - 1:
            # The final point must stop the mission even when GPS is a few
            # meters sideways. Once encoder/IMU says we reached the end portion
            # of the final segment, accept a wider cross-track band instead of
            # letting the mower drive past forever waiting for GPS radius.
            pass_margin_m = max(pass_margin_m, self.AUTO_DR_FINAL_PASS_MARGIN_M)
            max_cross_m = max(max_cross_m, self.AUTO_DR_FINAL_MAX_CROSSTRACK_M)
        pass_line_m = max(0.0, seg_len - pass_margin_m)
        if along < pass_line_m or cross > max_cross_m:
            return False
        if self.AUTO_DR_PASS_REQUIRE_GPS_CORRIDOR:
            return self._gps_segment_allows_handoff(reach_radius_m)
        return True

    def _gps_segment_allows_handoff(self, reach_radius_m):
        # current_idx >= len: out-of-bounds / mission over, let it pass through.
        if self.current_idx >= len(self.path_points):
            return True
        # current_idx == 0: no previous segment exists yet. Do NOT auto-pass;
        # the robot must physically reach WP1 via the distance check (inside_loose).
        # Returning True here is what caused WP1 to be skipped at mission start.
        if (
            self.current_idx <= 0
            or self.gps_x is None
            or self.gps_y is None
        ):
            return False

        start_lat, start_lon = self.path_points[self.current_idx - 1]
        end_lat, end_lon = self.path_points[self.current_idx]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.35:
            return True

        gps_n, gps_e = self._gps_delta_m(start_lat, start_lon, self.gps_x, self.gps_y)
        along = (gps_n * seg_n + gps_e * seg_e) / max(seg_len, 1e-6)
        cross = abs(gps_n * seg_e - gps_e * seg_n) / max(seg_len, 1e-6)
        remaining = seg_len - along
        end_n, end_e = self._gps_delta_m(self.gps_x, self.gps_y, end_lat, end_lon)
        dist_to_end = math.hypot(end_n, end_e)

        if dist_to_end <= reach_radius_m:
            return (
                cross <= self.AUTO_GPS_CORRIDOR_CONFIRM_M
                and remaining <= max(reach_radius_m, self.AUTO_GPS_PASS_WINDOW_M)
                and along >= -0.25
            )
        return (
            cross <= self.AUTO_GPS_CORRIDOR_CONFIRM_M
            and remaining <= max(reach_radius_m, self.AUTO_GPS_PASS_WINDOW_M)
            and along >= -0.25
        )

    def _segment_projection_from(self, lat, lon):
        if (
            self.current_idx <= 0
            or self.current_idx >= len(self.path_points)
            or lat is None
            or lon is None
        ):
            return None
        start_lat, start_lon = self.path_points[self.current_idx - 1]
        end_lat, end_lon = self.path_points[self.current_idx]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.35:
            return None
        p_n, p_e = self._gps_delta_m(start_lat, start_lon, lat, lon)
        along = (p_n * seg_n + p_e * seg_e) / max(seg_len, 1e-6)
        cross = abs(p_n * seg_e - p_e * seg_n) / max(seg_len, 1e-6)
        return along, cross, seg_len, along - seg_len

    def _ordered_segment_handoff_allowed(self, reach_radius_m):
        if not self.AUTO_DR_ORDERED_HANDOFF_ENABLE:
            return False
        if self.current_idx <= 0 or self.current_idx >= len(self.path_points):
            return False

        dr_ok = self._dr_segment_passed_current_waypoint(reach_radius_m)
        if dr_ok:
            if self.current_idx >= len(self.path_points) - 1:
                # At the final waypoint, trust ordered DR to stop the mission
                # instead of driving beyond the end while M9N GPS lags sideways.
                return True
            if self.AUTO_DR_PASS_REQUIRE_GPS_CORRIDOR:
                return self._gps_segment_allows_handoff(reach_radius_m)
            return True

        gps_proj = self._segment_projection_from(self.gps_x, self.gps_y)
        if gps_proj is None:
            return False
        along, cross, seg_len, overshoot = gps_proj
        return (
            overshoot >= self.AUTO_OVERSHOOT_HANDOFF_M
            and cross <= self.AUTO_OVERSHOOT_MAX_CROSSTRACK_M
        )

    def _gps_segment_progress_allows_confirm(self, reach_radius_m):
        if not self.AUTO_WP_REQUIRE_SEGMENT_PROGRESS:
            return True
        if self.current_idx <= 0:
            return True
        if (
            self.current_idx >= len(self.path_points)
            or self.gps_x is None
            or self.gps_y is None
        ):
            return True

        start_lat, start_lon = self.path_points[self.current_idx - 1]
        end_lat, end_lon = self.path_points[self.current_idx]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.35:
            return True

        gps_n, gps_e = self._gps_delta_m(start_lat, start_lon, self.gps_x, self.gps_y)
        along = (gps_n * seg_n + gps_e * seg_e) / max(seg_len, 1e-6)
        cross = abs(gps_n * seg_e - gps_e * seg_n) / max(seg_len, 1e-6)
        remaining = seg_len - along
        end_n, end_e = self._gps_delta_m(self.gps_x, self.gps_y, end_lat, end_lon)
        dist_to_end = math.hypot(end_n, end_e)
        progress_window_m = max(
            reach_radius_m,
            min(self.AUTO_WP_CONFIRM_PROGRESS_WINDOW_M, reach_radius_m + 0.70)
        )
        return (
            dist_to_end <= reach_radius_m
            and remaining <= progress_window_m
            and cross <= self.AUTO_GPS_CORRIDOR_CONFIRM_M
        )

    def _gps_confirm_allowed_by_dr(self, reach_radius_m):
        """Gate GPS waypoint hits with encoder/IMU progress.

        With GPS-only positioning, a 2-3m sideways/diagonal jump can briefly land
        inside a waypoint radius even while the mower is still mid-segment. GPS
        may start the confirmation timer, but it must agree with dead-reckoning
        that the mower has actually reached the end portion of the segment.
        """
        if not self._gps_segment_progress_allows_confirm(reach_radius_m):
            return False
        if not self.GPS_WAYPOINT_CONFIRM_WITH_DR:
            return True
        if self.current_idx <= 0:
            return True
        if (
            self.current_idx >= len(self.path_points)
            or self.dr_lat is None
            or self.dr_lon is None
        ):
            return True

        start_lat, start_lon = self.path_points[self.current_idx - 1]
        end_lat, end_lon = self.path_points[self.current_idx]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, end_lat, end_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.35:
            return True

        rob_n, rob_e = self._gps_delta_m(start_lat, start_lon, self.dr_lat, self.dr_lon)
        along = (rob_n * seg_n + rob_e * seg_e) / max(seg_len, 1e-6)
        cross = abs(rob_n * seg_e - rob_e * seg_n) / max(seg_len, 1e-6)
        remaining = seg_len - along
        confirm_window_m = max(reach_radius_m, self.GPS_WP_CONFIRM_DR_WINDOW_M)
        max_cross_m = max(self.GPS_WP_CONFIRM_DR_MAX_CROSSTRACK_M, self.IMU_SEGMENT_MAX_CROSSTRACK_M)
        return remaining <= confirm_window_m and cross <= max_cross_m

    def _finish_auto_mission(self):
        self.current_idx = len(self.path_points)
        self.mode_web = "STOP"
        self.web_run_mode = "AUTO"
        self.auto_manual_assist_enabled = False
        self.gps_start_only_active = False
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self._reset_turn_direction_state()
        self._reset_motion_progress_state()
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_cooldown_until = 0.0
        self.turn_boost_pwm = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self.steer_cmd_filt = 0.0
        self._auto_steer_mix_prev = 0.0
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self.send_pwm(0, 0)

    def _auto_manual_assist_tick(self):
        """Stop AUTO drive output but keep sequential waypoint confirmation alive."""
        if time.time() >= self.auto_manual_assist_drive_until:
            self.send_pwm(0, 0)
        if not self.path_points:
            return
        if self.current_idx >= len(self.path_points):
            self._finish_auto_mission()
            return

        target_lat, target_lon = self.path_points[self.current_idx]
        d_north, d_east = self.meter_error_to_waypoint(target_lat, target_lon)
        dist_to_wp = math.hypot(d_north, d_east)
        is_corner_target = self._is_corner_waypoint(self.current_idx)
        reach_radius_m = self._waypoint_handoff_radius_m() if is_corner_target else self.PATH_DENSE_REACH_M
        reach_radius_m = max(reach_radius_m, self.auto_manual_assist_reach_m)

        now = time.time()
        if dist_to_wp <= reach_radius_m:
            if self._wp_reach_candidate_idx != self.current_idx:
                self._wp_reach_candidate_idx = self.current_idx
                self._wp_reach_candidate_since = now
            if (now - self._wp_reach_candidate_since) >= self.auto_manual_assist_hold_s:
                reached_idx = self.current_idx
                self.current_idx += 1
                self._wp_reach_candidate_idx = -1
                self._wp_reach_candidate_since = 0.0
                self._reset_motion_progress_state()
                self.get_logger().warn(
                    f"🕹️ AUTO ASSIST confirmed WP {reached_idx + 1}/{len(self.path_points)} "
                    f"dist={dist_to_wp:.2f}m radius={reach_radius_m:.2f}m"
                )
                if self.current_idx >= len(self.path_points):
                    self._finish_auto_mission()
                    self.get_logger().info("🏁 Mission complete by AUTO ASSIST")
        elif self._wp_reach_candidate_idx == self.current_idx:
            self._wp_reach_candidate_idx = -1
            self._wp_reach_candidate_since = 0.0

    def auto_drive(self):
        now = time.time()
        if self._run_stuck_recovery():
            return
        if now < self.wp_settle_until:
            self.send_pwm(0, 0)
            return

        if self.current_idx >= len(self.path_points):
            self.send_pwm(0, 0)
            return

        if self._maybe_insert_auto_bypass():
            self.send_pwm(0, 0)
            return

        target_lat, target_lon = self.path_points[self.current_idx]
        d_north, d_east = self.meter_error_to_waypoint(target_lat, target_lon)
        dist_to_wp = math.hypot(d_north, d_east)
        drive_target_lat, drive_target_lon, drive_target_dist, cross_track_m = self._segment_lookahead_target(
            target_lat, target_lon, dist_to_wp
        )
        target_yaw, drive_north, drive_east, heading_src = self._auto_heading_target_yaw(
            drive_target_lat, drive_target_lon, dist_to_wp, cross_track_m
        )

        now = time.time()
        is_corner_target = self._is_corner_waypoint(self.current_idx)
        is_final_target = self.current_idx >= (len(self.path_points) - 1)
        turn_angle_rad = self._segment_turn_angle_rad(self.current_idx)
        gps_jump_damped = now < self._gps_live_jump_damp_until
        reach_radius_m = self._waypoint_handoff_radius_m() if is_corner_target else self.PATH_DENSE_REACH_M
        inside_loose = dist_to_wp <= reach_radius_m
        dr_passed_wp = self._ordered_segment_handoff_allowed(reach_radius_m)
        gps_passed_wp = self.AUTO_GPS_PASS_ENABLE and self._gps_segment_allows_handoff(reach_radius_m)
        confirm_radius_m = min(self.WAYPOINT_CONFIRM_M, reach_radius_m) if is_corner_target else reach_radius_m
        inside_confirm = dist_to_wp <= confirm_radius_m
        near_wp_dbg = dist_to_wp <= max(3.0, reach_radius_m * 2.5)
        if near_wp_dbg and (now - self._last_wp_gate_dbg_ts) > 0.35:
            self._last_wp_gate_dbg_ts = now
            raw_gap_m = float("nan")
            if (
                self.gps_raw_x is not None
                and self.gps_raw_y is not None
                and self.gps_x is not None
                and self.gps_y is not None
                and not self._gps_invalid(self.gps_raw_x, self.gps_raw_y)
            ):
                rg_n, rg_e = self._gps_delta_m(self.gps_x, self.gps_y, self.gps_raw_x, self.gps_raw_y)
                raw_gap_m = math.hypot(rg_n, rg_e)
            hold_age = (
                now - self._wp_reach_candidate_since
                if self._wp_reach_candidate_idx == self.current_idx and self._wp_reach_candidate_since > 0.0
                else 0.0
            )
            self.get_logger().info(
                f"[WP GATE] idx={self.current_idx + 1}/{len(self.path_points)} "
                f"dist={dist_to_wp:.2f}m reach={reach_radius_m:.2f}m confirm={confirm_radius_m:.2f}m "
                f"loose={inside_loose} in_confirm={inside_confirm} hold={hold_age:.2f}s "
                f"dr_pass={dr_passed_wp} gps_pass={gps_passed_wp} corner={is_corner_target} "
                f"gps=({self.gps_x:.7f},{self.gps_y:.7f}) raw_gap={raw_gap_m:.2f}m "
                f"drop={self._gps_jump_drop_count} pwm=({self.last_pwm_l},{self.last_pwm_r})"
            )
        if inside_loose or dr_passed_wp or gps_passed_wp:
            if self._wp_reach_candidate_idx != self.current_idx:
                self._wp_reach_candidate_idx = self.current_idx
                self._wp_reach_candidate_since = now
            required_hold_s = self.wp_reach_hold_s if is_corner_target else self.gps_reach_hold_s
            hold_ok = (now - self._wp_reach_candidate_since) >= required_hold_s
            # GPS distance must stay inside the reach zone briefly and agree
            # with segment/encoder progress. This keeps waypoint handoff
            # sequential even when GPS briefly jumps near the next point.
            if self.AUTO_STRICT_WAYPOINT_REACH:
                gps_reached = (
                    hold_ok
                    and inside_confirm
                    and self._gps_confirm_allowed_by_dr(reach_radius_m)
                )
            else:
                gps_reached = (
                    hold_ok
                    and (inside_confirm or gps_passed_wp)
                    and self._gps_confirm_allowed_by_dr(reach_radius_m)
                )
            is_final_wp = (self.current_idx >= len(self.path_points) - 1)
            if (dr_passed_wp and not is_final_wp) or gps_reached:
                reached_idx = self.current_idx
                self.current_idx += 1
                self._wp_reach_candidate_idx = -1
                self._wp_reach_candidate_since = 0.0
                self._reset_motion_progress_state()
                reached_is_corner = self._is_corner_waypoint(reached_idx)
                if reached_is_corner or dr_passed_wp:
                    self.get_logger().info(
                        f"✅ Reached {'corner' if reached_is_corner else 'segment'} "
                        f"{reached_idx + 1}/{len(self.path_points)} "
                        f"dist={dist_to_wp:.2f}m dr={dr_passed_wp}"
                    )
                if self.current_idx >= len(self.path_points):
                    self._finish_auto_mission()
                    self.get_logger().info("🏁 Mission complete")
                    return
                if reached_is_corner:
                    # Stop briefly only at original user/grid corners. Dense
                    # breadcrumbs are crossed continuously for smoother lines.
                    self.wp_settle_until = time.time() + self.wp_stop_settle_s
                    self.wp_align_active = True
                    self._reset_turn_direction_state()
                    self.send_pwm(0, 0)
                    self.pid_yaw.reset()
                    self.pid_speed.reset()
                return
        else:
            if self._wp_reach_candidate_idx == self.current_idx:
                self._wp_reach_candidate_idx = -1
                self._wp_reach_candidate_since = 0.0

        dt = now - self._last_ctrl_time
        self._last_ctrl_time = now
        if dt <= 0:
            return

        if self._last_progress_idx != self.current_idx or self._last_progress_dist_to_wp is None:
            self._path_progress_rate_mps = 0.0
        else:
            raw_progress = (self._last_progress_dist_to_wp - dist_to_wp) / max(dt, 1e-3)
            self._path_progress_rate_mps = (0.75 * self._path_progress_rate_mps) + (0.25 * raw_progress)
        self._last_progress_dist_to_wp = dist_to_wp
        self._last_progress_idx = self.current_idx

        if self.AUTO_NO_PROGRESS_GUARD_ENABLE and not self.wp_align_active:
            if self._auto_wp_guard_idx != self.current_idx:
                self._auto_wp_guard_idx = self.current_idx
                self._auto_wp_guard_start_dist = dist_to_wp
                self._auto_wp_guard_best_dist = dist_to_wp
                self._auto_wp_guard_start_ts = now
                self._auto_no_progress_count = 0
            else:
                if self._auto_wp_guard_best_dist is None:
                    self._auto_wp_guard_best_dist = dist_to_wp
                self._auto_wp_guard_best_dist = min(self._auto_wp_guard_best_dist, dist_to_wp)
                elapsed_guard_s = now - self._auto_wp_guard_start_ts
                gained_m = (self._auto_wp_guard_start_dist or dist_to_wp) - self._auto_wp_guard_best_dist
                if (
                    elapsed_guard_s >= self.AUTO_NO_PROGRESS_CHECK_S
                    and gained_m < self.AUTO_NO_PROGRESS_MIN_GAIN_M
                    and dist_to_wp > max(reach_radius_m * 1.4, 1.10)
                    and max(abs(self.last_pwm_l), abs(self.last_pwm_r)) > 35
                ):
                    self._auto_no_progress_count += 1
                    self._auto_force_direct_wp_until = now + self.AUTO_NO_PROGRESS_RETRY_S
                    self.wp_align_active = True
                    self.wp_settle_until = now + min(0.20, self.wp_stop_settle_s)
                    self._reset_turn_direction_state()
                    self.pid_yaw.reset()
                    self.steer_cmd_filt = 0.0
                    self._auto_steer_mix_prev = 0.0
                    self._auto_wp_guard_start_dist = dist_to_wp
                    self._auto_wp_guard_best_dist = dist_to_wp
                    self._auto_wp_guard_start_ts = now
                    self.get_logger().warn(
                        f"🧭 AUTO no-progress guard: WP {self.current_idx + 1}/{len(self.path_points)} "
                        f"dist={dist_to_wp:.2f}m gain={gained_m:.2f}m in {elapsed_guard_s:.1f}s; "
                        f"forcing direct waypoint heading"
                    )
                    self.send_pwm(0, 0)
                    return

        # ---- SOFT KICK (RAMP) ----
        if abs(self.last_enc_delta) < self.ENC_THRESHOLD:
            self._kick_pwm = min(self._kick_pwm + self.KICK_RAMP * dt, self.KICK_MAX)
        else:
            self._kick_pwm = 0.0

        # ---- SPEED FROM ENCODER ----
        pulses = self.last_enc_delta
        if abs(pulses) < self.ENC_MOVING_PULSE_DEADBAND:
            pulses = 0.0
        dist = self._pulses_to_distance_m(pulses)
        speed = dist / dt if dt > 0 else 0.0

        # Open-loop cruise is intentionally smoother than encoder speed PID here.
        # The motor encoder has backlash/free play, so speed PID was overreacting
        # and causing stop-go movement. Web speed scale still limits cruise PWM.
        base_pwm = self.DRIVE_MAX_PWM + self._kick_pwm

        # ---- YAW TRACK ----
        # During straight segments, hold planned segment heading with IMU.
        # GPS is used to know whether the endpoint is near, not to yank steering
        # left/right every time the GPS dot drifts sideways.
        yaw_err = normalize_angle(target_yaw - self.yaw)
        if abs(yaw_err) < self.HEADING_DEADBAND_RAD:
            yaw_err = 0.0
        yaw_err_abs = abs(yaw_err)
        if self._last_turn_yaw_err_abs is None:
            self._turn_yaw_err_rate = 0.0
        else:
            raw_yaw_progress = (self._last_turn_yaw_err_abs - yaw_err_abs) / max(dt, 1e-3)
            self._turn_yaw_err_rate = (0.75 * self._turn_yaw_err_rate) + (0.25 * raw_yaw_progress)
        self._last_turn_yaw_err_abs = yaw_err_abs

        # LiDAR obstacle avoidance (AUTO): use live scan plus short-lived memory.
        # Memory keeps the mower from snapping back into objects that have just
        # moved out of the forward scan cone while the robot is passing them.
        live_lidar_active = self.auto_lidar_avoid_enable and self.lidar_safety_active()
        memory_active, memory_min_m, memory_turn = self._obstacle_memory_threat()
        obstacle_avoid_active = self.mode_hardware == HW_AUTO and (live_lidar_active or memory_active)
        obstacle_turn = self.lidar_avoid_turn if live_lidar_active else memory_turn
        if self.lidar_avoid_commit_sign != 0.0 and now < self.lidar_avoid_commit_until:
            obstacle_turn = self.lidar_avoid_commit_sign * max(
                abs(obstacle_turn),
                self.lidar_escape_min_turn_rad
            )
        elif live_lidar_active and self.local_costmap_enable and self.local_costmap_active:
            obstacle_turn = self.local_costmap_best_angle
        elif live_lidar_active and self.lidar_footprint_enable and self.lidar_footprint_blocked:
            obstacle_turn = self.lidar_footprint_best_angle
        obstacle_min_m = float('inf')
        if live_lidar_active and math.isfinite(self.lidar_confirmed_min_front_m):
            obstacle_min_m = min(obstacle_min_m, self.lidar_confirmed_min_front_m)
        if live_lidar_active and math.isfinite(self.lidar_footprint_min_collision_m):
            obstacle_min_m = min(obstacle_min_m, self.lidar_footprint_min_collision_m)
        if live_lidar_active and self.local_costmap_enable and math.isfinite(self.local_costmap_min_collision_m):
            obstacle_min_m = min(obstacle_min_m, self.local_costmap_min_collision_m)
        if live_lidar_active and self.lidar_raw_hard_stop_active and math.isfinite(self.lidar_raw_hard_stop_m):
            obstacle_min_m = min(obstacle_min_m, self.lidar_raw_hard_stop_m)
        if memory_active and math.isfinite(memory_min_m):
            obstacle_min_m = min(obstacle_min_m, memory_min_m)
        obstacle_pivot_override = False
        obstacle_pivot_sign = 0.0
        orchard_gap_available = (
            self.orchard_avoid_enable
            and live_lidar_active
            and self.local_costmap_enable
            and self.local_costmap_active
            and self.local_costmap_best_clear_m >= self.orchard_min_clear_ahead_m
            and abs(self.local_costmap_best_angle) <= self.lidar_avoid_bias_max
        )
        if obstacle_avoid_active:
            corridor_hard_blocked = (
                live_lidar_active
                and self.lidar_corridor_enable
                and self.lidar_corridor_blocked
                and obstacle_min_m < self.lidar_corridor_stop_m
            )
            yaw_err = normalize_angle(yaw_err + obstacle_turn)
            hard_pivot_m = max(self.safety_distance_m, self.lidar_hard_pivot_m, self.front_stop_distance_m)
            if obstacle_min_m < hard_pivot_m:
                planned_path_available = (
                    self.local_costmap_enable
                    and self.local_costmap_active
                    and (not self.local_costmap_blocked)
                    and self.local_costmap_best_clear_m >= self.lidar_planned_bypass_min_clear_m
                    and abs(self.local_costmap_best_angle) <= self.lidar_avoid_bias_max
                )
                close_critical = obstacle_min_m < max(0.55, self.lidar_force_pivot_m)
                if close_critical or (not planned_path_available and not orchard_gap_available):
                    obstacle_pivot_override = True
                    if obstacle_turn > math.radians(2.0):
                        obstacle_pivot_sign = 1.0
                    elif obstacle_turn < -math.radians(2.0):
                        obstacle_pivot_sign = -1.0
                elif self.local_costmap_active:
                    obstacle_turn = self.local_costmap_best_angle
            if corridor_hard_blocked and obstacle_pivot_sign == 0.0 and not orchard_gap_available:
                # Dense close obstacle and no reliable escape side. Back up only
                # a tiny amount because the rear is blind, then pivot to rescan.
                self._start_stuck_recovery('obstacle_blocked', obstacle_turn if abs(obstacle_turn) > math.radians(1.0) else yaw_err)
                if not self._run_stuck_recovery():
                    self.send_pwm(0, 0)
                return

        camera_lane_bias, camera_lane_src = self._camera_lane_heading_bias(obstacle_avoid_active)
        if (
            camera_lane_bias != 0.0
            and not self.wp_align_active
            and not self._turn_in_place_active
        ):
            yaw_err = normalize_angle(yaw_err + camera_lane_bias)
            if heading_src:
                heading_src = f"{heading_src}+{camera_lane_src}"
            else:
                heading_src = camera_lane_src

        turn_policy = self._adaptive_turn_policy(
            is_corner_target,
            turn_angle_rad,
            dist_to_wp,
            yaw_err,
            gps_jump_damped,
            obstacle_avoid_active
        )
        self._last_turn_policy = turn_policy["name"]

        yaw_pwm_raw = self.pid_yaw.step(yaw_err, dt)
        a_steer = max(0.05, min(1.0, self.steer_filter_alpha))
        self.steer_cmd_filt = (1.0 - a_steer) * self.steer_cmd_filt + a_steer * yaw_pwm_raw
        yaw_pwm = self.steer_cmd_filt
        turn_boost = self._update_turn_boost(yaw_err, dt)

        # Hard obstacle stop/avoid: if LiDAR sees obstacle inside safety distance,
        # do not continue driving forward into it. Pivot away if a clear bias exists,
        # otherwise stop and wait.
        if obstacle_pivot_override:
            if obstacle_pivot_sign == 0.0:
                self._start_stuck_recovery('obstacle_blocked', obstacle_turn if abs(obstacle_turn) > math.radians(1.0) else yaw_err)
                if not self._run_stuck_recovery():
                    self.send_pwm(0, 0)
                return
            turn_mag = max(self.lidar_pivot_pwm, self.TURN_IN_PLACE_PWM, self.MIN_TURN_OUTER_PWM)
            yaw_not_progressing = (
                abs(self.last_imu_yaw_rate) < self.turn_stall_yaw_rate_rad_s
                and max(abs(self.last_enc_delta_l), abs(self.last_enc_delta_r)) < self.ENC_MOVING_PULSE_DEADBAND
            )
            if self._stuck_timer_triggered(yaw_not_progressing, '_turn_stuck_since', self.TURN_STUCK_DETECT_S, now):
                self._start_stuck_recovery('obstacle_blocked', obstacle_pivot_sign)
                if self._run_stuck_recovery():
                    return
            # Do not spin in place for obstacle/path entry. On soft soil this digs
            # the mower in and can loop forever; crawl a tight forward arc instead.
            self._send_turn_arc(
                obstacle_pivot_sign,
                reverse=False,
                outer_pwm=max(turn_mag, self.lidar_avoid_min_outer_pwm),
                inner_pwm=max(self.TURN_ARC_INNER_PWM, self.lidar_avoid_min_inner_pwm),
            )
            return

        if obstacle_avoid_active:
            # Avoidance must keep moving around the obstacle instead of getting
            # trapped by normal waypoint heading gates.
            self.wp_align_active = False
            self._heading_commit_active = False
            self._turn_in_place_active = False

        hard_align_needed = (
            self.AUTO_HARD_ALIGN_ENABLE
            and not obstacle_avoid_active
            and dist_to_wp > max(reach_radius_m, self.AUTO_HARD_ALIGN_MIN_DIST_M)
            and abs(yaw_err) >= self.AUTO_HARD_ALIGN_YAW_RAD
        )
        if hard_align_needed:
            self._heading_commit_active = True
            self._turn_in_place_active = True
            turn_mag = self._smooth_turn_pwm(yaw_err, yaw_pwm, turn_boost, align=False)
            turn_mag = max(
                int(round(self.AUTO_HARD_ALIGN_PIVOT_PWM)),
                min(int(round(self.AUTO_HARD_ALIGN_MAX_PWM)), int(turn_mag)),
            )
            turn_sign = self._turn_mix_sign(yaw_err, yaw_pwm, now)
            yaw_not_progressing = (
                abs(self.last_imu_yaw_rate) < self.turn_stall_yaw_rate_rad_s
                and self._turn_yaw_err_rate < self.TURN_STUCK_YAW_PROGRESS_RAD_S
                and abs(yaw_err) > self.turn_stall_err_rad
            )
            turn_stuck = yaw_not_progressing or (
                self._enc_slip_asymmetric()
                and abs(yaw_err) > self.turn_stall_err_rad
            )
            if self._stuck_timer_triggered(turn_stuck, '_turn_stuck_since', self.TURN_STUCK_DETECT_S, now):
                self._start_stuck_recovery('turn_in_place', yaw_err)
                self._run_stuck_recovery()
                return
            if now - getattr(self, "_last_hard_align_log_ts", 0.0) > 0.5:
                self._last_hard_align_log_ts = now
                self.get_logger().warn(
                    f"🧭 AUTO hard-align gate: idx={self.current_idx + 1}/{len(self.path_points)} "
                    f"dist={dist_to_wp:.2f}m yaw_err={math.degrees(yaw_err):+.1f}deg "
                    f"pivot_pwm={turn_mag} policy={turn_policy['name']}"
                )
            self._send_stationary_pivot(turn_sign, turn_mag)
            return

        # Waypoint handoff behavior:
        # after reaching a point, align heading to next waypoint before moving.
        if self.wp_align_active:
            align_tol_rad = turn_policy["align_tol_rad"]
            if abs(yaw_err) <= align_tol_rad:
                self.wp_align_active = False
                self._turn_latch_sign = 0.0
                self._turn_direction_override_until = 0.0
                self._turn_direction_override_sign = 0.0
                self.steer_cmd_filt = 0.0
                self._auto_steer_mix_prev = 0.0
                self.pid_yaw.reset()
                self.send_pwm(0, 0)
                self.wp_settle_until = time.time() + self.wp_stop_settle_s
                return
            else:
                turn_mag = self._smooth_turn_pwm(yaw_err, yaw_pwm, turn_boost, align=True)
                turn_sign = self._turn_mix_sign(yaw_err, yaw_pwm, now)
                yaw_not_progressing = (
                    abs(self.last_imu_yaw_rate) < self.turn_stall_yaw_rate_rad_s
                    and self._turn_yaw_err_rate < self.TURN_STUCK_YAW_PROGRESS_RAD_S
                    and abs(yaw_err) > self.turn_stall_err_rad
                )
                turn_stuck = yaw_not_progressing or (
                    self._enc_slip_asymmetric()
                    and abs(yaw_err) > self.turn_stall_err_rad
                )
                if self._stuck_timer_triggered(turn_stuck, '_turn_stuck_since', self.TURN_STUCK_DETECT_S, now):
                    self._start_stuck_recovery('wp_align', yaw_err)
                    self._run_stuck_recovery()
                    return
                if self.AUTO_HARD_ALIGN_ENABLE and abs(yaw_err) >= self.AUTO_HARD_ALIGN_YAW_RAD:
                    turn_mag = max(
                        int(round(self.AUTO_HARD_ALIGN_PIVOT_PWM)),
                        min(int(round(self.AUTO_HARD_ALIGN_MAX_PWM)), int(turn_mag)),
                    )
                    self._send_stationary_pivot(turn_sign, turn_mag)
                else:
                    self._send_turn_arc(
                        turn_sign,
                        reverse=False,
                        outer_pwm=max(turn_mag, self.TURN_ARC_OUTER_PWM),
                        inner_pwm=self.TURN_ARC_INNER_PWM,
                    )
                return

        if not obstacle_avoid_active:
            if self._heading_commit_active:
                if abs(yaw_err) <= turn_policy["heading_exit_rad"]:
                    self._heading_commit_active = False
            else:
                if abs(yaw_err) >= turn_policy["heading_enter_rad"]:
                    self._heading_commit_active = True

            if self._turn_in_place_active:
                if abs(yaw_err) <= turn_policy["exit_rad"]:
                    self._turn_in_place_active = False
                    self._turn_latch_sign = 0.0
                    self._turn_direction_override_until = 0.0
                    self._turn_direction_override_sign = 0.0
            else:
                if abs(yaw_err) >= turn_policy["enter_rad"]:
                    self._turn_in_place_active = True

            if self._turn_in_place_active:
                turn_mag = self._smooth_turn_pwm(yaw_err, yaw_pwm, turn_boost, align=False)
                turn_sign = self._turn_mix_sign(yaw_err, yaw_pwm, now)
                yaw_not_progressing = (
                    abs(self.last_imu_yaw_rate) < self.turn_stall_yaw_rate_rad_s
                    and self._turn_yaw_err_rate < self.TURN_STUCK_YAW_PROGRESS_RAD_S
                    and abs(yaw_err) > self.turn_stall_err_rad
                )
                turn_stuck = yaw_not_progressing or (
                    self._enc_slip_asymmetric()
                    and abs(yaw_err) > self.turn_stall_err_rad
                )
                if self._stuck_timer_triggered(turn_stuck, '_turn_stuck_since', self.TURN_STUCK_DETECT_S, now):
                    self._start_stuck_recovery('turn_in_place', yaw_err)
                    self._run_stuck_recovery()
                    return
                if self.AUTO_HARD_ALIGN_ENABLE and abs(yaw_err) >= self.AUTO_HARD_ALIGN_YAW_RAD:
                    turn_mag = max(
                        int(round(self.AUTO_HARD_ALIGN_PIVOT_PWM)),
                        min(int(round(self.AUTO_HARD_ALIGN_MAX_PWM)), int(turn_mag)),
                    )
                    self._send_stationary_pivot(turn_sign, turn_mag)
                else:
                    self._send_turn_arc(
                        turn_sign,
                        reverse=False,
                        outer_pwm=max(turn_mag, self.TURN_ARC_OUTER_PWM),
                        inner_pwm=self.TURN_ARC_INNER_PWM,
                    )
                return

        # Slow down when heading error is high so turning into path is easier
        heading_scale = max(0.68, math.cos(abs(yaw_err)))
        # Smooth approach near waypoint; doesn't need exact hit on point center.
        if dist_to_wp < 2.5:
            heading_scale *= max(0.75, dist_to_wp / 2.5)
        if obstacle_avoid_active:
            front_scale = max(0.0, 1.0 - min(1.0, self.lidar_front_threat))
            heading_scale *= max(0.32, front_scale)
            # Do not keep driving into a person/object once it enters the hard
            # pivot bubble. Arc while far, pivot/hold while close.
            if obstacle_min_m < max(self.safety_distance_m, self.lidar_hard_pivot_m, self.front_stop_distance_m):
                base_pwm = 0.0
        base_pwm *= heading_scale
        if (not obstacle_avoid_active) and (not gps_jump_damped) and cross_track_m >= self.GPS_CROSSTRACK_DEADBAND_M:
            # If GPS says we are drifting outside the line, slow down so the
            # steering correction can catch up before the mower overshoots far.
            xtrk_ratio = max(0.0, min(1.0, cross_track_m / max(0.1, self.CROSSTRACK_TIGHTEN_M)))
            xtrk_cap = self.AUTO_CORNER_APPROACH_PWM + (self.DRIVE_MAX_PWM - self.AUTO_CORNER_APPROACH_PWM) * (1.0 - xtrk_ratio)
            base_pwm = min(base_pwm, xtrk_cap)
        # U-turn/corner approach: cruise slowly before the corner so GPS/IMU can
        # settle and the pivot starts from a calm robot instead of a fast overshoot.
        if is_corner_target and dist_to_wp < self.AUTO_CORNER_SLOWDOWN_DIST_M:
            t_corner = max(0.0, min(1.0, dist_to_wp / max(0.1, self.AUTO_CORNER_SLOWDOWN_DIST_M)))
            corner_floor = turn_policy["corner_cap"]
            corner_cap = corner_floor + (self.DRIVE_MAX_PWM - corner_floor) * t_corner
            base_pwm = min(base_pwm, corner_cap)
        if turn_policy["crawl_cap"] is not None and base_pwm > 0.0:
            base_pwm = min(base_pwm, turn_policy["crawl_cap"])
        if obstacle_avoid_active and base_pwm > 0.0:
            base_pwm = max(base_pwm, self.lidar_avoid_min_base_pwm)
            if self.orchard_avoid_enable and live_lidar_active and self.local_costmap_active:
                # In a row of trees, crawl through the chosen gap instead of
                # accelerating into the next trunk or oscillating left/right.
                base_pwm = min(base_pwm, self.orchard_crawl_pwm)
        heading_blocked = (
            (not obstacle_avoid_active)
            and (abs(yaw_err) >= self.FORWARD_BLOCK_YAW_RAD)
        )
        if heading_blocked:
            base_pwm = 0.0
        elif (not obstacle_avoid_active) and self._heading_commit_active:
            # Do not freeze at medium heading errors. On soft soil a stopped
            # pivot can dig in; crawl-turn with enough torque until aligned.
            base_pwm = max(base_pwm, self.AUTO_TURN_POLICY_CRAWL_PWM)
        # When heading error is moderate, enforce enough thrust so wheels don.t stall.
        if (not heading_blocked) and abs(yaw_err) > math.radians(20.0):
            base_pwm = max(base_pwm, self.MIN_TURN_INNER_PWM)
        # Mower mission is forward-tracking only; prevent full reverse drive in AUTO/FOLLOW.
        base_pwm = max(0.0, base_pwm)
        if base_pwm > 0.0:
            base_pwm = max(self.MIN_FORWARD_PWM, base_pwm)

        # Forward-only differential steering:
        # cap steering to a fraction of forward PWM so wheel command does not flip negative.
        steer_cap = self.TURN_RATIO_MAX * base_pwm
        if obstacle_avoid_active and base_pwm > 0.0:
            steer_cap = max(steer_cap, self.lidar_avoid_min_steer_pwm)
        steer = max(-steer_cap, min(steer_cap, yaw_pwm))
        line_straight_hold = (
            not obstacle_avoid_active
            and base_pwm > 0.0
            and heading_src == "IMU_SEG"
            and abs(yaw_err) <= self.AUTO_LINE_STRAIGHT_HOLD_RAD
        )
        if line_straight_hold:
            steer = 0.0
            self.steer_cmd_filt = 0.0
            self._auto_steer_mix_prev = 0.0
        if obstacle_avoid_active and base_pwm > 0.0:
            avoid_sign = 0.0
            if abs(obstacle_turn) > math.radians(1.0):
                avoid_sign = 1.0 if obstacle_turn > 0.0 else -1.0
            elif self.lidar_avoid_commit_sign != 0.0:
                avoid_sign = self.lidar_avoid_commit_sign
            elif abs(yaw_err) > self.HEADING_DEADBAND_RAD:
                avoid_sign = 1.0 if yaw_err > 0.0 else -1.0
            if avoid_sign != 0.0:
                steer = math.copysign(max(abs(steer), self.lidar_avoid_min_steer_pwm), avoid_sign)
        if (
            base_pwm > 0.0
            and abs(yaw_err) > self.HEADING_DEADBAND_RAD
            and abs(yaw_err) < self.TURN_ACTIVE_YAW_RAD
            and abs(steer) < self.STRAIGHT_HOLD_MIN_STEER_PWM
        ):
            steer = math.copysign(self.STRAIGHT_HOLD_MIN_STEER_PWM, yaw_err)
        if not obstacle_avoid_active and base_pwm > 0.0:
            max_step = max(1.0, self.AUTO_STEER_MIX_SLEW_PWM_PER_S * dt)
            if steer > self._auto_steer_mix_prev:
                steer = min(steer, self._auto_steer_mix_prev + max_step)
            else:
                steer = max(steer, self._auto_steer_mix_prev - max_step)
            self._auto_steer_mix_prev = steer
        else:
            self._auto_steer_mix_prev = steer
        steer_for_mix = -steer if self.auto_arc_steer_invert else steer
        pwm_l = int(base_pwm + steer_for_mix)
        pwm_r = int(base_pwm - steer_for_mix)

        pre_shape_cap = self.TURN_MAX_PWM if obstacle_avoid_active else self.DRIVE_MAX_PWM
        pwm_l = max(0, min(pre_shape_cap, pwm_l))
        pwm_r = max(0, min(pre_shape_cap, pwm_r))
        pwm_l, pwm_r = self._shape_drive_pwm(pwm_l, pwm_r, yaw_err, steer)
        if obstacle_avoid_active and max(pwm_l, pwm_r) > 0:
            # Force an arc around the obstacle with both wheels above the
            # drivetrain deadzone. Stronger wheel becomes the outside wheel.
            if steer_for_mix >= 0:
                pwm_l = max(pwm_l, int(self.lidar_avoid_min_outer_pwm))
                pwm_r = max(pwm_r, int(self.lidar_avoid_min_inner_pwm))
            else:
                pwm_r = max(pwm_r, int(self.lidar_avoid_min_outer_pwm))
                pwm_l = max(pwm_l, int(self.lidar_avoid_min_inner_pwm))
            pwm_l = min(self.TURN_MAX_PWM, pwm_l)
            pwm_r = min(self.TURN_MAX_PWM, pwm_r)

        # Cruise slower, but if the robot is commanded forward and the drivetrain
        # is not actually moving, add temporary torque above cruise cap.
        near_straight = (abs(yaw_err) <= self.TURN_ACTIVE_YAW_RAD) and (abs(steer) <= 15.0)
        drive_not_moving = max(abs(self.last_enc_delta_l), abs(self.last_enc_delta_r)) < self.ENC_MOVING_PULSE_DEADBAND
        if (
            (near_straight or obstacle_avoid_active)
            and not heading_blocked
            and max(pwm_l, pwm_r) >= int(self.STRAIGHT_PWM_MIN)
            and drive_not_moving
        ):
            if self._drive_boost_since <= 0.0:
                self._drive_boost_since = now
            if (now - self._drive_boost_since) >= self.DRIVE_STALL_BOOST_DELAY_S:
                self.drive_stall_boost_pwm = min(
                    self.DRIVE_STALL_BOOST_MAX,
                    self.drive_stall_boost_pwm + self.DRIVE_STALL_BOOST_RAMP * dt
                )
        else:
            self._drive_boost_since = 0.0
            self.drive_stall_boost_pwm = max(
                0.0,
                self.drive_stall_boost_pwm - self.DRIVE_STALL_BOOST_DECAY * dt
            )

        if self.drive_stall_boost_pwm > 0.5 and max(pwm_l, pwm_r) > 0:
            boost = int(round(self.drive_stall_boost_pwm))
            boost_cap = self.TURN_MAX_PWM if obstacle_avoid_active else int(self.DRIVE_STALL_BOOST_CAP_PWM)
            pwm_l = min(boost_cap, pwm_l + boost)
            pwm_r = min(boost_cap, pwm_r + boost)

        traction_active = (
            not heading_blocked
            and max(pwm_l, pwm_r) >= int(self.TRACTION_ASSIST_MIN_CMD_PWM)
        )
        traction_pwm = self._update_traction_assist(traction_active, not drive_not_moving, dt, now)
        if traction_pwm > 0.5 and max(pwm_l, pwm_r) > 0:
            traction_add = int(round(traction_pwm))
            traction_cap = self.TURN_MAX_PWM if obstacle_avoid_active else int(self.DRIVE_STALL_BOOST_CAP_PWM)
            if pwm_l > 0:
                pwm_l = min(traction_cap, pwm_l + traction_add)
            if pwm_r > 0:
                pwm_r = min(traction_cap, pwm_r + traction_add)

        encoder_not_moving = (
            self.speed_mps < self.DRIVE_STUCK_SPEED_MAX
            and max(abs(self.last_enc_delta_l), abs(self.last_enc_delta_r)) < self.ENC_MOVING_PULSE_DEADBAND
        )
        slip_asymmetric = self._enc_slip_asymmetric()
        gps_not_progressing = (
            self.gps_ready
            and dist_to_wp > self.DRIVE_STUCK_PROGRESS_MIN_DIST_M
            and self._path_progress_rate_mps < self.DRIVE_STUCK_PROGRESS_MIN_MPS
            and not heading_blocked
            and not obstacle_avoid_active
        )
        drive_stuck = (
            self.AUTO_STUCK_RECOVERY_ENABLE
            and
            max(pwm_l, pwm_r) >= int(self.MIN_PWM)
            and (encoder_not_moving or gps_not_progressing or slip_asymmetric)
        )
        # Panic PWM is now reserved for real drivetrain stall/slip from encoder
        # evidence. GPS progress alone is too noisy and made the mower surge fast
        # even while it was physically moving.
        panic_active = False
        if self.AUTO_SLIP_PANIC_ENABLE:
            panic_active = self._update_slip_panic(
                (encoder_not_moving or slip_asymmetric) and max(pwm_l, pwm_r) >= int(self.MIN_PWM),
                yaw_err,
                now
            )
        if panic_active:
            if abs(yaw_err) > self.TURN_ACTIVE_YAW_RAD:
                turn_sign = self._turn_mix_sign(yaw_err, yaw_pwm, now)
                self._send_turn_arc(
                    turn_sign,
                    reverse=False,
                    outer_pwm=self.SLIP_PANIC_PWM,
                    inner_pwm=max(self.TURN_ARC_INNER_PWM, int(self.SLIP_PANIC_PWM * 0.55)),
                )
            else:
                self.send_pwm(self.SLIP_PANIC_PWM, self.SLIP_PANIC_PWM)
            return
        if self._stuck_timer_triggered(drive_stuck, '_drive_stuck_since', self.DRIVE_STUCK_DETECT_S, now):
            self._start_stuck_recovery('drive_path', yaw_err)
            self._run_stuck_recovery()
            return

        if now - self._last_auto_drive_dbg_ts > 0.5:
            self._last_auto_drive_dbg_ts = now
            self.get_logger().info(
                f"[AUTO DBG] idx={self.current_idx + 1}/{len(self.path_points)} dist={dist_to_wp:.2f}m look={drive_target_dist:.2f}m xtrk={cross_track_m:.2f}m hsrc={heading_src} straight={line_straight_hold} target={math.degrees(target_yaw):.1f}deg yaw={math.degrees(self.yaw):.1f}deg err={math.degrees(yaw_err):.1f}deg commit={self._heading_commit_active} pivot={self._turn_in_place_active} avoid={math.degrees(self.lidar_avoid_turn):.1f}deg obs={obstacle_avoid_active} minFront={self.lidar_confirmed_min_front_m:.2f}m pwm=({pwm_l},{pwm_r}) enc=({self.last_enc_delta_l:.1f},{self.last_enc_delta_r:.1f}) prog={self._path_progress_rate_mps:.2f}m/s yawProg={math.degrees(self._turn_yaw_err_rate):.1f}d/s rec={self._recovery_phase or '-'}"
                f" stallBoost={self.drive_stall_boost_pwm:.0f} traction={self.traction_assist_pwm:.0f} lidarPts={self.lidar_close_points}/{self.lidar_close_cluster}"
                f" turnPolicy={turn_policy['name']}({turn_policy.get('turn_deg', 0.0):.0f}deg)"
                f" cam={camera_lane_src}:{math.degrees(camera_lane_bias):+.1f}/{self.camera_lane_conf:.2f}"
                f" cmap={'ON' if self.local_costmap_active else 'OFF'}"
                f"/{'BLK' if self.local_costmap_blocked else 'CLR'}"
                f" angle={math.degrees(self.local_costmap_best_angle):.0f} clear={self.local_costmap_best_clear_m:.1f}"
            )

        self.send_pwm(pwm_l, pwm_r)

    def _enc_slip_asymmetric(self):
        abs_l = abs(self.last_enc_delta_l)
        abs_r = abs(self.last_enc_delta_r)
        hi = max(abs_l, abs_r)
        lo = min(abs_l, abs_r)
        if hi <= self.ENC_MOVING_PULSE_DEADBAND:
            return False
        return lo <= max(1.0, hi * self.SLIP_SIDE_RATIO_MAX)

    def _stuck_timer_triggered(self, active, attr_name, hold_s, now):
        ts = getattr(self, attr_name)
        if active:
            if ts <= 0.0:
                setattr(self, attr_name, now)
                return False
            return (now - ts) >= hold_s
        setattr(self, attr_name, 0.0)
        return False

    def _update_slip_panic(self, active, yaw_err, now):
        """Briefly punch PWM to 255 when the drivetrain keeps slipping/stalling."""
        if now < self._slip_panic_until:
            return True
        if not active:
            self._slip_panic_since = 0.0
            return False
        if self._slip_panic_since <= 0.0:
            self._slip_panic_since = now
            return False
        if (now - self._slip_panic_since) < self.SLIP_PANIC_HOLD_S:
            return False
        self._slip_panic_since = 0.0
        self._slip_panic_until = now + self.SLIP_PANIC_S
        self.get_logger().warn(
            f"⚡ SLIP PANIC PWM={self.SLIP_PANIC_PWM} for {self.SLIP_PANIC_S:.2f}s "
            f"yaw_err={math.degrees(yaw_err):.1f} enc=({self.last_enc_delta_l:.1f},{self.last_enc_delta_r:.1f})"
        )
        return True

    def _update_traction_assist(self, active, moving_ok, dt, now):
        if not self.AUTO_TRACTION_ASSIST_ENABLE:
            self._traction_assist_since = 0.0
            self.traction_assist_pwm = 0.0
            return 0.0

        if active and not moving_ok:
            if self._traction_assist_since <= 0.0:
                self._traction_assist_since = now
            if (now - self._traction_assist_since) >= self.TRACTION_ASSIST_START_S:
                self.traction_assist_pwm = min(
                    self.TRACTION_ASSIST_MAX_PWM,
                    self.traction_assist_pwm + self.TRACTION_ASSIST_RAMP_PWM_PER_S * max(0.0, dt)
                )
        else:
            self._traction_assist_since = 0.0
            self.traction_assist_pwm = max(
                0.0,
                self.traction_assist_pwm - self.TRACTION_ASSIST_DECAY_PWM_PER_S * max(0.0, dt)
            )
        return self.traction_assist_pwm

    def _start_stuck_recovery(self, reason, yaw_err):
        now = time.time()
        if now < self._recovery_cooldown_until:
            return False
        if reason != "obstacle_blocked" and not self.AUTO_STUCK_RECOVERY_ENABLE:
            self._recovery_phase = ""
            self._recovery_until = 0.0
            self._drive_stuck_since = 0.0
            self._turn_stuck_since = 0.0
            return False
        turn_recovery = reason in ("wp_align", "turn_in_place")
        obstacle_recovery = reason == "obstacle_blocked"
        if obstacle_recovery and self.OBSTACLE_RECOVERY_ENABLE:
            self._recovery_phase = "obstacle_backup"
            self._recovery_until = now + self.OBSTACLE_BACKUP_S
        elif reason == "drive_path" and self.UNSTICK_WIGGLE_ENABLE:
            self._recovery_phase = "unstick_wiggle"
            self._recovery_wiggle_step = 0
            self._recovery_until = now + self.UNSTICK_WIGGLE_STEP_S
        elif turn_recovery and self.AUTO_TURN_NUDGE_RECOVERY_ENABLE:
            self._recovery_phase = "turn_nudge"
            self._recovery_until = now + self.TURN_NUDGE_S
        else:
            self._recovery_phase = "reverse"
            self._recovery_until = now + self.RECOVERY_REVERSE_S
        self._recovery_turn_sign = 1.0 if yaw_err >= 0.0 else -1.0
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        if reason != "drive_path":
            self._recovery_wiggle_step = 0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self._kick_pwm = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self.traction_assist_pwm = 0.0
        self._traction_assist_since = 0.0
        self._auto_steer_mix_prev = 0.0
        self.get_logger().warn(
            f"⚠️ STUCK recovery: {reason} | yaw_err={math.degrees(yaw_err):.1f}deg | enc=({self.last_enc_delta_l:.1f},{self.last_enc_delta_r:.1f})"
        )
        return True

    def _run_stuck_recovery(self):
        now = time.time()
        if not self._recovery_phase:
            return False
        if now >= self._recovery_until:
            if self._recovery_phase == "turn_nudge":
                self._recovery_wiggle_step += 1
                if self._recovery_wiggle_step < max(1, self.TURN_NUDGE_STEPS):
                    self._recovery_until = now + self.TURN_NUDGE_S
                else:
                    self._recovery_phase = ""
                    self._recovery_until = 0.0
                    self._recovery_cooldown_until = now + self.TURN_NUDGE_COOLDOWN_S
                    self._turn_in_place_active = False
                    self.wp_align_active = False
                    self.pid_yaw.reset()
                    self.pid_speed.reset()
                    self._kick_pwm = 0.0
                    self._auto_steer_mix_prev = 0.0
                    return False
            ending_phase = self._recovery_phase
            if self._recovery_phase == "obstacle_backup":
                self._recovery_phase = "obstacle_pivot"
                self._recovery_until = now + self.OBSTACLE_RECOVERY_PIVOT_S
            elif self._recovery_phase == "unstick_wiggle":
                self._recovery_wiggle_step += 1
                if self._recovery_wiggle_step < max(1, self.UNSTICK_WIGGLE_STEPS):
                    self._recovery_until = now + self.UNSTICK_WIGGLE_STEP_S
                else:
                    self._recovery_phase = ""
                    self._recovery_until = 0.0
                    self._recovery_cooldown_until = now + self.RECOVERY_COOLDOWN_S
                    self.pid_yaw.reset()
                    self.pid_speed.reset()
                    self._kick_pwm = 0.0
                    return False
            elif self._recovery_phase == "reverse":
                self._recovery_phase = "pivot"
                self._recovery_until = now + self.RECOVERY_PIVOT_S
            else:
                self._recovery_phase = ""
                self._recovery_until = 0.0
                cooldown = self.OBSTACLE_RECOVERY_COOLDOWN_S if ending_phase == "obstacle_pivot" else self.RECOVERY_COOLDOWN_S
                self._recovery_cooldown_until = now + cooldown
                self.pid_yaw.reset()
                self.pid_speed.reset()
                self._kick_pwm = 0.0
                return False

        if self._recovery_phase == "unstick_wiggle":
            outer = max(0, min(255, int(self.UNSTICK_WIGGLE_PWM)))
            inner = max(0, min(255, int(self.UNSTICK_WIGGLE_INNER_PWM)))
            # Forward arc wiggle: the body keeps crawling while alternately
            # loading each side, which frees soft-soil slips better than a
            # stationary pivot.
            turn_left = ((self._recovery_wiggle_step % 2) == 0)
            if self._recovery_turn_sign < 0.0:
                turn_left = not turn_left
            if turn_left:
                self.send_pwm(outer, inner)
            else:
                self.send_pwm(inner, outer)
            return True

        if self._recovery_phase == "obstacle_backup":
            self.send_pwm(-self.OBSTACLE_BACKUP_PWM, -self.OBSTACLE_BACKUP_PWM)
            return True

        if self._recovery_phase == "reverse":
            self.send_pwm(-self.RECOVERY_REVERSE_PWM, -self.RECOVERY_REVERSE_PWM)
            return True

        if self._recovery_phase == "turn_nudge":
            reverse = (self._recovery_wiggle_step % 2) == 1
            self._send_turn_arc(
                self._recovery_turn_sign,
                reverse=reverse,
                outer_pwm=self.TURN_NUDGE_OUTER_PWM,
                inner_pwm=self.TURN_NUDGE_INNER_PWM,
            )
            return True

        if self._recovery_phase == "obstacle_pivot":
            self._send_turn_arc(
                self._recovery_turn_sign,
                reverse=False,
                outer_pwm=self.OBSTACLE_RECOVERY_PIVOT_PWM,
                inner_pwm=max(self.TURN_ARC_INNER_PWM, int(self.OBSTACLE_RECOVERY_PIVOT_PWM * 0.55)),
            )
            return True

        self._send_turn_arc(
            self._recovery_turn_sign,
            reverse=False,
            outer_pwm=self.RECOVERY_PIVOT_PWM,
            inner_pwm=self.TURN_REVERSE_ARC_INNER_PWM,
        )
        return True

    def _send_turn_arc(self, turn_sign, reverse=False, outer_pwm=None, inner_pwm=None):
        """Turn while moving so the mower does not dig itself in with a static pivot."""
        if reverse:
            outer = max(0, min(255, int(outer_pwm if outer_pwm is not None else self.TURN_REVERSE_ARC_OUTER_PWM)))
            inner = max(0, min(255, int(inner_pwm if inner_pwm is not None else self.TURN_REVERSE_ARC_INNER_PWM)))
            if turn_sign >= 0.0:
                self.send_pwm(-inner, -outer)
            else:
                self.send_pwm(-outer, -inner)
            return

        outer = max(0, min(255, int(outer_pwm if outer_pwm is not None else self.TURN_ARC_OUTER_PWM)))
        inner = max(0, min(255, int(inner_pwm if inner_pwm is not None else self.TURN_ARC_INNER_PWM)))
        if turn_sign >= 0.0:
            self.send_pwm(outer, inner)
        else:
            self.send_pwm(inner, outer)

    def _send_stationary_pivot(self, turn_sign, pwm):
        """Rotate in place in logical robot space; hardware mapping happens in send_pwm."""
        mag = max(0, min(255, int(round(pwm))))
        if turn_sign >= 0.0:
            self.send_pwm(mag, -mag)
        else:
            self.send_pwm(-mag, mag)

    def _update_turn_boost(self, yaw_err, dt):
        turning_need = abs(yaw_err) >= self.turn_stall_err_rad
        yaw_rate_ok = abs(self.last_imu_yaw_rate) >= self.turn_stall_yaw_rate_rad_s

        if turning_need and (not yaw_rate_ok):
            self.turn_boost_pwm = min(
                float(self.TURN_IN_PLACE_MAX_PWM - self.TURN_IN_PLACE_PWM),
                self.turn_boost_pwm + self.turn_boost_ramp_per_s * dt
            )
        else:
            self.turn_boost_pwm = max(
                0.0,
                self.turn_boost_pwm - self.turn_boost_decay_per_s * dt
            )
        return self.turn_boost_pwm

    def _smooth_turn_pwm(self, yaw_err, yaw_pwm, boost=0.0, align=False):
        """Pivot PWM with bell-shaped torque and near-target damping."""
        err_deg = abs(math.degrees(yaw_err))
        exit_deg = math.degrees(self.wp_align_tol_rad if align else self.TURN_IN_PLACE_EXIT_RAD)
        base_min = min(self.WP_ALIGN_TURN_PWM if align else self.TURN_IN_PLACE_PWM, self.TURN_PROFILE_MIN_PWM)
        peak_pwm = max(base_min, min(self.TURN_IN_PLACE_MAX_PWM, self.TURN_PROFILE_PEAK_PWM))
        peak_err = max(exit_deg + 5.0, self.TURN_PROFILE_PEAK_ERR_DEG)
        sigma = max(8.0, self.TURN_PROFILE_SIGMA_DEG)

        # Bell-ish profile: soft near target, strongest around the useful pivot
        # zone, then moderate again at huge 120-180 deg errors to avoid digging
        # holes before the robot has actually begun rotating.
        bell = math.exp(-0.5 * ((err_deg - peak_err) / sigma) ** 2)
        if err_deg > peak_err:
            bell = max(bell, 0.55)
        else:
            ramp = max(0.0, min(1.0, (err_deg - exit_deg) / max(1.0, peak_err - exit_deg)))
            bell = max(bell, 0.20 * ramp)
        pwm = base_min + (peak_pwm - base_min) * bell
        pwm = max(pwm, min(peak_pwm, abs(yaw_pwm)))

        yaw_rate = abs(self.last_imu_yaw_rate)
        braking = abs(yaw_err) <= self.TURN_BRAKE_ERR_RAD and yaw_rate >= self.TURN_BRAKE_YAW_RATE_RAD_S
        needs_breakaway = (
            abs(yaw_err) >= self.TURN_BREAKAWAY_MIN_ERR_RAD
            and yaw_rate < self.turn_stall_yaw_rate_rad_s
        )
        if needs_breakaway and not braking:
            pwm = max(pwm, min(self.TURN_IN_PLACE_MAX_PWM, self.TURN_BREAKAWAY_PWM))
            pwm = min(self.TURN_IN_PLACE_MAX_PWM, pwm + boost)
        else:
            pwm = min(self.TURN_IN_PLACE_MAX_PWM, pwm + boost)

        # If it is already rotating and close to target, soften the command so
        # inertia does not carry it past the desired heading.
        min_allowed = base_min
        if braking:
            pwm *= max(0.35, min(1.0, self.TURN_BRAKE_SCALE))
            min_allowed = self.TURN_BRAKE_MIN_PWM

        return int(round(max(min_allowed, min(self.TURN_IN_PLACE_MAX_PWM, pwm))))

    def _turn_mix_sign(self, yaw_err, yaw_pwm, now):
        """Return pivot direction, with a fast guard for wrong-way rotation."""
        # Pivot direction must follow the live shortest yaw error, not filtered PID
        # output. Around +/-180 deg the normalized yaw error can flip sign from
        # tiny IMU/GPS noise, so latch the chosen direction until we are near the
        # target. That prevents left-right-left oscillation while turning around.
        steer_for_mix = -yaw_err if self.auto_pivot_steer_invert else yaw_err
        sign = 1.0 if steer_for_mix >= 0.0 else -1.0

        if abs(yaw_err) <= self.turn_latch_release_rad:
            self._turn_latch_sign = 0.0
            self._turn_direction_override_until = 0.0
            self._turn_direction_override_sign = 0.0
            self._turn_wrong_way_since = 0.0
            return sign

        if self._turn_latch_sign == 0.0:
            self._turn_latch_sign = sign
        elif abs(yaw_err) >= self.turn_latch_ambiguous_rad:
            # Near 180 deg both directions are nearly equivalent. Keep the first
            # choice instead of letting sign jitter reverse the robot every frame.
            sign = self._turn_latch_sign
        else:
            sign = self._turn_latch_sign

        # _turn_yaw_err_rate is positive when abs(error) is shrinking. Negative
        # means the robot is rotating away from the target, which is exactly the
        # failure seen in field AUTO start-align tests.
        wrong_way = (
            abs(yaw_err) > self.TURN_IN_PLACE_EXIT_RAD
            and self._turn_yaw_err_rate <= -self.turn_wrong_way_progress_rad_s
        )
        if wrong_way:
            if self._turn_wrong_way_since <= 0.0:
                self._turn_wrong_way_since = now
            elif (now - self._turn_wrong_way_since) >= self.turn_wrong_way_guard_s:
                self._turn_direction_override_sign = -sign
                self._turn_direction_override_until = now + self.turn_wrong_way_override_s
                self._turn_latch_sign = self._turn_direction_override_sign
                self._turn_wrong_way_since = 0.0
                self.steer_cmd_filt = 0.0
                self.get_logger().warn(
                    f"↩️ AUTO turn wrong-way guard: yaw_err={math.degrees(yaw_err):+.1f}deg "
                    f"progress={math.degrees(self._turn_yaw_err_rate):+.1f}deg/s "
                    f"flip_sign={self._turn_direction_override_sign:+.0f}"
                )
        else:
            self._turn_wrong_way_since = 0.0

        if now < self._turn_direction_override_until and self._turn_direction_override_sign != 0.0:
            return self._turn_direction_override_sign
        return self._turn_latch_sign if self._turn_latch_sign != 0.0 else sign

    def _shape_drive_pwm(self, pwm_l, pwm_r, yaw_err, steer):
        """Apply motor deadzone/tuning so command can actually move wheels."""
        turning_active = abs(yaw_err) > self.TURN_ACTIVE_YAW_RAD
        allow_pivot = abs(yaw_err) > self.PIVOT_ALLOW_YAW_RAD
        straight_scale = max(0.0, min(1.0, self.auto_speed_scale))
        straight_min_pwm = int(round(self.STRAIGHT_PWM_MIN))
        straight_max_pwm = int(round(
            self.STRAIGHT_PWM_MIN + (self.STRAIGHT_PWM_MAX - self.STRAIGHT_PWM_MIN) * straight_scale
        ))

        # Remove very small chatter commands.
        if pwm_l < self.PWM_NOISE_FLOOR:
            pwm_l = 0
        if pwm_r < self.PWM_NOISE_FLOOR:
            pwm_r = 0

        if turning_active:
            # Ensure outer wheel has enough torque during turn.
            if steer >= 0:
                pwm_r = max(pwm_r, int(self.MIN_TURN_OUTER_PWM))
            else:
                pwm_l = max(pwm_l, int(self.MIN_TURN_OUTER_PWM))

            # Keep inner wheel moving for normal turns (less "one wheel stopped" behavior).
            # Allow pivot turn only when heading error is very large.
            if not allow_pivot:
                if steer >= 0:
                    pwm_l = max(pwm_l, int(self.MIN_TURN_INNER_PWM))
                else:
                    pwm_r = max(pwm_r, int(self.MIN_TURN_INNER_PWM))

            # Ensure L/R differential is enough to rotate body.
            diff = abs(pwm_r - pwm_l)
            if diff < self.MIN_TURN_DIFF_PWM:
                need = int(self.MIN_TURN_DIFF_PWM - diff)
                if steer >= 0:
                    pwm_r = min(self.TURN_MAX_PWM, pwm_r + need)
                    pwm_l = max(0, pwm_l - need)
                else:
                    pwm_l = min(self.TURN_MAX_PWM, pwm_l + need)
                    pwm_r = max(0, pwm_r - need)

            # Anti-stall: if turning but encoder barely moves, add extra turn boost.
            if abs(self.last_enc_delta) < self.TURN_STALL_ENC_THRESH:
                if steer >= 0:
                    pwm_r = min(self.TURN_MAX_PWM, pwm_r + int(self.TURN_STALL_BOOST_PWM))
                    pwm_l = max(0, pwm_l - int(self.TURN_STALL_BOOST_PWM * 0.5))
                else:
                    pwm_l = min(self.TURN_MAX_PWM, pwm_l + int(self.TURN_STALL_BOOST_PWM))
                    pwm_r = max(0, pwm_r - int(self.TURN_STALL_BOOST_PWM * 0.5))

            if not allow_pivot:
                if steer >= 0:
                    pwm_l = max(pwm_l, int(self.MIN_TURN_INNER_PWM))
                else:
                    pwm_r = max(pwm_r, int(self.MIN_TURN_INNER_PWM))
        else:
            # Straight/near-straight: allow web-adjusted cruise speed, but keep enough floor to move wheels.
            if pwm_l > 0:
                pwm_l = max(pwm_l, straight_min_pwm)
                pwm_l = min(pwm_l, straight_max_pwm)
            if pwm_r > 0:
                pwm_r = max(pwm_r, straight_min_pwm)
                pwm_r = min(pwm_r, straight_max_pwm)

        max_cap = self.TURN_MAX_PWM if turning_active else straight_max_pwm
        pwm_l = max(0, min(max_cap, int(pwm_l)))
        pwm_r = max(0, min(max_cap, int(pwm_r)))
        return pwm_l, pwm_r

    # =================================================
    # IO
    # =================================================
    def _slew_pwm_axis(self, target, prev, dt, up_rate, down_rate):
        dt = max(0.001, dt)
        # When changing direction, bleed to zero first to avoid drivetrain jerk.
        if target != 0.0 and prev != 0.0 and ((target > 0.0) != (prev > 0.0)):
            zero_step = self.PWM_ZERO_CROSS_PER_S * dt
            if prev > 0.0:
                return max(0.0, prev - zero_step)
            return min(0.0, prev + zero_step)

        rate = up_rate if abs(target) >= abs(prev) else down_rate
        step = rate * dt
        if target > prev:
            return min(target, prev + step)
        return max(target, prev - step)

    def _map_jetson_pwm_output(self, l, r):
        # Fixed mapping for the current Arduino firmware/wiring:
        # FWD(+,+)->(-,+), BACK(-,-)->(+,-), LEFT(-,+)->(-,-), RIGHT(+,-)->(+,+).
        l_cmd = -int(r)
        r_cmd = int(l)

        return (
            max(-255, min(255, l_cmd)),
            max(-255, min(255, r_cmd)),
        )

    def send_pwm(self, l, r):
        # Keep AUTO/FOLLOW control semantics in logical robot space:
        # +,+ = forward, -,- = reverse, opposite signs = pivot.
        # Hardware polarity mapping is applied only after smoothing.
        l_target = max(-255, min(255, int(l)))
        r_target = max(-255, min(255, int(r)))
        cap = 255
        if self.mode_web == "START" and self.mode_hardware == HW_AUTO:
            cap = max(0, min(255, int(self.AUTO_PWM_ABS_CAP)))
        elif self.mode_web == "START" and self.mode_hardware == HW_FOLLOW:
            cap = max(0, min(255, int(self.FOLLOW_PWM_ABS_CAP)))
        if cap < 255:
            capped_l = max(-cap, min(cap, l_target))
            capped_r = max(-cap, min(cap, r_target))
            if capped_l != l_target or capped_r != r_target:
                now_log = time.time()
                if now_log - getattr(self, "_last_pwm_cap_log_ts", 0.0) > 1.0:
                    self._last_pwm_cap_log_ts = now_log
                    mode_name = "AUTO" if self.mode_hardware == HW_AUTO else "FOLLOW"
                    self.get_logger().warn(
                        f"PWM capped in {mode_name}: ({l_target},{r_target}) -> ({capped_l},{capped_r}) cap={cap}"
                    )
            l_target, r_target = capped_l, capped_r

        now = time.time()
        dt = now - self._last_pwm_send_ts if self._last_pwm_send_ts > 0 else 0.05
        self._last_pwm_send_ts = now
        if l_target == 0 and r_target == 0:
            self._pwm_cmd_l = 0.0
            self._pwm_cmd_r = 0.0
        else:
            turn_mode = (l_target * r_target < 0) or (abs(l_target - r_target) > 70)
            up_rate = self.PWM_TURN_UP_PER_S if turn_mode else self.PWM_DRIVE_UP_PER_S
            down_rate = self.PWM_TURN_DOWN_PER_S if turn_mode else self.PWM_DRIVE_DOWN_PER_S
            self._pwm_cmd_l = self._slew_pwm_axis(float(l_target), self._pwm_cmd_l, dt, up_rate, down_rate)
            self._pwm_cmd_r = self._slew_pwm_axis(float(r_target), self._pwm_cmd_r, dt, up_rate, down_rate)
        l_logical = int(round(self._pwm_cmd_l))
        r_logical = int(round(self._pwm_cmd_r))
        l_out, r_out = self._map_jetson_pwm_output(l_logical, r_logical)

        self.last_pwm_l = l_out
        self.last_pwm_r = r_out
        self.send_serial(f"{l_out},{r_out}")

    def send_serial(self, s):
        try:
            if not self.arduino_connected:
                return
            if not self._arduino_rx_seen:
                # Keep the serial line quiet until the first Arduino frame is
                # received. This prevents a half-open CDC port from trapping the
                # main loop in write timeouts before reconnect can recover.
                return
            is_pwm_frame = re.match(r"^-?\d+,-?\d+$", str(s).strip()) is not None
            if is_pwm_frame and (time.time() - self._last_serial_write_timeout_ts) < 0.25:
                return
            if s == "EMG" or s == "RST" or s.startswith("MANUAL,"):
                self.get_logger().info(f"[TX->ARDUINO] {s}")
            self.ser.write((s + "\n").encode())
            self._arduino_write_timeout_count = 0
        except serial.SerialTimeoutException as e:
            self._last_serial_write_timeout_ts = time.time()
            self._arduino_write_timeout_count += 1
            now = self._last_serial_write_timeout_ts
            if now - self._last_serial_err_log_ts > 1.0:
                self._last_serial_err_log_ts = now
                self.get_logger().warn(f"⚠️ Arduino write timeout: {e}")
            if (
                (not self._arduino_rx_seen)
                and self._arduino_write_timeout_count >= int(os.getenv("ARDUINO_USB_RESET_TIMEOUTS", "4"))
                and (now - self._last_arduino_usb_reset_ts) > 20.0
            ):
                self._last_arduino_usb_reset_ts = now
                port = getattr(self.ser, "port", "")
                try:
                    if getattr(self, "ser", None) is not None and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.arduino_connected = False
                self._arduino_rx_seen = False
                self._arduino_write_timeout_count = 0
                self._usb_reset_serial_device(port)
        except Exception as e:
            self.arduino_connected = False
            self._arduino_rx_seen = False
            self._arduino_write_timeout_count = 0
            try:
                if getattr(self, "ser", None) is not None and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            now = time.time()
            if now - self._last_serial_err_log_ts > 1.0:
                self._last_serial_err_log_ts = now
                self.get_logger().error(f"❌ Arduino write error: {e}")

    # =================================================
    # PUBLISH
    # =================================================
    def publish_all(self):
        if self.gps_x is None or self.gps_y is None:
            return

        if self.emergency_latched:
            hw = HW_EMER
        else:
            hw = self.mode_hardware

        pub_lat = self.gps_x
        pub_lon = self.gps_y
        if (
            self.gps_control_mode == "START_ONLY"
            and self.gps_start_only_active
            and self.gps_raw_x is not None
            and self.gps_raw_y is not None
            and not self._gps_invalid(self.gps_raw_x, self.gps_raw_y)
        ):
            pub_lat = self.gps_raw_x
            pub_lon = self.gps_raw_y

        target_lat = float("nan")
        target_lon = float("nan")
        if self.path_points and 0 <= self.current_idx < len(self.path_points):
            try:
                target_lat = float(self.path_points[self.current_idx][0])
                target_lon = float(self.path_points[self.current_idx][1])
            except Exception:
                target_lat = float("nan")
                target_lon = float("nan")

        pwm_pub_l = self.arduino_pwm_l if self.arduino_pwm_valid else self.last_pwm_l
        pwm_pub_r = self.arduino_pwm_r if self.arduino_pwm_valid else self.last_pwm_r

        seg_from, seg_to = self._current_user_segment()
        msg = Float32MultiArray()
        msg.data = [
            float(pub_lat),           # [0]
            float(pub_lon),           # [1]
            float(self.yaw_ui),       # [2]
            float(hw),                # [3]
            float(self.gps_fix),      # [4]
            1.0 if self.gps_ready else 0.0,        # [5]
            float(len(self.gps_init_samples)),      # [6]
            float(self._gps_init_spread_m),         # [7]
            float(self._gps_jump_drop_count),       # [8]
            float(self.lidar_min_front_m if math.isfinite(self.lidar_min_front_m) else -1.0),  # [9]
            float(self.safety_distance_cm),         # [10]
            1.0 if (
                self.lidar_raw_hard_stop_active
                or (self.auto_lidar_avoid_enable and self.lidar_safety_active())
            ) else 0.0,   # [11]
            1.0 if self.gps_calibrating else 0.0,         # [12]
            float(pwm_pub_l),         # [13]
            float(pwm_pub_r),         # [14]
            1.0 if self.lidar_safety_enabled else 0.0,    # [15]
            float(self.yaw_offset_deg),   # [16]
            float(self.lidar_max_use_m),  # [17]
            float(self.enc_l_raw),        # [18]
            float(self.enc_r_raw),        # [19]
            1.0 if self.gps_control_mode == "START_ONLY" else 0.0,  # [20]
            float(self.auto_speed_scale * 100.0),   # [21]
            1.0 if self.arduino_pump_on else 0.0,   # [22]
            float(self.arduino_blade_state),        # [23]
            1.0 if self.arduino_manual_led else 0.0,   # [24]
            1.0 if self.arduino_auto_lamp else 0.0,    # [25]
            1.0 if self.arduino_emg_flag else 0.0,     # [26]
            float(self.arduino_can_soc),         # [27]
            float(self.arduino_can_voltage),     # [28]
            float(self.arduino_can_current),     # [29]
            float(self.arduino_can_temperature), # [30]
            float(self.arduino_can_status),      # [31]
            float(self.current_idx),             # [32]
            float(len(self.path_points)),        # [33]
            target_lat,                          # [34]
            target_lon,                          # [35]
            float(self.arduino_speed_l_mps),     # [36]
            float(self.arduino_speed_r_mps),     # [37]
            float(self.arduino_speed_avg_mps),   # [38]
            float(self.gps_num_sats),            # [39] satellite count
            float(seg_from),                     # [40] user segment from-index (-1=none)
            float(seg_to),                       # [41] user segment to-index (-1=none)
        ]
        self.gps_pub.publish(msg)

        mode_msg = Float32()
        mode_msg.data = float(hw)
        self.robot_status_mode_pub.publish(mode_msg)

        enc_l_msg = Float32()
        enc_l_msg.data = float(self.enc_l_raw)
        self.robot_status_enc_l_pub.publish(enc_l_msg)

        enc_r_msg = Float32()
        enc_r_msg.data = float(self.enc_r_raw)
        self.robot_status_enc_r_pub.publish(enc_r_msg)

        pump_msg = Bool()
        pump_msg.data = bool(self.arduino_pump_on)
        self.robot_status_pump_pub.publish(pump_msg)

        blade_msg = Float32()
        blade_msg.data = float(self.arduino_blade_state)
        self.robot_status_blade_pub.publish(blade_msg)

        emg_msg = Bool()
        emg_msg.data = bool(self.arduino_emg_flag or self.emergency_latched)
        self.robot_status_emg_pub.publish(emg_msg)

        soc_msg = Float32()
        soc_msg.data = float(self.arduino_can_soc)
        self.robot_battery_soc_pub.publish(soc_msg)

        voltage_msg = Float32()
        voltage_msg.data = float(self.arduino_can_voltage)
        self.robot_battery_voltage_pub.publish(voltage_msg)

        current_msg = Float32()
        current_msg.data = float(self.arduino_can_current)
        self.robot_battery_current_pub.publish(current_msg)

        temp_msg = Float32()
        temp_msg.data = float(self.arduino_can_temperature)
        self.robot_battery_temp_pub.publish(temp_msg)

        status_msg = Float32()
        status_msg.data = float(self.arduino_can_status)
        self.robot_battery_status_pub.publish(status_msg)

    # =================================================
    # DEBUG
    # =================================================
    def _current_user_segment(self):
        """Return (from_user_idx, to_user_idx) for the active segment, or (-1, -1)."""
        if not self.path_points or not self.path_corner_indices or self.current_idx <= 0:
            return -1, -1
        sorted_c = sorted(self.path_corner_indices)
        for i in range(len(sorted_c) - 1):
            if self.current_idx <= sorted_c[i + 1]:
                return i, i + 1
        last = len(sorted_c) - 1
        return max(0, last - 1), last

    def debug_log(self):
        now = time.time()
        if now - self._dbg_ts > 0.3:
            self._dbg_ts = now
            age = now - self.last_gps_time if self.last_gps_time > 0 else -1
            name = {0:"MAN",1:"AUTO",2:"FOLLOW",3:"EMER"}.get(
                HW_EMER if self.emergency_latched else self.mode_hardware
            )
            self.get_logger().info(
                f"[MODE:{name} WEB:{self.mode_web} EMG:{self.emergency_latched}] "
                f"Yaw={math.degrees(self.yaw):6.1f}deg | "
                f"GPSf=({self.gps_x},{self.gps_y}) "
                f"GPSr=({self.gps_raw_x},{self.gps_raw_y}) "
                f"fix={self.gps_fix} ready={self.gps_ready} warmup_n={len(self.gps_init_samples)} "
                f"spread={self._gps_init_spread_m:.2f}m age={age:.2f}s drop={self._gps_jump_drop_count} "
                f"v={self.speed_mps:.2f}m/s corr={self._fusion_last_corr_m:.2f}m "
                f"lidar_min={self.lidar_min_front_m:.2f}m safety={self.safety_distance_cm:.0f}cm enabled={self.lidar_safety_enabled} obs={self.lidar_safety_active()} "
                f"arduino_age={(now - self._last_arduino_rx_ts):.2f}s | "
                f"PWM=({self.last_pwm_l},{self.last_pwm_r})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = LawnmowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
