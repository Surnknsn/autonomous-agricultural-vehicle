#!/usr/bin/env python3
"""
lawnmower_node.py — โหนดหลักของหุ่นยนต์ตัดหญ้าอัตโนมัติ (LawnmowerNode)

ภาพรวมระบบ:
- รับคำสั่งจาก Web UI ผ่าน ROS topic /web_mode
- ควบคุมการเคลื่อนที่ 3 โหมด: MANUAL (ควบคุมมือ) / AUTO (เดินตาม waypoint) / FOLLOW (ตามคน)
- อ่านค่า yaw/GPS จาก Pixhawk ผ่าน MAVLink, อ่าน encoder ล้อจาก Arduino ผ่าน Serial
- ส่งคำสั่ง PWM ไปยัง Arduino เพื่อขับมอเตอร์ซ้าย-ขวา
- ใช้ PID controller สำหรับควบคุมทิศทางและความเร็วในโหมด AUTO
- มีระบบ dead reckoning (ประมาณตำแหน่งจาก encoder+IMU) เมื่อ GPS สัญญาณอ่อน
- ตรวจจับการติดขัด (stuck detection) และมีระบบ recovery อัตโนมัติ
- รองรับ LIDAR safety: หยุดเมื่อตรวจพบสิ่งกีดขวางในระยะอันตราย

ROS Topics ที่ใช้:
    Subscribe:
        /scan            — ข้อมูล LIDAR (LaserScan)
        /web_mode        — คำสั่งจาก Web UI (String)
        /waypoints       — รายการ waypoint สำหรับ AUTO (String/JSON)
        /manual_cmd      — คำสั่งควบคุมมือ (String)
        /emergency       — สัญญาณฉุกเฉิน (Bool)
        /follow_cmd      — คำสั่งจาก follow tracker (String)

    Publish:
        /robot_status    — สถานะปัจจุบันของหุ่นยนต์ (String/JSON)
        /odom_data       — ข้อมูล odometry (Float32MultiArray)
"""
import glob
import json
import math
import os
import re
import time
from collections import deque
import statistics

import rclpy
from rclpy.node import Node
import serial
from pymavlink import mavutil
from std_msgs.msg import Float32MultiArray, String, Float32, Bool
from sensor_msgs.msg import LaserScan

# =========================
# นิยามโหมดฮาร์ดแวร์ (ค่าคงที่จาก Arduino, ห้ามเปลี่ยน)
# =========================
# 0 = MANUAL  — ควบคุมด้วยมือผ่าน RC หรือ Web UI
# 1 = AUTO    — เดินตาม waypoint อัตโนมัติ
# 2 = FOLLOW  — ตามคน (ใช้ follow_tracker_node ร่วมกัน)
# 3 = EMERGENCY — โหมดฉุกเฉิน (กำหนดในซอฟต์แวร์ Jetson เท่านั้น)

HW_MANUAL = 0
HW_AUTO   = 1
HW_FOLLOW = 2
HW_EMER   = 3


def normalize_angle(a):
    """ปรับมุม a (radian) ให้อยู่ในช่วง [-π, π] โดยใช้ atan2"""
    return math.atan2(math.sin(a), math.cos(a))


# =========================
# PID CONTROLLER — ตัวควบคุม PID ทั่วไป
# =========================
class PID:
    """
    ตัวควบคุม PID (Proportional-Integral-Derivative)

    ใช้สำหรับ:
    - pid_yaw: ควบคุมทิศทางหัวหุ่นยนต์ให้ตรงกับ waypoint ในโหมด AUTO
    - pid_speed: ควบคุมความเร็วให้คงที่ในโหมด AUTO

    คุณสมบัติ:
    - รองรับ output clamping ผ่าน limit (ป้องกัน PWM เกินขอบเขต)
    - reset() ใช้เมื่อเริ่ม mission ใหม่ เพื่อล้าง integral windup
    """

    def __init__(self, kp, ki, kd, limit=None):
        """
        kp — Proportional gain (ตอบสนองต่อ error ปัจจุบัน)
        ki — Integral gain (ชดเชย error สะสม เช่น แรงเสียดทาน)
        kd — Derivative gain (ลด overshoot)
        limit — clamp output ให้อยู่ใน [-limit, +limit] (None = ไม่จำกัด)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.i = 0.0      # integral term สะสม
        self.prev = 0.0   # error รอบที่แล้ว (ใช้คำนวณ derivative)

    def reset(self):
        """รีเซ็ต integral และ derivative term — เรียกเมื่อเริ่ม mission ใหม่"""
        self.i = 0.0
        self.prev = 0.0

    def step(self, err, dt):
        """
        คำนวณ output PID ขั้นตอนเดียว

        err — error ปัจจุบัน (เช่น yaw error เป็น radian)
        dt  — ระยะเวลาตั้งแต่ step ก่อน (วินาที)
        return — output PWM correction (จำกัดด้วย limit ถ้ามี)
        """
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
            "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_9513133313735151A2F0-if00",
            "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24336303633351411171-if00",
        ]
        # Optional hard lock for Arduino port, e.g. ARDUINO_PORT=/dev/ttyACM0
        # to avoid accidental attach to wrong ACM device.
        arduino_port_override = os.getenv("ARDUINO_PORT", "").strip()
        if arduino_port_override:
            self.ENCODER_PORT_CANDIDATES = [arduino_port_override] + self.ENCODER_PORT_CANDIDATES
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
        self.arduino_can_invalid_hold_s = float(os.getenv("CAN_INVALID_HOLD_S", "4.0"))
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
        # Straight driving can move around PWM 70 on current LiFePO4 setup.
        # Keep turn PWM separate/high so steering still has enough torque.
        self.MIN_PWM = 70.0
        self.STRAIGHT_PWM_MIN = float(os.getenv("AUTO_STRAIGHT_PWM_MIN", "70"))
        self.STRAIGHT_PWM_MAX = float(os.getenv("AUTO_STRAIGHT_PWM_MAX", "100"))
        self.DRIVE_MAX_PWM = int(self.STRAIGHT_PWM_MAX)
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
        self.DRIVE_STALL_BOOST_RAMP = float(os.getenv("DRIVE_STALL_BOOST_RAMP", "45.0"))
        self.DRIVE_STALL_BOOST_DECAY = float(os.getenv("DRIVE_STALL_BOOST_DECAY", "120.0"))
        self.DRIVE_STALL_BOOST_MAX = float(os.getenv("DRIVE_STALL_BOOST_MAX", "35.0"))
        self.DRIVE_STALL_BOOST_DELAY_S = float(os.getenv("DRIVE_STALL_BOOST_DELAY_S", "0.55"))
        self.DRIVE_STALL_BOOST_CAP_PWM = float(os.getenv("DRIVE_STALL_BOOST_CAP_PWM", "125.0"))
        self.TURN_MAX_PWM = 255
        self.PWM_DRIVE_UP_PER_S = 120.0
        self.PWM_DRIVE_DOWN_PER_S = 180.0
        # Pivot turns need to overcome sand/static friction quickly; slow ramp
        # digs a groove before the body rotates. Down-ramp stays controlled so
        # the robot does not snap back and overshoot heading.
        self.PWM_TURN_UP_PER_S = float(os.getenv("PWM_TURN_UP_PER_S", "1800.0"))
        self.PWM_TURN_DOWN_PER_S = float(os.getenv("PWM_TURN_DOWN_PER_S", "900.0"))
        self.PWM_ZERO_CROSS_PER_S = 1800.0
        self._pwm_cmd_l = 0.0
        self._pwm_cmd_r = 0.0
        self._last_pwm_send_ts = time.time()

        self.mode_hardware = HW_MANUAL   # จาก Arduino
        self._mode_candidate = self.mode_hardware
        self._mode_candidate_count = 0
        # Same debounce for all mode transitions to avoid MAN-sticky behavior.
        self.MODE_DEBOUNCE_COUNT_ACTIVE = 2
        self.MODE_DEBOUNCE_COUNT_MANUAL = 2
        self._last_arduino_rx_ts = 0.0
        self._last_mode_rx_ts = 0.0
        self._last_serial_err_log_ts = 0.0
        self._last_serial_parse_warn_ts = 0.0
        self._last_mode_parse_debug_ts = 0.0
        self._last_arduino_verbose_log_ts = 0.0
        self._warned_fw_old_log_format = False
        self.arduino_connected = False
        # Serial can pause briefly while USB/MCU handles mode-switch noise.
        # Keep timeout relaxed to avoid false reconnect storms.
        self.arduino_rx_timeout_s = float(os.getenv("ARDUINO_RX_TIMEOUT_S", "2.0"))
        self._last_arduino_retry_ts = 0.0
        self._last_arduino_state_warn_ts = 0.0
        self.mode_web = "STOP"           # จาก Web
        self.web_run_mode = "AUTO"       # AUTO / FOLLOW; START alone is treated as AUTO-safe.
        self.emergency_latched = False
        # Slip-tolerant waypoint tracking for real ground conditions.
        self.WAYPOINT_REACH_M = float(os.getenv("WP_REACH_M", "0.95"))
        self.WAYPOINT_TURN_REACH_M = float(os.getenv("WP_TURN_REACH_M", "0.85"))
        self.WAYPOINT_TURN_ANGLE_RAD = math.radians(float(os.getenv("WP_TURN_ANGLE_DEG", "55.0")))
        self.SEGMENT_LOOKAHEAD_M = float(os.getenv("SEGMENT_LOOKAHEAD_M", "2.35"))
        self.SEGMENT_LOOKAHEAD_MIN_M = float(os.getenv("SEGMENT_LOOKAHEAD_MIN_M", "1.35"))
        self.CROSSTRACK_TIGHTEN_M = float(os.getenv("CROSSTRACK_TIGHTEN_M", "1.55"))
        self.CROSSTRACK_LOOKAHEAD_M = float(os.getenv("CROSSTRACK_LOOKAHEAD_M", "1.90"))
        self.GPS_CROSSTRACK_DEADBAND_M = float(os.getenv("GPS_CROSSTRACK_DEADBAND_M", "0.30"))
        self.GPS_CROSSTRACK_MAX_CORR_M = float(os.getenv("GPS_CROSSTRACK_MAX_CORR_M", "0.85"))
        self.WAYPOINT_REACH_MAX_CROSSTRACK_M = float(os.getenv("WP_REACH_MAX_CROSSTRACK_M", "0.50"))
        self.IMU_SEGMENT_HEADING_ENABLE = os.getenv("AUTO_IMU_SEGMENT_HEADING", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_SEGMENT_HEADING_LOCK = os.getenv("AUTO_SEGMENT_HEADING_LOCK", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_DR_REACH_ENABLE = os.getenv("AUTO_DR_REACH_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_DR_PASS_MARGIN_M = float(os.getenv("AUTO_DR_PASS_MARGIN_M", "0.20"))
        self.AUTO_DR_FINAL_PASS_MARGIN_M = float(os.getenv("AUTO_DR_FINAL_PASS_MARGIN_M", "0.90"))
        self.AUTO_DR_FINAL_MAX_CROSSTRACK_M = float(os.getenv("AUTO_DR_FINAL_MAX_CROSSTRACK_M", "1.50"))
        self.AUTO_DR_PASS_REQUIRE_GPS_CORRIDOR = os.getenv("AUTO_DR_PASS_REQUIRE_GPS_CORRIDOR", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_GPS_CORRIDOR_CONFIRM_M = float(os.getenv("AUTO_GPS_CORRIDOR_CONFIRM_M", "1.80"))
        self.AUTO_GPS_PASS_WINDOW_M = float(os.getenv("AUTO_GPS_PASS_WINDOW_M", "1.25"))
        self.IMU_SEGMENT_MIN_REMAIN_M = float(os.getenv("AUTO_IMU_SEGMENT_MIN_REMAIN_M", "1.20"))
        self.IMU_SEGMENT_MAX_CROSSTRACK_M = float(os.getenv("AUTO_IMU_SEGMENT_MAX_CROSSTRACK_M", "2.50"))
        # Optional breadcrumbs for very long map segments. Keep this off by
        # default so line-path follows the user's original points directly.
        self.PATH_DENSIFY_ENABLE = os.getenv("PATH_DENSIFY_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.PATH_DENSIFY_SPACING_M = float(os.getenv("PATH_DENSIFY_SPACING_M", "1.25"))
        self.PATH_DENSE_REACH_M = float(os.getenv("PATH_DENSE_REACH_M", "0.50"))
        self.PATH_DENSIFY_MIN_SEG_M = float(os.getenv("PATH_DENSIFY_MIN_SEG_M", "0.08"))
        self.PATH_MIN_USER_POINT_SPACING_M = float(os.getenv("PATH_MIN_USER_POINT_SPACING_M", "0.75"))
        # Keep AUTO close to the drawn path using IMU/yaw feedback.
        # This is path correction, not left/right motor-side compensation.
        self.HEADING_DEADBAND_RAD = math.radians(float(os.getenv("AUTO_HEADING_DEADBAND_DEG", "10.0")))
        self.AUTO_LINE_STRAIGHT_HOLD_RAD = math.radians(float(os.getenv("AUTO_LINE_STRAIGHT_HOLD_DEG", "24.0")))
        self.MIN_FORWARD_PWM = self.STRAIGHT_PWM_MIN
        self.TURN_RATIO_MAX = 0.30
        self.STRAIGHT_HOLD_MIN_STEER_PWM = float(os.getenv("AUTO_STRAIGHT_HOLD_MIN_STEER_PWM", "0.0"))
        self.AUTO_STEER_MIX_SLEW_PWM_PER_S = float(os.getenv("AUTO_STEER_MIX_SLEW_PWM_PER_S", "10.0"))
        self._auto_steer_mix_prev = 0.0
        self.MIN_TURN_OUTER_PWM = float(os.getenv("AUTO_MIN_TURN_OUTER_PWM", "140.0"))
        self.MIN_TURN_INNER_PWM = float(os.getenv("AUTO_MIN_TURN_INNER_PWM", "95.0"))
        self.MIN_TURN_DIFF_PWM = 42.0
        self.TURN_ACTIVE_YAW_RAD = math.radians(38.0)
        self.PIVOT_ALLOW_YAW_RAD = math.radians(95.0)
        self.PWM_NOISE_FLOOR = 18
        self.TURN_STALL_BOOST_PWM = 16
        self.TURN_STALL_ENC_THRESH = self.ENC_FREE_GAP_PULSES
        # If heading error is large, rotate in place first before moving forward.
        self.TURN_IN_PLACE_ENTER_RAD = math.radians(100.0)
        self.TURN_IN_PLACE_EXIT_RAD = math.radians(22.0)
        self.TURN_IN_PLACE_PWM = float(os.getenv("AUTO_TURN_IN_PLACE_PWM", "125.0"))
        self.TURN_IN_PLACE_MAX_PWM = float(os.getenv("AUTO_TURN_IN_PLACE_MAX_PWM", "255.0"))
        self.TURN_BREAKAWAY_PWM = float(os.getenv("AUTO_TURN_BREAKAWAY_PWM", "255.0"))
        self.WP_ALIGN_TURN_PWM = float(os.getenv("AUTO_WP_ALIGN_TURN_PWM", "120.0"))
        self.TURN_BRAKE_MIN_PWM = float(os.getenv("AUTO_TURN_BRAKE_MIN_PWM", "85.0"))
        self.TURN_PROFILE_MIN_PWM = float(os.getenv("AUTO_TURN_PROFILE_MIN_PWM", "115.0"))
        self.TURN_PROFILE_PEAK_PWM = float(os.getenv("AUTO_TURN_PROFILE_PEAK_PWM", "205.0"))
        self.TURN_PROFILE_PEAK_ERR_DEG = float(os.getenv("AUTO_TURN_PROFILE_PEAK_ERR_DEG", "70.0"))
        self.TURN_PROFILE_SIGMA_DEG = float(os.getenv("AUTO_TURN_PROFILE_SIGMA_DEG", "42.0"))
        self.AUTO_CORNER_SLOWDOWN_DIST_M = float(os.getenv("AUTO_CORNER_SLOWDOWN_DIST_M", "3.0"))
        self.AUTO_CORNER_APPROACH_PWM = float(os.getenv("AUTO_CORNER_APPROACH_PWM", "78.0"))
        self.TURN_BREAKAWAY_MIN_ERR_RAD = math.radians(float(os.getenv("AUTO_TURN_BREAKAWAY_MIN_ERR_DEG", "20.0")))
        self.TURN_BRAKE_ERR_RAD = math.radians(float(os.getenv("AUTO_TURN_BRAKE_ERR_DEG", "28.0")))
        self.TURN_BRAKE_YAW_RATE_RAD_S = math.radians(float(os.getenv("AUTO_TURN_BRAKE_YAW_RATE_DEG_S", "16.0")))
        self.TURN_BRAKE_SCALE = float(os.getenv("AUTO_TURN_BRAKE_SCALE", "0.55"))
        self.FORWARD_BLOCK_YAW_RAD = math.radians(105.0)
        self.HEADING_COMMIT_ENTER_RAD = math.radians(95.0)
        self.HEADING_COMMIT_EXIT_RAD = math.radians(28.0)
        self._heading_commit_active = False
        self.turn_boost_pwm = 0.0
        self.turn_boost_ramp_per_s = 65.0
        self.turn_boost_decay_per_s = 120.0
        self.turn_stall_yaw_rate_rad_s = math.radians(3.0)
        self.turn_stall_err_rad = math.radians(25.0)
        self.turn_wrong_way_progress_rad_s = math.radians(float(os.getenv("AUTO_TURN_WRONG_WAY_DEG_S", "10.0")))
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
        self.auto_arc_steer_invert = os.getenv("AUTO_ARC_STEER_INVERT", "1").lower() in ("1", "true", "yes", "on")
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
        self.wp_stop_settle_s = float(os.getenv("WP_STOP_SETTLE_S", "0.12"))
        self.wp_align_tol_rad = math.radians(float(os.getenv("WP_ALIGN_TOL_DEG", "14.0")))
        self.wp_align_active = False
        self.wp_settle_until = 0.0
        self.WAYPOINT_CONFIRM_M = float(os.getenv("WP_CONFIRM_M", "0.85"))
        self.wp_reach_hold_s = float(os.getenv("WP_REACH_HOLD_S", "0.30"))
        self.gps_reach_hold_s = float(os.getenv("GPS_REACH_HOLD_S", "0.10"))
        self.GPS_WAYPOINT_CONFIRM_WITH_DR = os.getenv("GPS_WP_CONFIRM_WITH_DR", "0").lower() in ("1", "true", "yes", "on")
        self.GPS_WP_CONFIRM_DR_WINDOW_M = float(os.getenv("GPS_WP_CONFIRM_DR_WINDOW_M", "1.25"))
        self.GPS_WP_CONFIRM_DR_MAX_CROSSTRACK_M = float(os.getenv("GPS_WP_CONFIRM_DR_MAX_CROSSTRACK_M", "3.00"))
        self.AUTO_START_SNAP_ENABLE = os.getenv("AUTO_START_SNAP_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_START_SNAP_MAX_CROSSTRACK_M = float(os.getenv("AUTO_START_SNAP_MAX_CROSSTRACK_M", "0.65"))
        self.AUTO_START_SNAP_MIN_GAIN_M = float(os.getenv("AUTO_START_SNAP_MIN_GAIN_M", "2.50"))
        self.AUTO_ALIGN_ON_START = os.getenv("AUTO_ALIGN_ON_START", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_REJOIN_LINE_ENABLE = os.getenv("AUTO_REJOIN_LINE_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.AUTO_WP_DIRECT_APPROACH_M = float(os.getenv("AUTO_WP_DIRECT_APPROACH_M", "2.20"))
        self.AUTO_FORCE_USER_WP_CORNERS = os.getenv("AUTO_FORCE_USER_WP_CORNERS", "1").lower() in ("1", "true", "yes", "on")
        self._wp_reach_candidate_idx = -1
        self._wp_reach_candidate_since = 0.0
        self.DRIVE_STUCK_SPEED_MAX = 0.03
        self.DRIVE_STUCK_PROGRESS_MIN_MPS = float(os.getenv("DRIVE_STUCK_PROGRESS_MIN_MPS", "0.025"))
        self.DRIVE_STUCK_PROGRESS_MIN_DIST_M = float(os.getenv("DRIVE_STUCK_PROGRESS_MIN_DIST_M", "0.90"))
        self.DRIVE_STUCK_DETECT_S = 2.2
        self.TURN_STUCK_DETECT_S = 1.4
        self.AUTO_STUCK_RECOVERY_ENABLE = os.getenv("AUTO_STUCK_RECOVERY_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.AUTO_SLIP_PANIC_ENABLE = os.getenv("AUTO_SLIP_PANIC_ENABLE", "0").lower() in ("1", "true", "yes", "on")
        self.TURN_STUCK_YAW_PROGRESS_RAD_S = math.radians(float(os.getenv("TURN_STUCK_YAW_PROGRESS_DEG_S", "2.0")))
        self.SLIP_SIDE_RATIO_MAX = 0.28
        self.RECOVERY_REVERSE_PWM = 175
        self.RECOVERY_REVERSE_S = 0.35
        self.RECOVERY_PIVOT_PWM = 225
        self.RECOVERY_PIVOT_S = 0.45
        self.RECOVERY_COOLDOWN_S = 0.9
        self.SLIP_PANIC_PWM = int(os.getenv("SLIP_PANIC_PWM", "255"))
        self.SLIP_PANIC_HOLD_S = float(os.getenv("SLIP_PANIC_HOLD_S", "1.35"))
        self.SLIP_PANIC_S = float(os.getenv("SLIP_PANIC_S", "0.30"))
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._recovery_phase = ""
        self._recovery_until = 0.0
        self._recovery_turn_sign = 1.0
        self._recovery_cooldown_until = 0.0
        self._slip_panic_since = 0.0
        self._slip_panic_until = 0.0
        self._path_progress_rate_mps = 0.0
        self._last_progress_dist_to_wp = None
        self._last_progress_idx = -1
        self._turn_yaw_err_rate = 0.0
        self._last_turn_yaw_err_abs = None
        self.auto_speed_scale = max(0.0, min(1.0, float(os.getenv("AUTO_SPEED_DEFAULT_PCT", "25")) / 100.0))

        # ---------- PWM DEBUG ----------
        self.last_pwm_l = 0
        self.last_pwm_r = 0

        self.raw_path_points = []
        self.path_points = []
        self.path_corner_indices = set()
        self.current_idx = 0
        self._auto_start_gps = None   # robot GPS position when AUTO mission starts

        # ---------- GPS ----------
        self.gps_x = None
        self.gps_y = None
        self.gps_raw_x = None
        self.gps_raw_y = None
        self.gps_fix = 0
        self.gps_control_mode = "LIVE"
        self.gps_start_only_active = False
        self.last_gps_time = 0.0
        self.gps_from_pixhawk = False
        self._last_gps_update_ts = 0.0
        self.gps_max_speed_mps = 2.5     # reject impossible GPS jumps, but allow live mower motion
        self.gps_min_step_m = 0.8        # tolerate normal RTK/GPS step while driving
        self.gps_live_auto_alpha = float(os.getenv("GPS_LIVE_AUTO_ALPHA", "0.85"))
        self.gps_live_auto_max_jump_m = float(os.getenv("GPS_LIVE_AUTO_MAX_JUMP_M", "3.0"))
        self.gps_live_auto_along_alpha = float(os.getenv("GPS_LIVE_AUTO_ALONG_ALPHA", "0.65"))
        self.gps_live_auto_lateral_alpha = float(os.getenv("GPS_LIVE_AUTO_LATERAL_ALPHA", "0.14"))
        self.gps_live_auto_along_max_step_m = float(os.getenv("GPS_LIVE_AUTO_ALONG_MAX_STEP_M", "0.75"))
        self.gps_live_auto_lateral_max_step_m = float(os.getenv("GPS_LIVE_AUTO_LATERAL_MAX_STEP_M", "0.08"))
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
        self.safety_distance_cm = 100.0
        self.safety_distance_m = self.safety_distance_cm / 100.0
        self.lidar_safety_enabled = True
        # 0.0 = no extra cap (use LiDAR driver range_max directly)
        self.lidar_max_use_m = float(os.getenv("LIDAR_MAX_USE_M", "0.0"))
        # Use only front 240 deg (left/right 120 deg) to ignore rear structure/noise.
        self.lidar_front_sector_deg = float(os.getenv("LIDAR_FRONT_SECTOR_DEG", "240.0"))
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
        self.lidar_avoid_distance_m = float(os.getenv("AUTO_LIDAR_AVOID_DISTANCE_M", "2.85"))
        self.lidar_avoid_clear_margin_m = float(os.getenv("AUTO_LIDAR_AVOID_CLEAR_MARGIN_M", "0.85"))
        self.lidar_hard_pivot_m = float(os.getenv("AUTO_LIDAR_HARD_PIVOT_M", "1.45"))
        self.lidar_avoid_min_base_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_BASE_PWM", "90.0"))
        self.lidar_avoid_min_steer_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_STEER_PWM", "95.0"))
        self.lidar_avoid_min_outer_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_OUTER_PWM", "150.0"))
        self.lidar_avoid_min_inner_pwm = float(os.getenv("AUTO_LIDAR_AVOID_MIN_INNER_PWM", "90.0"))
        self.lidar_obstacle_enter_s = float(os.getenv("LIDAR_OBS_ENTER_S", "0.45"))
        self.lidar_obstacle_exit_s = float(os.getenv("LIDAR_OBS_EXIT_S", "1.60"))
        self.lidar_min_close_points = int(os.getenv("LIDAR_MIN_CLOSE_POINTS", "9"))
        self.lidar_min_close_cluster = int(os.getenv("LIDAR_MIN_CLOSE_CLUSTER", "7"))
        self.lidar_min_cluster_points = int(os.getenv("LIDAR_MIN_CLUSTER_POINTS", "5"))
        self.lidar_min_cluster_width_m = float(os.getenv("LIDAR_MIN_CLUSTER_WIDTH_M", "0.12"))
        self.lidar_cluster_max_gap = int(os.getenv("LIDAR_CLUSTER_MAX_GAP", "2"))
        self.lidar_cluster_max_range_jump_m = float(os.getenv("LIDAR_CLUSTER_MAX_RANGE_JUMP_M", "0.45"))
        self.lidar_min_threat = float(os.getenv("LIDAR_MIN_THREAT", "0.35"))
        self.lidar_noise_margin_m = float(os.getenv("LIDAR_NOISE_MARGIN_M", "0.18"))
        self.lidar_swap_lr = os.getenv("LIDAR_SWAP_LR", "0").lower() in ("1", "true", "yes", "on")
        self.lidar_obstacle_active = False
        self.lidar_avoid_turn = 0.0
        self.lidar_avoid_turn_filt = 0.0
        self.lidar_avoid_filter_alpha = 0.34
        self.lidar_avoid_bias_max = math.radians(float(os.getenv("AUTO_LIDAR_AVOID_BIAS_MAX_DEG", "46.0")))
        self.lidar_avoid_commit_sign = 0.0
        self.lidar_avoid_commit_until = 0.0
        self.lidar_avoid_hold_s = 2.80
        self.lidar_avoid_release_ratio = 0.95
        # Local corridor planner for tree/obstacle-dense areas. This keeps
        # avoidance reactive and lightweight, but chooses the clearest forward
        # gap instead of only comparing total left-vs-right threat.
        self.lidar_corridor_enable = os.getenv("AUTO_LIDAR_CORRIDOR_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.lidar_corridor_sectors = max(5, int(os.getenv("AUTO_LIDAR_CORRIDOR_SECTORS", "9")))
        self.robot_width_m = float(os.getenv("ROBOT_WIDTH_M", "0.95"))
        self.robot_length_m = float(os.getenv("ROBOT_LENGTH_M", "1.60"))
        self.lidar_corridor_side_margin_m = float(os.getenv("AUTO_LIDAR_CORRIDOR_SIDE_MARGIN_M", "0.30"))
        self.lidar_corridor_min_clear_m = float(os.getenv(
            "AUTO_LIDAR_CORRIDOR_MIN_CLEAR_M",
            str(max(1.55, self.robot_width_m + 2.0 * self.lidar_corridor_side_margin_m))
        ))
        self.lidar_corridor_prefer_front = float(os.getenv("AUTO_LIDAR_CORRIDOR_FRONT_WEIGHT", "0.55"))
        self.lidar_corridor_blocked = False
        self.lidar_corridor_best_angle = 0.0
        self.lidar_corridor_best_clear_m = float('inf')
        self.lidar_corridor_sector_clear = []
        # Enable real AUTO obstacle bypass by default: when LiDAR confirms an
        # obstacle on the current segment, insert short temporary waypoints to
        # step around it, then return to the original path.
        self.auto_bypass_enable = os.getenv("AUTO_BYPASS_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.auto_bypass_side_offset_m = float(os.getenv("AUTO_BYPASS_SIDE_OFFSET_M", "1.80"))
        self.auto_bypass_forward_m = float(os.getenv("AUTO_BYPASS_FORWARD_M", "1.20"))
        self.auto_bypass_pass_m = float(os.getenv("AUTO_BYPASS_PASS_M", "4.20"))
        self.auto_bypass_return_m = float(os.getenv("AUTO_BYPASS_RETURN_M", "6.20"))
        self.auto_bypass_min_remaining_m = float(os.getenv("AUTO_BYPASS_MIN_REMAINING_M", "2.20"))
        self.auto_bypass_cooldown_s = float(os.getenv("AUTO_BYPASS_COOLDOWN_S", "5.0"))
        self._auto_bypass_inserted = False
        self._auto_bypass_end_idx = -1
        self._auto_bypass_last_ts = 0.0
        self._base_path_points = []
        self._base_path_corner_indices = set()
        self.steer_cmd_filt = 0.0
        self.steer_filter_alpha = float(os.getenv("AUTO_STEER_FILTER_ALPHA", "0.08"))
        self._last_pixhawk_retry_ts = 0.0

        # ---------- FOLLOW TRACKER CMD ----------
        self.follow_tracker_enabled = os.getenv("FOLLOW_TRACKER_ENABLE", "1").lower() in ("1", "true", "yes", "on")
        self.follow_cmd = "STOP"
        self.follow_cmd_ts = 0.0
        self.follow_pwm_override_l = 0
        self.follow_pwm_override_r = 0
        self.follow_cmd_timeout_s = float(os.getenv("FOLLOW_CMD_TIMEOUT_S", "1.8"))
        self.follow_forward_pwm = float(os.getenv("FOLLOW_FORWARD_PWM", "105"))
        self.follow_turn_pwm = float(os.getenv("FOLLOW_TURN_PWM", "135"))
        self.follow_lidar_stop_m = float(os.getenv("FOLLOW_LIDAR_STOP_M", "2.0"))
        self.follow_stall_boost_pwm = 0.0
        self.follow_stall_boost_ramp = float(os.getenv("FOLLOW_STALL_BOOST_RAMP", "180.0"))
        self.follow_stall_boost_decay = float(os.getenv("FOLLOW_STALL_BOOST_DECAY", "120.0"))
        self.follow_stall_boost_max = float(os.getenv("FOLLOW_STALL_BOOST_MAX", "155.0"))
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
        self.create_subscription(Float32, '/safety_distance_cm', self.safety_distance_cb, 10)
        self.create_subscription(Float32, '/lidar_max_use_m', self.lidar_max_use_cb, 10)
        self.create_subscription(Bool, '/lidar_safety_enable', self.lidar_safety_enable_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)


        # ---------- SERIAL (Arduino) ----------
        self.ser = self.connect_arduino(self.ENCODER_PORT_CANDIDATES)
        self.arduino_connected = True
        self._last_arduino_rx_ts = time.time()

        # ---------- MAVLINK ----------
        self.master = self.connect_pixhawk(self.PIXHAWK_PORT_CANDIDATES)
        self.setup_mavlink_streams()

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
            f"AUTO_DR_FINAL_PASS_MARGIN_M={self.AUTO_DR_FINAL_PASS_MARGIN_M:.2f} "
            f"AUTO_DR_FINAL_MAX_CROSSTRACK_M={self.AUTO_DR_FINAL_MAX_CROSSTRACK_M:.2f} "
            f"AUTO_GPS_CORRIDOR_CONFIRM_M={self.AUTO_GPS_CORRIDOR_CONFIRM_M:.2f}"
        )
        self.get_logger().info(
            f"[CFG] JETSON_PWM_ROTATE_MAP_90={self.jetson_pwm_rotate_map_90} "
            f"AUTO_PWM_SWAP_LR={self.auto_pwm_swap_lr} "
            f"AUTO_PWM_INVERT_L={self.auto_pwm_invert_l} AUTO_PWM_INVERT_R={self.auto_pwm_invert_r}"
        )

    # =================================================
    # CALLBACKS และ SERIAL UTILITIES
    # =================================================

    def get_encoder_port_candidates(self, configured_candidates):
        """
        สร้างรายการ port ที่เป็นไปได้สำหรับ Arduino encoder

        ลำดับการค้นหา:
        1. Port ที่กำหนดใน config (ENCODER_PORT_CANDIDATES)
        2. Port by-id ที่มีชื่อ Arduino
        3. Fallback: /dev/ttyACM* (ใช้เมื่อไม่เจอแบบ by-id)
        """
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

        # 2) stable by-id names for common USB-UART chipsets (exclude Pixhawk/Holybro)
        by_id_patterns = ["/dev/serial/by-id/*Arduino*"]
        for pattern in by_id_patterns:
            for p in sorted(glob.glob(pattern)):
                name = os.path.basename(p).lower()
                if "holybro" in name or "pixhawk" in name or "silicon_labs" in name or "cp210" in name:
                    continue
                add_port(p)

        # 3) fallback to ACM only (avoid LiDAR on ttyUSB*)
        for p in sorted(glob.glob("/dev/ttyACM*")):
            add_port(p)

        return ports

    def connect_arduino(self, configured_candidates, baud=115200, timeout=0.0):
        candidates = self.get_encoder_port_candidates(configured_candidates)
        pixhawk_reals = {os.path.realpath(p) for p in self.PIXHAWK_PORT_CANDIDATES}
        last_error = None
        for port in candidates:
            # Never steal Pixhawk serial port
            if os.path.realpath(port) in pixhawk_reals:
                continue
            try:
                ser = serial.Serial(port, baud, timeout=timeout)
                ser.setDTR(False)
                time.sleep(1)
                ser.reset_input_buffer()
                self.get_logger().info(f"✅ Arduino Ready: {port}")
                return ser
            except Exception as e:
                last_error = e
                self.get_logger().warn(f"⚠️ Arduino not available: {port} ({e})")

        raise RuntimeError(
            f"Cannot open Arduino on any candidate port: {candidates} | last_error={last_error}"
        )

    def get_pixhawk_port_candidates(self, configured_candidates):
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

        # 2) Pixhawk/Holybro by-id names
        by_id_patterns = [
            "/dev/serial/by-id/*Holybro*",
            "/dev/serial/by-id/*Pixhawk*",
        ]
        for pattern in by_id_patterns:
            for p in sorted(glob.glob(pattern)):
                add_port(p)

        # 3) fallback to ACM ports commonly used by Pixhawk USB CDC
        for p in sorted(glob.glob("/dev/ttyACM*")):
            add_port(p)

        return ports

    def connect_pixhawk(self, candidates, baud=921600):
        """
        เชื่อมต่อ MAVLink กับ Pixhawk โดยลองทีละ port จาก candidates

        ลองเชื่อมต่อแต่ละ port และรอ heartbeat 2 วินาที
        ถ้าสำเร็จ return mavutil connection object
        ถ้าล้มเหลวทุก port จะ raise RuntimeError
        """
        candidates = self.get_pixhawk_port_candidates(candidates)
        last_error = None
        for port in candidates:
            try:
                master = mavutil.mavlink_connection(
                    port,
                    baud=baud,
                    autoreconnect=True,
                    robust_parsing=True,
                    source_system=255
                )
                hb = master.wait_heartbeat(timeout=2.0)
                if hb is None:
                    raise RuntimeError("no heartbeat")
                self.PIXHAWK_PORT = port
                self.get_logger().info(f"✅ Pixhawk connected: {port}")
                return master
            except Exception as e:
                last_error = e
                self.get_logger().warn(f"⚠️ Pixhawk not available: {port} ({e})")
        raise RuntimeError(
            f"Cannot open Pixhawk on any candidate port: {candidates} | last_error={last_error}"
        )

    def setup_mavlink_streams(self):
        """
        กำหนด stream rate ของ MAVLink message จาก Pixhawk

        บังคับให้ Pixhawk ส่ง ATTITUDE (ทิศทาง) และ GLOBAL_POSITION_INT (GPS)
        ในอัตราที่กำหนด แม้ว่า autopilot default จะช้ากว่านี้
        """
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
    
    def web_mode_cb(self, msg):
        """
        Callback รับคำสั่งจาก Web UI ผ่าน topic /web_mode

        คำสั่งที่รองรับ:
        - START / AUTO_START  — เริ่ม mission อัตโนมัติ
        - FOLLOW_START        — เริ่มโหมด follow (ตามคน)
        - STOP                — หยุด mission
        - PAUSE               — หยุดชั่วคราว (เก็บสถานะ mission ไว้)
        - RESUME              — ต่อ mission จากที่หยุด
        - AUTOSPEED,<pct>     — ตั้งความเร็ว AUTO เป็น % (0-100)
        - GPS_RESYNC          — ซิงค์ GPS state ใหม่
        """
        raw_cmd = msg.data.strip().upper()
        cmd = raw_cmd

        # Normalize web command aliases so AUTO/FOLLOW pages can use dedicated labels.
        if cmd == "AUTO_START":
            self.web_run_mode = "AUTO"
            cmd = "START"
        elif cmd == "FOLLOW_START":
            self.web_run_mode = "FOLLOW"
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
            self.gps_start_only_active = False
            self.send_pwm(0, 0)

            try:
                self._restore_base_path(reset_idx=False)
                self.current_idx = 0
            except:
                pass
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

            self.get_logger().warn("⏹️ STOP: AUTO/FOLLOW RESET")
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
                # Record the robot's GPS position as the virtual segment start so
                # cross-track correction works even for the first leg (start → WP1).
                if self.gps_x is not None and self.gps_y is not None:
                    self._auto_start_gps = (self.gps_x, self.gps_y)
                else:
                    self._auto_start_gps = None
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

    def _reset_dead_reckoning_state(self, sync_to_gps=True):
        """
        รีเซ็ต state ทั้งหมดที่ใช้ใน dead reckoning (DR)

        เรียกเมื่อเริ่ม mission ใหม่หรือ GPS กลับมามีสัญญาณ
        sync_to_gps=True: นำ GPS position ปัจจุบันมาเป็น anchor ของ DR
        """
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
        """รีเซ็ต state ที่ติดตาม progress การเคลื่อนที่ตาม path — เรียกเมื่อเริ่ม waypoint ใหม่"""
        self._path_progress_rate_mps = 0.0
        self._last_progress_dist_to_wp = None
        self._last_progress_idx = -1
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
        """
        ซิงค์ GPS state ใหม่เมื่อ GPS กลับมามีสัญญาณหลังหาย

        รีเซ็ต dead reckoning state และตั้ง gps_x/gps_y ให้ตรงกับ GPS raw ล่าสุด
        เรียกจาก web_mode_cb เมื่อรับคำสั่ง GPS_RESYNC
        """
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
        if min_spacing <= 0.0:
            return list(points)

        filtered = [points[0]]
        skipped = 0
        for idx, point in enumerate(points[1:], start=1):
            last = filtered[-1]
            dn, de = self._gps_delta_m(last[0], last[1], point[0], point[1])
            dist_m = math.hypot(dn, de)
            is_final = idx == len(points) - 1
            if dist_m < min_spacing and not is_final:
                skipped += 1
                continue
            if dist_m < max(0.15, min_spacing * 0.35) and is_final and len(filtered) > 1:
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
        if self.gps_x is None or self.gps_y is None or len(self.path_points) < 3:
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
            best_idx > 0
            and best_cross <= self.AUTO_START_SNAP_MAX_CROSSTRACK_M
            and (first_dist - best_cross) >= self.AUTO_START_SNAP_MIN_GAIN_M
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
        self._reset_motion_progress_state()
        self.pid_yaw.reset()
        self.steer_cmd_filt = 0.0
        self.get_logger().warn(
            f"🎯 AUTO start snapped to path idx {self.current_idx + 1}/{len(self.path_points)} "
            f"xtrk={best_cross:.2f}m first={first_dist:.2f}m"
        )

    def waypoint_cb(self, msg):
        """
        Callback รับรายการ waypoint จาก Web UI (topic /waypoints)

        Format: Float32MultiArray ที่มีคู่ค่า [lat0, lon0, lat1, lon1, ...]
        - ตรวจสอบความถูกต้องของ data length (ต้องเป็นเลขคู่)
        - แปลง lat/lon เป็น x/y เมตรเทียบกับ GPS anchor
        - เรียก path densification ถ้าเปิดใช้งาน
        - เก็บ path_corner_indices เพื่อให้รู้ว่า waypoint ใดเป็นมุม
        """
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

    def safety_distance_cb(self, msg):
        cm = float(msg.data)
        cm = max(0.0, min(400.0, cm))
        self.safety_distance_cm = cm
        self.safety_distance_m = cm / 100.0
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

    def lidar_cb(self, msg):
        """
        Callback รับข้อมูล LaserScan จาก LIDAR (topic /scan)

        ทำงาน:
        1. วิเคราะห์ sector ด้านหน้า/ซ้าย/ขวาเพื่อหาสิ่งกีดขวาง
        2. จัดกลุ่ม (cluster) จุดที่ใกล้กันเพื่อลด false positive
        3. ถ้าระยะ < lidar_stop_m และ mode ไม่ใช่ MANUAL → หยุด
        4. ส่งข้อมูล safety status กลับไปยัง /robot_status
        """
        half_sector = math.radians(self.lidar_front_sector_deg * 0.5)
        raw_min_front = float('inf')
        min_front = float('inf')
        front_threat = 0.0
        left_threat = 0.0
        right_threat = 0.0
        close_points = 0
        close_cluster = 0
        close_beams = []
        angle_step = abs(float(msg.angle_increment)) if msg.angle_increment else 0.0
        avoid_distance_m = max(self.safety_distance_m, self.lidar_avoid_distance_m)
        corridor_n = max(5, int(self.lidar_corridor_sectors))
        corridor_clear = [float('inf')] * corridor_n
        corridor_threat = [0.0] * corridor_n

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
                    else:
                        left_threat += threat
                else:
                    if a >= 0.0:
                        left_threat += threat
                    else:
                        right_threat += threat
            for _, r, a, _ in cluster:
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
        )
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

        # Positive = turn left, Negative = turn right. In cluttered areas,
        # corridor mode points toward the clearest local gap. Fallback is the
        # old total-threat comparison.
        turn_raw = (right_threat - left_threat)
        if self.lidar_corridor_enable and (candidate or self.lidar_obstacle_active):
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

        if self.lidar_obstacle_active and desired_sign != 0.0:
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
            desired_turn = self.lidar_avoid_commit_sign * max(abs(desired_turn), release_mag)

        alpha = self.lidar_avoid_filter_alpha
        self.lidar_avoid_turn_filt = (1.0 - alpha) * self.lidar_avoid_turn_filt + alpha * desired_turn
        if (not self.lidar_obstacle_active) and abs(self.lidar_avoid_turn_filt) < math.radians(1.0):
            self.lidar_avoid_turn_filt = 0.0
        self.lidar_avoid_turn = self.lidar_avoid_turn_filt
        self.lidar_last_update_ts = now

    def emergency_cb(self, msg):
        """
        Callback รับสัญญาณฉุกเฉินจาก Web UI (topic /emergency)

        คำสั่ง EMERGENCY: latch สถานะฉุกเฉิน, หยุดมอเตอร์ทันที,
        ส่งคำสั่ง EMG ไปยัง Arduino ซ้ำๆ เพื่อให้แน่ใจว่าหยุด
        คำสั่ง CLEAR: ปลด latch เพื่อให้กลับมาทำงานได้ตามปกติ
        """
        cmd = msg.data.strip().upper()
        self.get_logger().warn(f"[WEB EMERGENCY CMD] {cmd}")

        if cmd == "EMERGENCY":
            # Always re-send EMG so Arduino is forced into EMER loop
            # even after serial reconnect or MCU restart.
            was_latched = self.emergency_latched
            self.emergency_latched = True
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
                self.get_logger().error("🛑 EMERGENCY LATCHED")
            else:
                self.get_logger().warn("🛑 EMERGENCY RE-ASSERT")

        elif cmd == "RESET":
            if self.emergency_latched:
                self.emergency_latched = False
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
                self.get_logger().info("🔄 EMERGENCY RESET")
            else:
                self.get_logger().warn("ℹ️ RESET IGNORED (not in EMERGENCY)")

    # =================================================
    # MAIN LOOP — loop หลักที่ทำงานตามความถี่ที่กำหนด
    # =================================================
    def update(self):
        """
        Loop หลักของระบบ (เรียกโดย ROS timer ~20-50 Hz)

        ลำดับการทำงานในแต่ละรอบ:
        1. ลองเชื่อมต่อ Pixhawk ใหม่ถ้าหลุด
        2. read_pixhawk() — อ่าน IMU/GPS จาก Pixhawk
        3. read_arduino() — อ่าน encoder และ mode จาก Arduino
        4. ensure_arduino_link() — ตรวจสอบและ reconnect Serial ถ้าหลุด
        5. _decay_motion_estimate() — ลด estimate ความเร็วเมื่อไม่ได้รับ encoder
        6. control_logic() — คำนวณ PWM ตาม mode (AUTO/FOLLOW/MANUAL)
        7. publish_all() — ส่งข้อมูลสถานะออก ROS topics
        8. debug_log() — log สถานะเพื่อ debug
        """
        if self.master is None:
            now = time.time()
            if now - self._last_pixhawk_retry_ts > 2.0:
                self._last_pixhawk_retry_ts = now
                try:
                    self.master = self.connect_pixhawk(self.PIXHAWK_PORT_CANDIDATES)
                    self.setup_mavlink_streams()
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Pixhawk reconnect failed: {e}")
        self.read_pixhawk()
        self.read_arduino()
        self.ensure_arduino_link()
        self._decay_motion_estimate()
        self.control_logic()
        self.publish_all()
        self.debug_log()

    def ensure_arduino_link(self):
        """
        ตรวจสอบและ reconnect Serial กับ Arduino ถ้าหลุดการเชื่อมต่อ

        ถ้า Arduino ไม่ส่งข้อมูลมาเกิน arduino_rx_timeout_s:
        - บังคับ mode เป็น MANUAL + STOP เพื่อความปลอดภัย
        - ลองเปิด Serial port ใหม่จาก candidate list
        """
        now = time.time()
        if self.arduino_connected and (now - self._last_arduino_rx_ts) <= self.arduino_rx_timeout_s:
            return

        if self.arduino_connected:
            self.arduino_connected = False
            self.mode_hardware = HW_MANUAL
            self.mode_web = "STOP"
            if now - self._last_arduino_state_warn_ts > 1.0:
                self._last_arduino_state_warn_ts = now
                self.get_logger().error("❌ Arduino link timeout/disconnected -> force STOP + MANUAL")

        if now - self._last_arduino_retry_ts < 1.0:
            return
        self._last_arduino_retry_ts = now
        try:
            self.ser = self.connect_arduino(self.ENCODER_PORT_CANDIDATES)
            self.arduino_connected = True
            self._last_arduino_rx_ts = time.time()
            self._mode_candidate = self.mode_hardware
            self._mode_candidate_count = 0
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

    def _pulses_to_distance_m(self, pulses):
        """แปลงจำนวน encoder pulse เป็นระยะทาง (เมตร) ตาม WHEEL_PULSES_PER_REV และเส้นรอบวงล้อ"""
        return (pulses / self.WHEEL_PULSES_PER_REV) * self.WHEEL_CIRCUM_M

    def _update_yaw_filtered(self, yaw_enu):
        """
        อัปเดต yaw ที่กรองแล้วด้วย exponential moving average (unit-circle EMA)

        ใช้ alpha ต่างกันตอน stationary (alpha_stop) และตอนเคลื่อนที่ (alpha_move)
        เพื่อให้ yaw smooth แต่ยังตอบสนองพอเมื่อหันเลี้ยว
        """
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
        """
        อัปเดตตำแหน่ง dead reckoning จาก encoder pulse

        ใช้จำนวน pulse ที่ล้อหมุน + yaw ปัจจุบันเพื่อประมาณตำแหน่ง
        มี deadband กรอง noise เล็กน้อยออก (ENC_DRIFT_PULSE_DEADBAND, ENC_MOVING_PULSE_DEADBAND)
        ตำแหน่งที่คำนวณได้ใช้เมื่อ GPS หาย เพื่อให้หุ่นยนต์เดินต่อไปได้
        """
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
        """
        คำนวณ unit vector ของ path segment ที่กำลังเดินอยู่

        ใช้สำหรับคำนวณ crosstrack error (ระยะห่างจากเส้นทางที่กำหนด)
        และ along-track progress เพื่อปรับทิศทางใน AUTO mode
        """
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
        """
        Fuse GPS measurement กับ dead reckoning ขณะ AUTO mode กำลังวิ่ง

        ใช้ alpha แยกสำหรับแนว along-track (เร็ว) และ lateral (ช้า)
        เพื่อให้ GPS แก้ไขตำแหน่งได้ แต่ไม่กระโดดข้ามทาง
        """
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

        along *= max(0.0, min(1.0, self.gps_live_auto_along_alpha))
        lateral *= max(0.0, min(1.0, self.gps_live_auto_lateral_alpha))
        along = max(-self.gps_live_auto_along_max_step_m, min(self.gps_live_auto_along_max_step_m, along))
        lateral = max(-self.gps_live_auto_lateral_max_step_m, min(self.gps_live_auto_lateral_max_step_m, lateral))

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
        """
        อัปเดต GPS position ผ่าน filter (EMA + outlier rejection)

        ขั้นตอน:
        1. รอ GPS warm-up (gps_init_samples) ก่อน ready
        2. กรอง GPS jump ที่เร็วเกินไป (outlier rejection)
        3. Smooth ด้วย EMA ที่ alpha ต่างกันตาม mode และสถานะการเคลื่อนที่
        4. Fuse ด้วย _fuse_live_auto_gps_measurement() ขณะ AUTO วิ่ง
        """
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
            if jump_m <= self.gps_live_auto_max_jump_m:
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
        """
        อ่านและประมวลผล MAVLink messages จาก Pixhawk (non-blocking)

        Message ที่รองรับ:
        - ATTITUDE       → อัปเดต yaw (ทิศทาง) ผ่าน _update_yaw_filtered
        - GLOBAL_POSITION_INT → อัปเดต GPS lat/lon ผ่าน _update_gps_filtered
        - SCALED_IMU2    → อ่าน yaw rate สำหรับตรวจสอบการหมุน

        วนอ่านทุก message ที่ค้างอยู่ใน buffer จนหมด
        """
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
                self._update_gps_filtered(msg.lat * 1e-7, msg.lon * 1e-7, msg.fix_type)

    # =================================================
    # ARDUINO — อ่านและส่งข้อมูลผ่าน Serial กับ Arduino
    # =================================================
    def _parse_arduino_line(self, line):
        """
        แยกวิเคราะห์ข้อมูล CSV ที่ Arduino ส่งมา

        Format ที่รองรับ (ตาม firmware version):
        - เก่า:  mode,enc1,enc2
        - กลาง: mode,enc1,enc2,pump,blade,manual_led,auto_lamp,emg
        - ใหม่:  mode,enc1,enc2,pump,blade,manual_led,auto_lamp,emg,can_soc,can_voltage,can_current,can_temp,can_status

        Return: dict ของค่าที่ parse ได้ หรือ None ถ้า format ผิด
        """
        # Parse Arduino CSV status line.
        # Old format: mode,enc1,enc2
        # Mid format: mode,enc1,enc2,pump,blade,manual_led,auto_lamp,emg
        # New format: mode,enc1,enc2,pump,blade,manual_led,auto_lamp,emg,can_soc,can_voltage,can_current,can_temp,can_status
        s = line.strip()
        if not s or s.startswith("#"):
            return None
        try:
            parts = [p.strip() for p in s.split(",")]
            if len(parts) not in (3, 8, 13):
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

        # Arduino initializes CAN fields as 0,0,0,UNKNOWN. That frame is useful at boot,
        # but after a real BMS frame it should not make the dashboard blink.
        looks_like_default_frame = (
            status == 2
            and soc <= 0.0
            and math.isfinite(voltage) and abs(voltage) < 0.01
            and math.isfinite(current) and abs(current) < 0.01
            and math.isfinite(temperature) and abs(temperature) < 0.01
        )

        have_recent_real_can = (
            self.arduino_can_last_valid_ts > 0.0
            and (now - self.arduino_can_last_valid_ts) <= self.arduino_can_invalid_hold_s
        )

        if looks_like_default_frame and have_recent_real_can:
            return

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

        if not looks_like_default_frame and (
            ("voltage" in parsed and voltage > 0.0)
            or status in (0, 1)
            or ("soc" in parsed and soc > 0.0)
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
        # # TX mode=0 ... can_soc=85 can_v=26.40 can_i=-3.20 can_t=31.0 can_st=0
        if not line.startswith("# TX"):
            return False
        fields = dict(re.findall(r"\b(can_soc|can_v|can_i|can_t|can_st)=([^\s]+)", line))
        if not fields:
            return False
        self._update_can_cache(
            can_soc=fields.get("can_soc", None),
            can_voltage=fields.get("can_v", None),
            can_current=fields.get("can_i", None),
            can_temperature=fields.get("can_t", None),
            can_status=fields.get("can_st", None),
        )
        return True

    def read_arduino(self):
        """
        อ่านและประมวลผล serial data จาก Arduino

        อ่านทุก line ที่ค้างใน buffer:
        - Line ปกติ (CSV) → _parse_arduino_line → อัปเดต encoder, mode, battery
        - Line debug "# CAN {}" → _parse_can_debug_line → อัปเดต battery CAN bus
        - Line debug "# TX" → _parse_can_tx_debug_line → อัปเดต battery status

        ถ้าอ่านสำเร็จ → อัปเดต _last_arduino_rx_ts และ arduino_connected = True
        """
        try:
            if not self.arduino_connected:
                return
            # Non-blocking serial read to keep ROS timer loop responsive.
            if hasattr(self.ser, "in_waiting") and self.ser.in_waiting <= 0:
                return

            lines_processed = 0
            while lines_processed < 30 and (not hasattr(self.ser, "in_waiting") or self.ser.in_waiting > 0):
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    break
                lines_processed += 1
                now = time.time()
                # Any incoming line means serial link is alive.
                self._last_arduino_rx_ts = now

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
                mode_in = int(parsed["mode"])
                enc_l = float(parsed["enc_l"])
                enc_r = float(parsed["enc_r"])
                if mode_in not in (HW_MANUAL, HW_AUTO, HW_FOLLOW):
                    if now - self._last_serial_parse_warn_ts > 1.0:
                        self._last_serial_parse_warn_ts = now
                        self.get_logger().warn(f"⚠️ Ignored serial mode out of range: mode={mode_in} line='{line}'")
                    continue

                if mode_in == self._mode_candidate:
                    self._mode_candidate_count += 1
                else:
                    self._mode_candidate = mode_in
                    self._mode_candidate_count = 1

                old = self.mode_hardware
                need_count = (
                    self.MODE_DEBOUNCE_COUNT_MANUAL
                    if mode_in == HW_MANUAL
                    else self.MODE_DEBOUNCE_COUNT_ACTIVE
                )
                if (
                    mode_in != self.mode_hardware
                    and self._mode_candidate_count >= need_count
                ):
                    self.mode_hardware = mode_in
                    self._mode_candidate_count = 0

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
            now = time.time()
            if now - self._last_serial_err_log_ts > 1.0:
                self._last_serial_err_log_ts = now
                self.get_logger().warn(f"⚠️ Arduino read/parse error: {e}")

    # =================================================
    # CONTROL — ส่วนคำนวณและส่งคำสั่งควบคุม
    # =================================================
    def manual_cb(self, msg):
        """
        Callback รับคำสั่ง manual จาก Web UI joystick (topic /manual_cmd)

        Format: "L:pwmL,R:pwmR" หรือ "EMG" (emergency)
        - ถ้า emergency_latched: ส่ง EMG ไปยัง Arduino เท่านั้น
        - ถ้า mode ไม่ใช่ MANUAL: ไม่ส่ง PWM (ป้องกัน command ซ้อน)
        - ปกติ: forward PWM ไปยัง Arduino ผ่าน Serial
        """
        raw = msg.data.strip()
        cmd = raw.upper()

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
        """
        Logic หลักสำหรับคำนวณคำสั่ง PWM ตาม mode ปัจจุบัน

        ลำดับการทำงาน:
        1. ตรวจสอบ Arduino connected และ emergency
        2. ตรวจสอบ mode จาก Arduino hardware (MANUAL/AUTO/FOLLOW)
        3. ถ้า MANUAL: ส่ง manual PWM จาก joystick
        4. ถ้า AUTO + mode_web == START: เรียก auto_drive_logic()
        5. ถ้า FOLLOW + mode_web == START: เรียก follow_drive_logic()
        6. ถ้า STOP/PAUSE: ส่ง PWM = 0
        7. ส่ง PWM สุดท้ายไปยัง Arduino ผ่าน send_pwm()
        """
        if not self.arduino_connected:
            # Do not continue auto behavior when Arduino link is down.
            return

        if self.emergency_latched:
            # EMERGENCY: Arduino handles manual loop, Jetson does not send auto PWM
            return

        if self.gps_calibrating:
            self.send_pwm(0, 0)
            return

        if self.mode_web == "PAUSE":
            self.send_pwm(0, 0)
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
        return self.lidar_obstacle_active

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
        if self.mode_hardware != HW_AUTO or self.mode_web != "START":
            return False
        if not self.lidar_safety_enabled or not self.lidar_obstacle_active:
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

        # Positive side means left of the current path direction.
        side = self.lidar_avoid_commit_sign
        if side == 0.0:
            side = 1.0 if self.lidar_right_threat >= self.lidar_left_threat else -1.0
        side = 1.0 if side >= 0.0 else -1.0
        nx, ny = -uy * side, ux * side

        obstacle_m = self.lidar_confirmed_min_front_m if math.isfinite(self.lidar_confirmed_min_front_m) else self.lidar_avoid_distance_m
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
        if self.current_idx <= 0 or self.gps_x is None or self.gps_y is None:
            return endpoint_lat, endpoint_lon, dist_to_endpoint, 0.0

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
        if (
            not self.IMU_SEGMENT_HEADING_ENABLE
            or self.current_idx <= 0
            or self.gps_x is None
            or self.gps_y is None
        ):
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS"
        start_lat, start_lon = self.path_points[self.current_idx - 1]
        seg_n, seg_e = self._gps_delta_m(start_lat, start_lon, endpoint_lat, endpoint_lon)
        seg_len = math.hypot(seg_n, seg_e)
        if seg_len < 0.5:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS"

        if dist_to_endpoint <= self.AUTO_WP_DIRECT_APPROACH_M:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "GPS_END"

        if self.AUTO_REJOIN_LINE_ENABLE and cross_track_m >= self.GPS_CROSSTRACK_DEADBAND_M:
            return math.atan2(direct_e, direct_n), direct_n, direct_e, "REJOIN"

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

    def _waypoint_handoff_radius_m(self):
        """Use a tighter reach radius at row ends/corners so AUTO does not start the U-turn early."""
        if self.current_idx <= 0 or self.current_idx >= (len(self.path_points) - 1):
            return self.WAYPOINT_REACH_M
        prev_lat, prev_lon = self.path_points[self.current_idx - 1]
        cur_lat, cur_lon = self.path_points[self.current_idx]
        next_lat, next_lon = self.path_points[self.current_idx + 1]
        in_n, in_e = self._gps_delta_m(prev_lat, prev_lon, cur_lat, cur_lon)
        out_n, out_e = self._gps_delta_m(cur_lat, cur_lon, next_lat, next_lon)
        in_len = math.hypot(in_n, in_e)
        out_len = math.hypot(out_n, out_e)
        if in_len < 0.4 or out_len < 0.4:
            return self.WAYPOINT_REACH_M
        dot = max(-1.0, min(1.0, (in_n * out_n + in_e * out_e) / (in_len * out_len)))
        turn_angle = math.acos(dot)
        if turn_angle >= self.WAYPOINT_TURN_ANGLE_RAD:
            return min(self.WAYPOINT_REACH_M, self.WAYPOINT_TURN_REACH_M)
        return self.WAYPOINT_REACH_M

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
            return True
        return (
            cross <= self.AUTO_GPS_CORRIDOR_CONFIRM_M
            and remaining <= max(reach_radius_m, self.AUTO_GPS_PASS_WINDOW_M)
            and along >= -0.25
        )

    def _gps_confirm_allowed_by_dr(self, reach_radius_m):
        """Gate GPS waypoint hits with encoder/IMU progress.

        With GPS-only positioning, a 2-3m sideways/diagonal jump can briefly land
        inside a waypoint radius even while the mower is still mid-segment. GPS
        may start the confirmation timer, but it must agree with dead-reckoning
        that the mower has actually reached the end portion of the segment.
        """
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
        reach_radius_m = self._waypoint_handoff_radius_m() if is_corner_target else self.PATH_DENSE_REACH_M
        # Count waypoints by distance only. Requiring cross-track to be small
        # made the robot refuse to advance when GPS jitter placed it slightly
        # beside the line, which is what made grid/line paths look stuck or skip.
        inside_loose = dist_to_wp <= reach_radius_m
        dr_passed_wp = self._dr_segment_passed_current_waypoint(reach_radius_m)
        gps_passed_wp = self._gps_segment_allows_handoff(reach_radius_m)
        confirm_radius_m = min(self.WAYPOINT_CONFIRM_M, reach_radius_m) if is_corner_target else reach_radius_m
        inside_confirm = dist_to_wp <= confirm_radius_m
        if inside_loose or dr_passed_wp or gps_passed_wp:
            if self._wp_reach_candidate_idx != self.current_idx:
                self._wp_reach_candidate_idx = self.current_idx
                self._wp_reach_candidate_since = now
            required_hold_s = self.wp_reach_hold_s if is_corner_target else self.gps_reach_hold_s
            hold_ok = (now - self._wp_reach_candidate_since) >= required_hold_s
            # DR/encoder segment pass is trusted immediately. GPS distance must
            # stay inside the reach zone briefly and agree with DR progress so
            # one diagonal GPS jump cannot skip a point.
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
                if reached_is_corner:
                    self.get_logger().info(f"✅ Reached corner {reached_idx + 1}/{len(self.path_points)}")
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

        # Lidar obstacle avoidance (AUTO): add steering bias away from obstacle.
        obstacle_avoid_active = self.mode_hardware == HW_AUTO and self.lidar_safety_active()
        obstacle_pivot_override = False
        obstacle_pivot_sign = 0.0
        if obstacle_avoid_active:
            if self.lidar_corridor_enable and self.lidar_corridor_blocked:
                # Dense trees/objects and no confirmed gap wide enough: do not
                # squeeze through. Waiting is safer than carving a random arc.
                self.send_pwm(0, 0)
                return
            yaw_err = normalize_angle(yaw_err + self.lidar_avoid_turn)
            hard_pivot_m = max(self.safety_distance_m, self.lidar_hard_pivot_m)
            if self.lidar_confirmed_min_front_m < hard_pivot_m:
                obstacle_pivot_override = True
                if self.lidar_avoid_turn > math.radians(2.0):
                    obstacle_pivot_sign = 1.0
                elif self.lidar_avoid_turn < -math.radians(2.0):
                    obstacle_pivot_sign = -1.0

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
                self.send_pwm(0, 0)
                return
            turn_mag = max(self.TURN_IN_PLACE_PWM, self.MIN_TURN_OUTER_PWM)
            steer_for_mix = -turn_mag if obstacle_pivot_sign < 0.0 else turn_mag
            if steer_for_mix >= 0:
                self.send_pwm(int(turn_mag), -int(turn_mag))
            else:
                self.send_pwm(-int(turn_mag), int(turn_mag))
            return

        if obstacle_avoid_active:
            # Avoidance must keep moving around the obstacle instead of getting
            # trapped by normal waypoint heading gates.
            self.wp_align_active = False
            self._heading_commit_active = False
            self._turn_in_place_active = False

        # Waypoint handoff behavior:
        # after reaching a point, align heading to next waypoint before moving.
        if self.wp_align_active:
            if abs(yaw_err) <= self.wp_align_tol_rad:
                self.wp_align_active = False
                self._turn_latch_sign = 0.0
                self._turn_direction_override_until = 0.0
                self._turn_direction_override_sign = 0.0
            else:
                turn_mag = self._smooth_turn_pwm(yaw_err, yaw_pwm, turn_boost, align=True)
                turn_sign = self._turn_mix_sign(yaw_err, yaw_pwm, now)
                if turn_sign >= 0:
                    pwm_l = int(turn_mag)
                    pwm_r = -int(turn_mag)
                else:
                    pwm_l = -int(turn_mag)
                    pwm_r = int(turn_mag)
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
                self.send_pwm(pwm_l, pwm_r)
                return

        if not obstacle_avoid_active:
            if self._heading_commit_active:
                if abs(yaw_err) <= self.HEADING_COMMIT_EXIT_RAD:
                    self._heading_commit_active = False
            else:
                if abs(yaw_err) >= self.HEADING_COMMIT_ENTER_RAD:
                    self._heading_commit_active = True

            if self._turn_in_place_active:
                if abs(yaw_err) <= self.TURN_IN_PLACE_EXIT_RAD:
                    self._turn_in_place_active = False
                    self._turn_latch_sign = 0.0
                    self._turn_direction_override_until = 0.0
                    self._turn_direction_override_sign = 0.0
            else:
                if abs(yaw_err) >= self.TURN_IN_PLACE_ENTER_RAD:
                    self._turn_in_place_active = True

            if self._turn_in_place_active:
                turn_mag = self._smooth_turn_pwm(yaw_err, yaw_pwm, turn_boost, align=False)
                turn_sign = self._turn_mix_sign(yaw_err, yaw_pwm, now)
                if turn_sign >= 0:
                    pwm_l = int(turn_mag)
                    pwm_r = -int(turn_mag)
                else:
                    pwm_l = -int(turn_mag)
                    pwm_r = int(turn_mag)
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
                self.send_pwm(pwm_l, pwm_r)
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
            if self.lidar_confirmed_min_front_m < max(self.safety_distance_m, self.lidar_hard_pivot_m):
                base_pwm = 0.0
        base_pwm *= heading_scale
        # U-turn/corner approach: cruise slowly before the corner so GPS/IMU can
        # settle and the pivot starts from a calm robot instead of a fast overshoot.
        if is_corner_target and dist_to_wp < self.AUTO_CORNER_SLOWDOWN_DIST_M:
            t_corner = max(0.0, min(1.0, dist_to_wp / max(0.1, self.AUTO_CORNER_SLOWDOWN_DIST_M)))
            corner_cap = self.AUTO_CORNER_APPROACH_PWM + (self.DRIVE_MAX_PWM - self.AUTO_CORNER_APPROACH_PWM) * t_corner
            base_pwm = min(base_pwm, corner_cap)
        if obstacle_avoid_active and base_pwm > 0.0:
            base_pwm = max(base_pwm, self.lidar_avoid_min_base_pwm)
        heading_blocked = (
            (not obstacle_avoid_active)
            and (self._heading_commit_active or (abs(yaw_err) >= self.FORWARD_BLOCK_YAW_RAD))
        )
        if heading_blocked:
            base_pwm = 0.0
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
            if abs(self.lidar_avoid_turn) > math.radians(1.0):
                avoid_sign = 1.0 if self.lidar_avoid_turn > 0.0 else -1.0
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
                if turn_sign >= 0:
                    self.send_pwm(self.SLIP_PANIC_PWM, -self.SLIP_PANIC_PWM)
                else:
                    self.send_pwm(-self.SLIP_PANIC_PWM, self.SLIP_PANIC_PWM)
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
                f" stallBoost={self.drive_stall_boost_pwm:.0f} lidarPts={self.lidar_close_points}/{self.lidar_close_cluster}"
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

    def _start_stuck_recovery(self, reason, yaw_err):
        now = time.time()
        if now < self._recovery_cooldown_until:
            return False
        self._recovery_phase = "reverse"
        self._recovery_until = now + self.RECOVERY_REVERSE_S
        self._recovery_turn_sign = 1.0 if yaw_err >= 0.0 else -1.0
        self._drive_stuck_since = 0.0
        self._turn_stuck_since = 0.0
        self._heading_commit_active = False
        self._turn_in_place_active = False
        self.pid_yaw.reset()
        self.pid_speed.reset()
        self._kick_pwm = 0.0
        self.drive_stall_boost_pwm = 0.0
        self._drive_boost_since = 0.0
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
            if self._recovery_phase == "reverse":
                self._recovery_phase = "pivot"
                self._recovery_until = now + self.RECOVERY_PIVOT_S
            else:
                self._recovery_phase = ""
                self._recovery_until = 0.0
                self._recovery_cooldown_until = now + self.RECOVERY_COOLDOWN_S
                self.pid_yaw.reset()
                self.pid_speed.reset()
                self._kick_pwm = 0.0
                return False

        if self._recovery_phase == "reverse":
            self.send_pwm(-self.RECOVERY_REVERSE_PWM, -self.RECOVERY_REVERSE_PWM)
            return True

        if self._recovery_turn_sign >= 0.0:
            self.send_pwm(self.RECOVERY_PIVOT_PWM, -self.RECOVERY_PIVOT_PWM)
        else:
            self.send_pwm(-self.RECOVERY_PIVOT_PWM, self.RECOVERY_PIVOT_PWM)
        return True

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

        wrong_way = (
            abs(yaw_err) > self.TURN_IN_PLACE_EXIT_RAD
            and self._turn_yaw_err_rate < -self.turn_wrong_way_progress_rad_s
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
        """
        ส่งคำสั่ง PWM ไปยัง Arduino พร้อม slew rate limiter

        l, r — PWM ล้อซ้าย/ขวา ในช่วง [-255, 255]
        +,+ = เดินหน้า, -,- = ถอยหลัง, ต่างเครื่องหมาย = หมุนแกน

        การ slew limit ป้องกัน current spike เมื่อเปลี่ยน PWM เร็วเกินไป
        (อัตราเพิ่ม/ลด แยกระหว่าง forward/turn mode)
        แล้วแปลง logical PWM → hardware PWM ตาม jetson_pwm_rotate_map_90
        """
        # Keep AUTO/FOLLOW control semantics in logical robot space:
        # +,+ = forward, -,- = reverse, opposite signs = pivot.
        # Hardware polarity mapping is applied only after smoothing.
        l_target = max(-255, min(255, int(l)))
        r_target = max(-255, min(255, int(r)))

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
        """
        ส่ง string command ไปยัง Arduino ผ่าน Serial (append '\\n')

        รองรับคำสั่งพิเศษ EMG (emergency), RST (reset), MANUAL (PWM command)
        ถ้า Arduino ไม่ได้เชื่อมต่อ: return ทันทีโดยไม่ส่ง
        ถ้า Serial error: ตั้ง arduino_connected = False
        """
        try:
            if not self.arduino_connected:
                return
            if s == "EMG" or s == "RST" or s.startswith("MANUAL,"):
                self.get_logger().info(f"[TX->ARDUINO] {s}")
            self.ser.write((s + "\n").encode())
        except Exception as e:
            self.arduino_connected = False
            now = time.time()
            if now - self._last_serial_err_log_ts > 1.0:
                self._last_serial_err_log_ts = now
                self.get_logger().error(f"❌ Arduino write error: {e}")

    # =================================================
    # PUBLISH
    # =================================================
    def publish_all(self):
        """
        Publish ข้อมูลสถานะหุ่นยนต์ทั้งหมดออก ROS topics

        ส่งออก topics:
        - /robot_status (JSON String) — ตำแหน่ง, mode, battery, LIDAR status
        - /odom_data (Float32MultiArray) — x, y, yaw, speed สำหรับ visualization

        ถ้า GPS ยังไม่ ready: return ทันที (ไม่ publish ข้อมูลผิดพลาด)
        """
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

        msg = Float32MultiArray()
        msg.data = [
            float(pub_lat),
            float(pub_lon),
            float(self.yaw_ui),
            float(hw),
            float(self.gps_fix),
            1.0 if self.gps_ready else 0.0,
            float(len(self.gps_init_samples)),
            float(self._gps_init_spread_m),
            float(self._gps_jump_drop_count),
            float(self.lidar_min_front_m if math.isfinite(self.lidar_min_front_m) else -1.0),
            float(self.safety_distance_cm),
            1.0 if self.lidar_safety_active() else 0.0,
            1.0 if self.gps_calibrating else 0.0,
            float(self.last_pwm_l),
            float(self.last_pwm_r),
            1.0 if self.lidar_safety_enabled else 0.0,
            float(self.yaw_offset_deg),
            float(self.lidar_max_use_m),
            float(self.enc_l_raw),
            float(self.enc_r_raw),
            1.0 if self.gps_control_mode == "START_ONLY" else 0.0,
            float(self.auto_speed_scale * 100.0),
            1.0 if self.arduino_pump_on else 0.0,
            float(self.arduino_blade_state),
            1.0 if self.arduino_manual_led else 0.0,
            1.0 if self.arduino_auto_lamp else 0.0,
            1.0 if self.arduino_emg_flag else 0.0,
            float(self.arduino_can_soc),
            float(self.arduino_can_voltage),
            float(self.arduino_can_current),
            float(self.arduino_can_temperature),
            float(self.arduino_can_status),
            float(self.current_idx),
            float(len(self.path_points)),
            target_lat,
            target_lon,
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
    # DEBUG — log สถานะเพื่อ debug และ monitor
    # =================================================
    def debug_log(self):
        """
        Log สถานะระบบทุก 0.3 วินาที เพื่อใช้ debug และ monitor

        แสดง: mode, GPS position, yaw, speed, PWM ซ้าย/ขวา,
               waypoint ปัจจุบัน, สถานะ LIDAR safety
        """
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
    """จุดเริ่มต้นของโปรแกรม: เริ่ม ROS2, สร้าง LawnmowerNode, รัน spin loop"""
    rclpy.init(args=args)
    node = LawnmowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
