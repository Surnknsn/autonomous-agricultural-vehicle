#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import serial
import time
import math
import sys

from pymavlink import mavutil
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

# =====================================================
class LawnMowerControl(Node):
    def __init__(self):
        super().__init__('lawnmower_node')

        # ---------- CONFIG ----------
        # Arduino Encoder + Motor
        self.ARDUINO_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24336303633351411171-if00"
        self.ARDUINO_BAUD = 115200

        # Pixhawk IMU / GPS
        self.PIXHAWK_PORT = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00"
        self.PIXHAWK_BAUD = 57600

        # Encoder / Wheels
        self.PULSE_PER_REV = 600
        self.WHEEL_DIAMETER = 0.3
        self.GEAR_RATIO = 3
        self.wheel_circum = math.pi * self.WHEEL_DIAMETER

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_enc = None
        self.mode = "MAN"
        self.pwmL = 0
        self.pwmR = 0

        # ---------- ROS ----------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- SERIAL ----------
        self.ser_arduino = serial.Serial(self.ARDUINO_PORT, self.ARDUINO_BAUD, timeout=0.1)
        time.sleep(2)
        self.ser_arduino.reset_input_buffer()

        self.master = mavutil.mavlink_connection(self.PIXHAWK_PORT, baud=self.PIXHAWK_BAUD)
        self.master.wait_heartbeat()

        self.get_logger().info("OK START LAWN MOWER NODE")

        # ---------- TIMER ----------
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

        # ---------- FILTER ----------
        self.alpha = 0.3  # low-pass filter for yaw

    # =====================================================
    def update(self):
        # ---- READ YAW FROM PIXHAWK ----
        try:
            msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=0.05)
            if msg:
                # Low-pass filter
                self.yaw = self.alpha * msg.yaw + (1 - self.alpha) * self.yaw
        except:
            pass

        # ---- MOCK GPS ----
        # สำหรับทดลอง, real GPS ต่อทีหลัง
        self.gps_lat = 13.7563  # Mock lat
        self.gps_lon = 100.5018  # Mock lon

        # ---- READ ENCODER FROM ARDUINO ----
        try:
            line = self.ser_arduino.readline().decode(errors='ignore').strip()
        except:
            return
        if not line or "," not in line:
            return

        try:
            header, encL, encR = line.split(",")
            encL = int(encL)
            encR = int(encR)
            self.mode = header
        except:
            return

        avg_enc = (encL + encR) / 2.0
        if self.prev_enc is None:
            self.prev_enc = avg_enc
            return
        delta_enc = avg_enc - self.prev_enc
        self.prev_enc = avg_enc

        # ---- DISTANCE ----
        distance = (delta_enc / (self.PULSE_PER_REV * self.GEAR_RATIO)) * self.wheel_circum

        # ---- UPDATE POSITION ----
        if self.mode == "AUTO":
            # x,y updated only in AUTO if GPS valid (mock always valid)
            self.x += distance * math.cos(self.yaw)
            self.y += distance * math.sin(self.yaw)
        elif self.mode == "MAN":
            # Manual mode, position just shows encoder
            self.x += distance * math.cos(self.yaw)
            self.y += distance * math.sin(self.yaw)
        # FOLLOW_ME stub
        elif self.mode == "FOLLOW":
            pass

        # ---- MOTOR CONTROL ----
        if self.mode == "AUTO":
            # send PWM command to Arduino
            try:
                cmd = f"{self.pwmL},{self.pwmR}\n"
                self.ser_arduino.write(cmd.encode())
            except:
                pass
        else:
            # manual -> stop motor
            try:
                self.ser_arduino.write(b"0,0\n")
            except:
                pass

        # ---- PUBLISH ROS ----
        self.publish_odom()
        self.publish_tf()

        self.get_logger().info(
            f"MODE={self.mode}  x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}° "
            f"gps=({self.gps_lat:.5f},{self.gps_lon:.5f})"
        )

    # =====================================================
    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        self.odom_pub.publish(odom)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.yaw_to_quaternion(self.yaw)
        self.tf_broadcaster.sendTransform(t)

    # =====================================================
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw/2.0)
        q.w = math.cos(yaw/2.0)
        return q

# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = LawnMowerControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
