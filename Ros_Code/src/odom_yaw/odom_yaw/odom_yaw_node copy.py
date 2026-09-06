import rclpy
from rclpy.node import Node
import serial
import time
import math
from pymavlink import mavutil
import signal
import sys

# ----- CONFIG -----
ENCODER_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24336303633351411171-if00"
ENCODER_BAUD = 115200

PIXHAWK_PORT = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00"
PIXHAWK_BAUD = 921600

PULSE_PER_REV = 600
WHEEL_DIAMETER = 0.3
GEAR_RATIO = 3

wheel_circum = math.pi * WHEEL_DIAMETER

# ----- ROS2 NODE -----
class OdomYawNode(Node):
    def __init__(self):
        super().__init__('odom_yaw_node')
        self.ser = serial.Serial(ENCODER_PORT, ENCODER_BAUD, timeout=1, dsrdtr=False)
        self.ser.setDTR(False)
        time.sleep(2)
        self.ser.reset_input_buffer()

        self.master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
        self.get_logger().info("Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received. Starting Odom + Yaw...")

        self.x = 0.0
        self.y = 0.0
        self.prev_enc = None
        self.yaw = 0.0
        self.mode = "MAN"

        # Timer loop 50Hz
        self.timer = self.create_timer(0.02, self.loop)

    def loop(self):
        # --- READ YAW ---
        msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.yaw = msg.yaw

        # --- READ ENCODER ---
        line = self.ser.readline().decode(errors='ignore').strip()
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

        # --- DISTANCE ---
        distance = (delta_enc / (PULSE_PER_REV * GEAR_RATIO)) * wheel_circum

        # --- UPDATE POSITION ---
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        # --- PRINT ---
        self.get_logger().info(f"MODE={self.mode}  x={self.x:.2f} m  y={self.y:.2f} m  yaw={math.degrees(self.yaw):.1f}°")

    def destroy_node(self):
        try:
            self.ser.close()
        except:
            pass
        try:
            self.master.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
