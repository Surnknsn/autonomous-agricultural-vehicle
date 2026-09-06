#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import serial
import time
import math

from pymavlink import mavutil

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String


class OdomYawNode(Node):

    def __init__(self):
        super().__init__('odom_yaw_node')

        # ================= CONFIG =================
        self.ENCODER_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24336303633351411171-if00"
        self.ENCODER_BAUD = 115200

        self.PIXHAWK_PORT = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00"
        self.PIXHAWK_BAUD = 921600

        self.PULSE_PER_REV = 600
        self.WHEEL_DIAMETER = 0.3
        self.GEAR_RATIO = 3
        self.WHEEL_BASE = 0.5   # ระยะล้อซ้าย-ขวา (เมตร)

        self.MAX_PWM = 255

        self.wheel_circum = math.pi * self.WHEEL_DIAMETER

        # ================= STATE =================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_enc = None
        self.mode = "MAN"

        self.cmd_v = 0.0
        self.cmd_w = 0.0

        # ================= ROS =================
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # ================= SERIAL =================
        self.ser = serial.Serial(
            self.ENCODER_PORT,
            self.ENCODER_BAUD,
            timeout=0.05
        )
        self.ser.setDTR(False)
        time.sleep(2)
        self.ser.reset_input_buffer()

        # ================= PIXHAWK =================
        self.master = mavutil.mavlink_connection(
            self.PIXHAWK_PORT,
            baud=self.PIXHAWK_BAUD
        )
        self.master.wait_heartbeat()

        self.get_logger().info("ODOM + MODE + MOTOR NODE STARTED")

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz

    # =====================================================
    def cmd_vel_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    # =====================================================
    def update(self):
        # -------- YAW FROM PIXHAWK --------
        msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.yaw = msg.yaw

        # -------- READ ARDUINO --------
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except:
            return

        if not line or "," not in line:
            return

        try:
            mode, encL, encR = line.split(",")
            encL = int(encL)
            encR = int(encR)
            self.mode = mode
        except:
            return

        # -------- PUBLISH MODE --------
        m = String()
        m.data = self.mode
        self.mode_pub.publish(m)

        # -------- ODOM --------
        avg_enc = (encL + encR) / 2.0

        if self.prev_enc is None:
            self.prev_enc = avg_enc
            return

        delta_enc = avg_enc - self.prev_enc
        self.prev_enc = avg_enc

        distance = (delta_enc / (self.PULSE_PER_REV * self.GEAR_RATIO)) * self.wheel_circum

        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        self.publish_odom()
        self.publish_tf()

        # -------- MOTOR CONTROL --------
        if self.mode in ["AUTO", "FOLLOW"]:
            self.send_motor_cmd()
        else:
            self.stop_motor()

    # =====================================================
    def send_motor_cmd(self):
        v = self.cmd_v
        w = self.cmd_w

        v_l = v - (w * self.WHEEL_BASE / 2.0)
        v_r = v + (w * self.WHEEL_BASE / 2.0)

        pwm_l = int(max(min(v_l * 100, self.MAX_PWM), -self.MAX_PWM))
        pwm_r = int(max(min(v_r * 100, self.MAX_PWM), -self.MAX_PWM))

        cmd = f"M,{pwm_l},{pwm_r}\n"
        self.ser.write(cmd.encode())

    def stop_motor(self):
        self.ser.write(b"M,0,0\n")

    # =====================================================
    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        self.odom_pub.publish(odom)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.yaw_to_quaternion(self.yaw)

        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdomYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
