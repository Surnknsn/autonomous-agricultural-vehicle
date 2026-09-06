import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import serial
import math
from pymavlink import mavutil

# ----- CONFIG -----
ENCODER_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_24336303633351411171-if00"
ENCODER_BAUD = 115200
PIXHAWK_PORT = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00"
PIXHAWK_BAUD = 921600
PULSE_PER_REV = 600
WHEEL_DIAMETER = 0.3
GEAR_RATIO = 3

# ----- NODE -----
class LawnMowerNode(Node):
    def __init__(self):
        super().__init__('lawnmower_node')
        self.get_logger().info("LawnMowerNode started")

        # ROS publishers
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)
        self.obs_pub = self.create_publisher(String, '/ultrasonic', 10)

        # Serial setup
        self.ser = serial.Serial(ENCODER_PORT, ENCODER_BAUD, timeout=1, dsrdtr=False)
        self.ser.setDTR(False)
        self.ser.reset_input_buffer()

        # MAVLink setup
        self.master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
        self.master.wait_heartbeat()

        # Variables
        self.x = 0.0
        self.y = 0.0
        self.prev_enc = None
        self.yaw = 0.0
        self.wheel_circum = math.pi * WHEEL_DIAMETER

        # Timer for loop
        self.timer = self.create_timer(0.1, self.loop)

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
            mode = header
        except:
            return

        avg_enc = (encL + encR) / 2.0
        if self.prev_enc is None:
            self.prev_enc = avg_enc
            return
        delta_enc = avg_enc - self.prev_enc
        self.prev_enc = avg_enc

        distance = (delta_enc / (PULSE_PER_REV * GEAR_RATIO)) * self.wheel_circum
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        # --- PUBLISH POSE ---
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # --- MOCK PWM CONTROL ---
        left_pwm = 1500 + int(distance*1000)
        right_pwm = 1500 + int(distance*1000)
        self.get_logger().info(f"Mock PWM: {{'left_pwm': {left_pwm}, 'right_pwm': {right_pwm}}}")

        # --- PUBLISH OBSTACLE ---
        obs_dist = 20  # cm mock
        self.obs_pub.publish(String(data=str(obs_dist)))
        if obs_dist < 25:
            self.get_logger().info(f"Obstacle detected at {obs_dist} cm")

def main(args=None):
    rclpy.init(args=args)
    node = LawnMowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.master.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
