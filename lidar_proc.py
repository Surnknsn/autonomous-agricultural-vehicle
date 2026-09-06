#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        # รับค่าจาก Lidar (ข้อมูลดิบ)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # ส่งค่าระยะที่สรุปแล้วไปที่โค้ดหลัก
        self.pub = self.create_publisher(Float32, '/lidar_warning', 10)

    def scan_cb(self, msg):
        # กรองเฉพาะระยะด้านหน้า (มุม -15 ถึง +15 องศา)
        front_ranges = []
        for i, r in enumerate(msg.ranges):
            # มุม 0 คือหน้าตรง, i คือลำดับจุด
            angle = msg.angle_min + (i * msg.angle_increment)
            if math.radians(-15) < angle < math.radians(15):
                if msg.range_min < r < msg.range_max:
                    front_ranges.append(r)
        
        if front_ranges:
            min_dist = min(front_ranges) # หาระยะที่ใกล้ที่สุดตรงหน้า
            out = Float32()
            out.data = float(min_dist)
            self.pub.publish(out)

import math
def main():
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()