#!/usr/bin/env python3
"""
lidar_proc.py — โหนด ROS2 สำหรับประมวลผลข้อมูล LIDAR

หน้าที่หลัก:
- รับข้อมูล LaserScan จาก topic /scan
- กรองเฉพาะมุมด้านหน้า (-15 ถึง +15 องศา)
- หาระยะที่ใกล้ที่สุดในแนวหน้าแล้วส่งออกไปยัง /lidar_warning
  เพื่อให้โหนดควบคุมหลักใช้ตัดสินใจหยุดหรือหลีกเลี่ยงสิ่งกีดขวาง
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class LidarProcessor(Node):
    """โหนดประมวลผล LIDAR สำหรับตรวจจับสิ่งกีดขวางด้านหน้าหุ่นยนต์"""

    def __init__(self):
        super().__init__('lidar_processor')
        # Subscribe รับข้อมูล LaserScan ดิบจากเซ็นเซอร์ LIDAR
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # Publish ระยะที่ใกล้ที่สุดด้านหน้าไปให้โหนดควบคุมหลัก
        self.pub = self.create_publisher(Float32, '/lidar_warning', 10)

    def scan_cb(self, msg):
        """
        Callback รับข้อมูล LaserScan ทุกครั้งที่ LIDAR ส่งข้อมูลมา

        ขั้นตอน:
        1. วนซ้ำจุดสแกนทุกจุดพร้อมคำนวณมุมของแต่ละจุด
        2. เลือกเฉพาะจุดที่อยู่ในช่วงมุม -15 ถึง +15 องศา (แนวหน้า)
        3. กรองค่าที่อยู่นอกช่วง range_min–range_max ออก (ค่าไม่ถูกต้อง)
        4. Publish ระยะใกล้ที่สุดผ่าน /lidar_warning
        """
        # รวบรวมระยะทั้งหมดในแนวหน้า (-15 ถึง +15 องศา)
        front_ranges = []
        for i, r in enumerate(msg.ranges):
            # คำนวณมุมของจุดที่ i (มุม 0 คือตรงหน้า)
            angle = msg.angle_min + (i * msg.angle_increment)
            if math.radians(-15) < angle < math.radians(15):
                # กรองเฉพาะค่าระยะที่อยู่ในช่วงที่เซ็นเซอร์วัดได้จริง
                if msg.range_min < r < msg.range_max:
                    front_ranges.append(r)

        if front_ranges:
            # หาระยะที่ใกล้ที่สุดตรงหน้า แล้ว publish เพื่อแจ้งเตือนระบบควบคุม
            min_dist = min(front_ranges)
            out = Float32()
            out.data = float(min_dist)
            self.pub.publish(out)


def main():
    """จุดเริ่มต้นของโปรแกรม: สร้าง node และรัน ROS2 spin loop"""
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
