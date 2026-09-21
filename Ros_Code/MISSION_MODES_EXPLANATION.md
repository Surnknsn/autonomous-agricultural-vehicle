# Mission Modes: Technology and Operation

เอกสารสรุปการทำงานของรถเกษตรอัตโนมัติใน 3 โหมดหลัก สำหรับนำไปใช้ประกอบสไลด์หรืออธิบายระบบ

---

## 1. AUTO Mowing Mode

### โหมดตัดหญ้าอัตโนมัติตามเส้นทาง

ระบบใช้ waypoint หรือเส้นทางที่ผู้ควบคุมกำหนดไว้ เพื่อให้รถเคลื่อนที่ผ่านพื้นที่ทำงานอย่างเป็นระบบ โดยใช้ข้อมูลตำแหน่งและทิศทางของรถเป็น feedback สำหรับปรับการเคลื่อนที่อย่างต่อเนื่อง

### 1. Mission Planning

**Web UI + Waypoint Path**

ผู้ควบคุมกำหนดเส้นทางหรือจุด waypoint บนหน้าเว็บ เพื่อกำหนดพื้นที่และทิศทางการเคลื่อนที่ของรถตัดหญ้า

### 2. Localization

**GPS + Pixhawk IMU**

ใช้ GPS ระบุตำแหน่งรถ และใช้ IMU จาก Pixhawk ตรวจสอบ heading/yaw เพื่อให้ระบบทราบว่ารถอยู่ที่ใดและกำลังหันไปทิศทางใด

### 3. Path Tracking

**Waypoint Navigation + Error Calculation**

ระบบคำนวณระยะห่างจาก waypoint, heading error และ cross-track error เพื่อประเมินว่ารถเบี่ยงออกจากเส้นทางมากน้อยเพียงใด

### 4. Feedback Control

**Encoder + PID Controller**

ใช้ encoder ตรวจสอบการหมุนและความเร็วของล้อ แล้วใช้ PID Controller ลดความคลาดเคลื่อนระหว่างเส้นทางที่ต้องการกับการเคลื่อนที่จริง

### 5. Motion Control

**Differential Drive + PWM**

แปลงผลลัพธ์จาก controller เป็นความเร็วล้อซ้ายและขวาที่แตกต่างกัน เพื่อควบคุมการเดินหน้า การเลี้ยว การปรับแนว และการหมุนเข้าหา waypoint

### 6. Safety Layer

**RPLIDAR + LaserScan + Emergency Control**

ตรวจสอบสิ่งกีดขวาง ความถูกต้องของ sensor การเชื่อมต่อ และสถานะ emergency ก่อนอนุญาตให้รถขับเคลื่อน หากไม่ปลอดภัยระบบจะลดความเร็วหรือหยุดรถ

### Technology Flow

```text
Waypoint จาก Web UI
        ↓
GPS + Pixhawk IMU ระบุตำแหน่งและทิศทาง
        ↓
คำนวณ Position / Heading / Cross-track Error
        ↓
PID Controller
        ↓
Differential PWM ซ้าย-ขวา
        ↓
RPLIDAR Safety Check
        ↓
Jetson → Serial → Arduino
        ↓
Motor Driver และระบบตัดหญ้า
```

### สรุปสำหรับพูดบนสไลด์

> AUTO Mowing Mode ใช้ Web UI ในการกำหนดเส้นทางตัดหญ้า จากนั้นใช้ GPS และ Pixhawk IMU ระบุตำแหน่งและทิศทางของรถ ใช้ Encoder เป็น feedback และ PID Controller เพื่อลดความคลาดเคลื่อนของเส้นทาง ก่อนส่งคำสั่ง Differential PWM ผ่าน Jetson และ Arduino ไปยังมอเตอร์ โดยมี RPLIDAR และ Emergency Control เป็นชั้นความปลอดภัย

---

## 2. Manual Spraying Mode

### โหมดควบคุมรถและระบบฉีดพ่นด้วยผู้ปฏิบัติงาน

ระบบให้ผู้ปฏิบัติงานควบคุมรถตัดหญ้าหรือรถฉีดพ่นโดยตรงผ่านรีโมต RC เหมาะสำหรับการควบคุมในพื้นที่ที่ต้องการการตัดสินใจจากมนุษย์แบบทันที

### 1. Human Control

**RC Transmitter + iBusBM**

ผู้ปฏิบัติงานควบคุมทิศทางรถและอุปกรณ์เสริมผ่านรีโมต RC โดย Arduino รับค่าคำสั่งจาก receiver ผ่าน protocol iBus

### 2. Drive Control

**Arduino Mega 2560 + Motor Driver**

Arduino แปลงค่า channel จากรีโมตเป็นทิศทางและความเร็วของมอเตอร์ซ้าย/ขวา เพื่อควบคุมการเดินหน้า ถอยหลัง และเลี้ยวแบบ differential drive

### 3. Spraying Control

**RC Channel + Relay + Water Pump**

ใช้ channel ที่กำหนดจากรีโมตเพื่อเปิดหรือปิดปั๊มน้ำ/ระบบฉีดพ่น โดย Arduino ควบคุม relay และใช้ threshold กับ debounce เพื่อป้องกันการสั่งงานจากสัญญาณรบกวน

### 4. Auxiliary Actuation

**Servo Motor + Relay Control**

ใช้ channel เพิ่มเติมควบคุม servo และอุปกรณ์เสริม เช่น ระบบใบมีดหรือกลไกที่ต่อผ่าน relay โดยจำกัดช่วงมุมและอัตราการเปลี่ยนตำแหน่งเพื่อให้การทำงานนุ่มนวล

### 5. Feedback and Monitoring

**Wheel Encoder + CAN BMS + Serial Status**

Arduino อ่าน encoder เพื่อตรวจสอบการเคลื่อนที่ของล้อ และอ่านข้อมูลแบตเตอรี่ผ่าน CAN เช่น SOC, voltage, current และ temperature จากนั้นส่งสถานะกลับไปยัง Jetson

### 6. Safety Control

**Deadband + Signal Validation + Emergency Stop**

ระบบตรวจสอบค่าจากรีโมต หากสัญญาณผิดปกติหรือขาดหายจะหยุดมอเตอร์และหยุดอุปกรณ์ที่เกี่ยวข้อง เพื่อป้องกันการทำงานโดยไม่ได้ตั้งใจ

### Technology Flow

```text
RC Transmitter
        ↓
iBusBM รับคำสั่งจากผู้ควบคุม
        ↓
Arduino ประมวลผล Channel
        ↓
แยกคำสั่ง Drive / Pump / Servo / Relay
        ↓
Motor Driver + Water Pump + Servo
        ↓
Wheel Encoder + CAN BMS ส่งสถานะกลับ
```

### สรุปสำหรับพูดบนสไลด์

> Manual Spraying Mode ใช้ RC Transmitter เป็นแหล่งคำสั่งหลัก โดย iBusBM รับค่าการควบคุมเข้าสู่ Arduino Mega 2560 เพื่อควบคุมมอเตอร์ ปั๊มน้ำ servo และอุปกรณ์เสริมแบบ real-time พร้อมตรวจสอบสัญญาณและ emergency stop ก่อนสั่งงานจริง

---

## 3. Follow Me Mode

### โหมดติดตามผู้ควบคุมด้วย Computer Vision และ LiDAR

ระบบใช้กล้องตรวจจับบุคคล ยืนยันว่าเป็น owner ที่เลือกไว้ และควบคุมรถให้รักษาทิศทางกับระยะห่างที่เหมาะสม โดยมี LiDAR ช่วยตรวจระยะและสิ่งกีดขวาง

### 1. Target Detection

**Camera + OpenCV + YOLO11n**

กล้องส่งภาพเข้าสู่ Jetson และใช้ OpenCV เตรียมภาพก่อนให้ YOLO11n ตรวจจับตำแหน่งของบุคคลในภาพ โดยใช้ class `person`

### 2. Target Tracking

**ByteTrack**

ติดตามบุคคลเดิมต่อเนื่องระหว่างหลายเฟรม เพื่อให้ระบบรักษาความต่อเนื่องของเป้าหมายขณะบุคคลเคลื่อนที่

### 3. Owner Identification

**OSNet ReID**

สร้าง feature หรือ embedding ของบุคคลจากภาพ แล้วเปรียบเทียบกับ owner profile เพื่อยืนยันว่าบุคคลที่ตรวจพบคือผู้ควบคุม ไม่ใช่บุคคลอื่น

### 4. Distance and Obstacle Sensing

**RPLIDAR + LaserScan**

ใช้ LiDAR ตรวจระยะห่างของเป้าหมายและตรวจสอบสิ่งกีดขวางรอบรถ เพื่อช่วยป้องกันการชนระหว่างการติดตาม

### 5. Follow Control

**Bearing Error + Distance Error**

คำนวณว่าผู้ควบคุมอยู่ทางซ้ายหรือขวาของภาพ และอยู่ใกล้หรือไกลเกินไป จากนั้นปรับความเร็วล้อซ้าย/ขวาเพื่อรักษาทิศทางและระยะติดตาม

### 6. Real-time Actuation

**Jetson + ROS 2 + Arduino**

Jetson ประมวลผลภาพและตัดสินใจการติดตาม จากนั้นส่งคำสั่ง Differential PWM ผ่าน Serial ไปยัง Arduino เพื่อควบคุมมอเตอร์จริง

### 7. Safety and Fallback

**Confidence Check + Target Lost + Emergency Stop**

หากตรวจไม่พบ owner, confidence ต่ำ, target หาย หรือ LiDAR พบสิ่งกีดขวาง ระบบจะลดความเร็วหรือหยุดรถ โดยมี MediaPipe และ color matching เป็น fallback ในบางกรณี

### Technology Flow

```text
Camera Frame
        ↓
OpenCV Preprocessing
        ↓
YOLO11n ตรวจจับบุคคล
        ↓
ByteTrack รักษา Track ID
        ↓
OSNet ReID ยืนยัน Owner
        ↓
LiDAR ตรวจระยะและสิ่งกีดขวาง
        ↓
คำนวณ Bearing / Distance Error
        ↓
Follow Controller
        ↓
Jetson → Serial → Arduino
        ↓
Differential Drive
```

### สรุปสำหรับพูดบนสไลด์

> Follow Me Mode ใช้กล้องและ YOLO11n ตรวจจับบุคคล ใช้ ByteTrack รักษาการติดตามข้ามเฟรม และใช้ OSNet ReID ยืนยันตัวตนของ owner จากนั้นรวมข้อมูลกับ LiDAR เพื่อคำนวณทิศทางและระยะห่าง ก่อนส่งคำสั่ง Differential PWM ผ่าน Jetson และ Arduino ให้รถติดตามอย่างปลอดภัย

---

## Technology ที่ใช้ร่วมกันทั้งระบบ

| Technology | หน้าที่ |
|---|---|
| **Jetson** | ประมวลผล high-level control, computer vision และ decision making |
| **ROS 2 Foxy** | เชื่อม node, sensor, controller และข้อมูลสถานะ |
| **Arduino Mega 2560** | Real-time I/O, emergency handling และควบคุม hardware |
| **Serial 115200** | ส่งคำสั่ง PWM และรับ status ระหว่าง Jetson กับ Arduino |
| **Differential Drive** | ควบคุมการเคลื่อนที่ด้วยความเร็วล้อซ้ายและขวา |
| **RPLIDAR** | ตรวจระยะและสิ่งกีดขวางเพื่อความปลอดภัย |
| **Safety Supervisor** | ตรวจ mode, timeout, sensor validity และ emergency ก่อนขับรถ |

## สรุปภาพรวม 3 โหมด

- **AUTO Mowing:** ใช้ตำแหน่งและทิศทางของรถเป็น reference เพื่อวิ่งตามเส้นทางตัดหญ้า
- **Manual Spraying:** ใช้คำสั่งจากผู้ปฏิบัติงานเป็น reference เพื่อควบคุมรถและระบบฉีดพ่นแบบ real-time
- **Follow Me:** ใช้ตำแหน่งและ identity ของ owner เป็น reference เพื่อรักษาทิศทางและระยะห่าง

ทั้ง 3 โหมดใช้ hardware actuation และ safety layer ร่วมกัน แต่ใช้แหล่ง reference และ algorithm ในการตัดสินใจแตกต่างกัน
