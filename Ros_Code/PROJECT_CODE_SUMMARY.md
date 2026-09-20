# สรุปโค้ดระบบ Autonomous Agricultural Vehicle

เอกสารนี้สรุปโครงสร้างและหน้าที่ของโค้ดใน `Ros_Code` จากไฟล์ที่มีอยู่ใน workspace ณ วันที่จัดทำเอกสาร โดยแยกให้เห็นว่าแต่ละ library, model และ package ใช้ทำอะไร รวมถึงส่วนที่เป็นโค้ดหลัก ส่วนทดลอง และ fallback

## 1. ภาพรวมระบบ

ระบบเป็นรถเกษตร/รถตัดหญ้าแบบ differential drive ที่มีโหมดหลัก 3 โหมด:

- **MANUAL**: รับคำสั่งจากรีโมต RC ผ่าน iBus และควบคุมมอเตอร์/servo/ปั๊มน้ำ/ใบมีด
- **AUTO**: รับคำสั่งการเคลื่อนที่จาก Jetson/ROS 2 แล้วส่ง PWM ไป Arduino
- **FOLLOW**: ใช้กล้องและระบบตรวจจับ/ติดตามคน ร่วมกับ LiDAR และ ReID เพื่อให้รถติดตามเจ้าของที่เลือกไว้

โครงสร้างการควบคุมโดยย่อ:

```text
กล้อง / LiDAR / Web UI / ROS 2
            |
            v
Jetson: lawnmower_control + YOLO + OSNet + logic ความปลอดภัย
            |
            | Serial 115200: PWM, EMG, RST, สถานะ
            v
Arduino Mega 2560
            |
            v
Motor driver / มอเตอร์ / encoder / servo / relay / CAN battery
```

## 2. รุ่นและโมเดลที่ใช้

| รายการ | รุ่น/ไฟล์ | หน้าที่ |
|---|---|---|
| ROS | **ROS 2 Foxy** | middleware สำหรับ node, topic, message และ launch |
| Object detection | **YOLO11n** (`yolo11n.pt`) | ตรวจจับตำแหน่งคนในภาพและสร้าง bounding box |
| Object tracking | **ByteTrack** (`bytetrack.yaml`) | รักษา track ID ของคนข้ามหลายเฟรม |
| Person ReID | **OSNet x0.25 MSMT17** (`osnet_x0_25_msmt17_b1.onnx`) | สร้าง embedding เพื่อแยกคนที่เลือกจากคนอื่น |
| ReID fallback | **MobileNetV3-Small** จาก `torchvision` | ใช้แทน OSNet เมื่อ OSNet/ONNX ใช้งานไม่ได้ |
| Pose fallback | **MediaPipe Pose** | fallback สำหรับตรวจ/ติดตามบุคคลเมื่อ YOLO ใช้ไม่ได้หรือโหมด auto ต้อง fallback |
| Appearance fallback | HSV/color histogram | เปรียบเทียบสีเสื้อผ้าเมื่อ ReID model ใช้ไม่ได้ |
| LiDAR | **rplidar_ros 2.1.4** | รับข้อมูล LaserScan สำหรับระยะและความปลอดภัยรอบรถ |
| Flight/autopilot bridge | MAVROS/PX4-related code | มี package และ launch สำหรับเชื่อม PX4/MAVROS แต่ไม่ใช่เส้นทางควบคุมหลักที่ยืนยันจากเอกสาร startup ปัจจุบัน |

### 2.1 YOLO ใช้ทำอะไร

โค้ด import `YOLO` จากไลบรารี `ultralytics` และตั้งค่าเริ่มต้นเป็น:

```text
model: yolo11n.pt
confidence: 0.34
image size: 320
classes: [0]
tracking: enabled
tracker: bytetrack.yaml
```

`class 0` ของโมเดล COCO คือ **person** ดังนั้นระบบไม่ได้ใช้ YOLO เพื่อจำแนกพืชหรือวัชพืช แต่ใช้ตรวจว่ามีคนอยู่ตรงไหนเพื่อการ FOLLOW

ลำดับการทำงาน:

1. รับภาพจากกล้องหรือ HTTP camera endpoint
2. YOLO ตรวจจับคนในภาพ
3. เลือก bounding box ที่เหมาะสม
4. ใช้ขนาด/ตำแหน่งกล่องประมาณทิศทางและระยะของเป้าหมาย
5. ส่งข้อมูลต่อให้ ReID, LiDAR fusion และตัวควบคุมการเคลื่อนที่
6. คำนวณ PWM ซ้าย/ขวา แล้วส่งไป Arduino

สามารถเปลี่ยน model path ด้วย environment variable:

```bash
export FOLLOW_YOLO_MODEL=/path/to/model.pt
```

ถ้า path ที่กำหนดไม่มีอยู่ โค้ดจะ fallback ไปหา `yolo11n.pt` ใน environment ปัจจุบัน

### 2.2 OSNet ใช้ทำอะไร

OSNet ไม่ได้ทำหน้าที่ตรวจว่าคนอยู่ที่ไหน เพราะหน้าที่นั้นเป็นของ YOLO แต่ทำหน้าที่ **Person Re-Identification (ReID)**:

- แปลงภาพ crop ของบุคคลเป็น feature vector/embedding
- เปรียบเทียบคนที่เห็นในเฟรมปัจจุบันกับ reference ของเจ้าของที่เลือกไว้
- ช่วยแยกคนที่ต้องติดตามออกจากคนอื่น
- ช่วย reacquire เมื่อคนหลุดเฟรมแล้วกลับเข้ามาใหม่
- ใช้ร่วมกับสีเสื้อ/กางเกงและประวัติการติดตามเพื่อแก้ความกำกวม

โมเดลที่ใช้คือ `osnet_x0_25_msmt17_b1.onnx` ซึ่งเป็น OSNet ขนาดเล็กที่ train ด้วย dataset MSMT17 และถูกเรียกผ่าน `onnxruntime`

จุดสำคัญจากโค้ดปัจจุบัน:

```python
providers=["CPUExecutionProvider"]
```

ดังนั้น OSNet ในโค้ดชุดนี้ถูกสั่งให้รันบน CPU ของ Jetson ไม่ใช่ GPU โดยตรง แม้ Jetson จะเป็นเครื่องที่ประมวลผลหลักก็ตาม `onnxruntime` เป็นตัว runtime สำหรับรันไฟล์ ONNX ไม่ใช่โมเดลอีกตัวหนึ่ง

### 2.3 MobileNetV3-Small fallback

ถ้า OSNet ใช้ไม่ได้ โค้ดจะพยายามใช้ `MobileNet_V3_Small_Weights.DEFAULT` จาก `torchvision` เพื่อสร้าง embedding แทน โดยเลือก device ตามค่า:

```text
FOLLOW_REID_CUDA=1  -> ใช้ CUDA ถ้า torch.cuda.is_available()
FOLLOW_REID_CUDA=0  -> ใช้ CPU เป็นค่าเริ่มต้น
```

ถ้า MobileNet ก็ใช้ไม่ได้ ระบบจะลดรูปแบบไปใช้ color/HSV matching

### 2.4 MediaPipe Pose fallback

โค้ด import `mediapipe.python.solutions.pose` และเตรียม Pose model ไว้เป็น fallback โดยเฉพาะกรณี:

- ตั้ง `FOLLOW_DETECTOR=mediapipe`
- ตั้ง `FOLLOW_DETECTOR=auto` แล้ว YOLO ตรวจไม่พบคนหรือใช้งานไม่ได้
- ไลบรารี/ไฟล์ YOLO ไม่พร้อม

MediaPipe ในระบบนี้เป็นทางสำรอง ไม่ใช่โมเดลหลักเมื่อ `FOLLOW_DETECTOR` เป็นค่าเริ่มต้น `yolo`

## 3. ROS 2 packages

### 3.1 `lawnmower_control`

แพ็กเกจหลักของระบบควบคุม ประกอบด้วย node สำคัญ:

- `lawnmower_node.py`
  - สื่อสารกับ Arduino ผ่าน serial
  - รับ/ส่งสถานะรถ
  - ควบคุมโหมด AUTO/FOLLOW
  - เชื่อมข้อมูล ROS, encoder, battery และคำสั่งเคลื่อนที่
  - มีโค้ด `pymavlink` สำหรับงานเชื่อมต่อ MAVLink ที่เกี่ยวข้อง
- `follow_tracker_node.py`
  - รับภาพกล้อง
  - YOLO11n detection
  - ByteTrack tracking
  - OSNet/MobileNet/สีเสื้อ ReID
  - อ่าน LaserScan
  - ประเมินเป้าหมายและคำนวณคำสั่งติดตาม
- `camera_lane_assist_node.py`
  - ช่วยประเมิน lane/ภาพจากกล้องในงาน auto หรือ lane assist

ROS dependencies ที่ประกาศใน `package.xml`:

- `rclpy`: เขียน ROS 2 node ด้วย Python
- `std_msgs`: message พื้นฐาน เช่น String และ Bool
- `geometry_msgs`: ตำแหน่ง/ความเร็ว/เรขาคณิตของระบบ
- `sensor_msgs`: ข้อมูล sensor เช่น LaserScan
- `nav_msgs`: Odometry และข้อมูล navigation
- `tf2_ros`: การจัดการ transform ระหว่าง frame

Python libraries ที่ใช้จริงใน node หลัก แต่ไม่ได้ pin version ใน package manifest:

- `opencv-python`/`cv2`: อ่านภาพ resize, color conversion, crop และ image processing
- `numpy`: array, vector, norm และ cosine similarity
- `requests`: ดึงภาพจาก HTTP camera และเรียก HTTP API
- `pyserial`/`serial`: สื่อสารกับ Arduino
- `pymavlink`: สื่อสารกับอุปกรณ์หรือระบบที่ใช้ MAVLink
- `onnxruntime`: รัน OSNet ONNX
- `torch`: tensor และ inference ของ MobileNet fallback
- `torchvision`: MobileNetV3-Small และ preprocessing
- `Pillow`: แปลงภาพสำหรับ torchvision
- `mediapipe`: pose/person fallback
- `ultralytics`: YOLO11n detection และ tracking

### 3.2 `rplidar_ros`

แพ็กเกจ C++ ของ Slamtec สำหรับ ROS 2 มี version ใน manifest เป็น **2.1.4**

หน้าที่:

- เปิด serial หรือ network channel ไปยัง RPLIDAR
- แปลงข้อมูล scan เป็น `sensor_msgs/LaserScan`
- รองรับรุ่นที่ระบุใน package เช่น A1, A2, A3, S1, S2, S3 และ T1
- มี launch file แยกตามรุ่นและ launch แบบ view ใน RViz

ค่าเริ่มต้นที่เห็นใน node RPLIDAR ได้แก่:

```text
serial_port: /dev/ttyUSB0
serial_baudrate: 1000000
channel_type: serial
```

### 3.3 `amr_description`

แพ็กเกจ description ของตัวรถ:

- URDF/Xacro
- mesh ของตัวรถ
- `robot_state_publisher`
- `joint_state_publisher`
- RViz2
- launch สำหรับแสดงโมเดลรถ

Dependencies:

- `joint_state_publisher`
- `robot_state_publisher`
- `rviz2`
- `xacro`

แพ็กเกจนี้ใช้สำหรับ visualization และ robot model ไม่ใช่ตัวควบคุมมอเตอร์โดยตรง

### 3.4 `odom_yaw`

แพ็กเกจ Python สำหรับงาน odometry/yaw และการประมวลผลข้อมูลทิศทางของรถ ใช้ `rclpy`, `std_msgs` และ `geometry_msgs`

ในโฟลเดอร์มีไฟล์สำรองชื่อ `odom_yaw_node copy.py` และ `odom_yaw_node copy 2.py` ซึ่งควรถือเป็นไฟล์ทดลอง/สำรองจนกว่าจะตรวจ launch ที่เรียกใช้งานจริง

### 3.5 `my_mavros_launch`

แพ็กเกจ launch สำหรับเปิดชุด MAVROS/PX4 ที่เกี่ยวข้อง มี launch file `px4.launch.py`

ใน manifest ยังไม่มี runtime dependency ของ MAVROS ระบุไว้อย่างชัดเจน จึงควรติดตั้งและตรวจจาก environment ของเครื่องจริงก่อนใช้งาน

### 3.6 `px4_mavros_bridge`

แพ็กเกจ/โค้ด bridge สำหรับ PX4/MAVROS มี:

- `px4_bridge.launch.py`
- `px4_bridge_raw.launch.py`
- `lawnmower_node.py` รุ่นเก่าหรือเส้นทาง bridge แยก

จากเอกสารเดิมใน workspace ส่วนนี้มีอยู่ใน source แต่ไม่ได้ยืนยันว่า script startup หลักเรียกใช้ในทุกครั้ง จึงควรแยกจากเส้นทาง `lawnmower_control` หลัก

## 4. Arduino Mega 2560

ไฟล์หลักคือ `arduino/mega2560_control/mega2560_control.ino`

### 4.1 Library ของ Arduino

| Library | หน้าที่ |
|---|---|
| `Arduino.h` | API พื้นฐานของ Arduino เช่น `pinMode`, `digitalWrite`, `analogWrite`, `millis`, `map`, `constrain` |
| `IBusBM.h` | อ่าน channel จาก receiver รีโมตผ่าน iBus |
| `Servo.h` | สร้างสัญญาณควบคุม servo 2 ตัว |
| `SPI.h` | bus SPI สำหรับอุปกรณ์ CAN |
| `mcp_can.h` | ควบคุม MCP CAN และอ่านข้อมูล battery/BMS |

ใน repository ไม่ได้ระบุ version ของ Arduino core หรือ library เหล่านี้ จึงไม่ควรสรุปรุ่นย่อยจาก source เพียงอย่างเดียว

### 4.2 Pin และอุปกรณ์

- มอเตอร์ซ้าย: PWM `11`, direction `36/35`
- มอเตอร์ขวา: PWM `10`, direction `41/40`
- รีเลย์ใบมีดขึ้น/ลง: `5/6`
- ปั๊มน้ำ: `4`
- Servo: `12/13`
- Encoder ซ้าย: `18/17`
- Encoder ขวา: `20/21`
- CAN CS: `53`
- CAN interrupt: `2`
- mode switch: `D1=48`, `D2=49`
- ไฟ auto: `22`
- ไฟ manual: `26`

### 4.3 โหมดการทำงาน

Mapping ของ mode switch:

```text
D1=0, D2=0 -> MANUAL
D1=0, D2=1 -> AUTO
D1=1, D2=0 -> FOLLOW
D1=1, D2=1 -> fallback เป็น MANUAL
```

มี debounce ประมาณ `80 ms` เพื่อป้องกัน mode เปลี่ยนเพราะสัญญาณสั่น

### 4.4 MANUAL

- อ่าน CH1/CH2 จาก iBus เพื่อควบคุมมอเตอร์ซ้าย/ขวา
- อ่าน CH3/CH4 เพื่อควบคุม servo
- อ่าน CH5 เพื่อสั่งใบมีด UP/DOWN/STOP ผ่าน relay
- อ่าน CH6 เพื่อเปิด/ปิดปั๊มน้ำ
- มี deadband, filtering และจำกัดอัตราการเปลี่ยน servo
- ถ้า channel ผิดปกติหรือค่าต่ำเกินไป จะหยุดมอเตอร์และใบมีด

### 4.5 AUTO/FOLLOW และ Serial จาก Jetson

Arduino รับ command จาก Jetson ผ่าน `Serial` ที่ baudrate `115200`

รูปแบบสำคัญ:

```text
PWM,<left>,<right>     -> ควบคุม PWM ซ้าย/ขวา
MANUAL,<left>,<right>  -> emergency web joystick แบบ PWM
EMG                    -> latch emergency และหยุดรถ
RST                    -> reset emergency และหยุดรถก่อนเริ่มใหม่
```

ใน AUTO/FOLLOW คำสั่ง PWM จาก Jetson จะถูกแปลงเป็นทิศทางและความเร็ว แล้วส่งเข้า `driveHardware()`

### 4.6 Emergency และความปลอดภัย

ลำดับ priority หลัก:

1. web emergency หรือ emergency command
2. ถ้ามีคำสั่ง web joystick สด ให้ใช้คำสั่งนั้น
3. ถ้าไม่มีคำสั่งสด ค่าเริ่มต้นคือหยุดรถ
4. RC fallback ใน emergency ถูกปิดไว้ด้วย `WEB_EMER_ALLOW_RC_FALLBACK 0`

การป้องกันที่มีในโค้ด:

- emergency latch ด้วย `EMG`
- reset ด้วย `RST`
- active brake เมื่อหยุด โดยตั้ง INA/INB เป็น HIGH/HIGH
- จำกัด PWM ช่วง `-255..255`
- ยืนยันคำสั่งขับต่อเนื่องหลายรอบก่อนเริ่มเคลื่อนที่
- debounce mode และสวิตช์ใบมีด/ปั๊มน้ำ
- รายงาน encoder, mode, PWM และ CAN status กลับไปทาง serial

### 4.7 CAN battery debug

โค้ดอ่าน CAN frame ของ Daly/Pylontech-like BMS ที่ ID:

- `0x355`: SOC และ SOH
- `0x356`: voltage, current, temperature
- `0x351`: charge/discharge limit
- `0x359`: status/protection

ค่าที่อ่านได้ถูกส่งกลับไปทาง serial เพื่อให้ Jetson หรือระบบ web นำไปแสดงผล

## 5. Web application และ rosbridge

### 5.1 Web server

ไฟล์หลัก:

- `web_app/app_web.py`
- `web/templates/index.html`
- `web/static/`

Library สำคัญ:

- `Flask`: HTTP web server และ route/API
- `flask_sock`: WebSocket endpoint ฝั่ง Flask
- `websocket`: ติดต่อ WebSocket/rosbridge
- `requests`: เรียก camera/API ภายนอก
- `Werkzeug secure_filename`: ป้องกันชื่อไฟล์อัปโหลดที่ไม่ปลอดภัย
- `OpenCV`: อ่าน/ประมวลผลภาพกล้องเมื่อมีการใช้งาน
- `MediaPipe`: pose helper ใน web เมื่อ dependency พร้อม
- HTML/CSS/JavaScript: dashboard, control, camera, map และ state display
- `roslibjs`: ฝั่ง browser สำหรับคุยกับ ROS ผ่าน rosbridge ตามหน้าเว็บ/เอกสารระบบ

### 5.2 rosbridge

`rosbridge_websocket` ทำหน้าที่เป็น gateway ระหว่าง browser กับ ROS 2:

```text
Browser JavaScript <-> WebSocket <-> rosbridge <-> ROS topics/services
```

เว็บจึงสามารถ:

- แสดงสถานะรถ
- ส่ง emergency/reset และคำสั่ง manual
- ดู camera snapshot/video
- เลือก owner profile
- อ่านข้อมูล mission/map/datalog

### 5.3 ไฟล์ข้อมูลและ dataset

ภายใน `web/` มีข้อมูลที่เกี่ยวข้องกับ:

- mission log และ datalog
- saved paths
- shared UI state
- owner/reference images
- static assets และ PWA files

ข้อมูลเหล่านี้เป็น data/runtime ของระบบ ไม่ใช่โมเดล YOLO โดยตรง

## 6. ลำดับการทำงานของ FOLLOW

```text
1. กล้องส่งภาพไปยัง follow_tracker_node
2. YOLO11n ตรวจจับ class person
3. ByteTrack ช่วยรักษา track ต่อเนื่อง
4. crop คนจาก bounding box
5. สร้าง feature ด้วย OSNet
6. เปรียบเทียบกับ reference profile ที่เลือก
7. ผสมคะแนน OSNet กับสีเสื้อ/กางเกงและประวัติ track
8. ใช้ LiDAR ตรวจระยะและ obstacle safety
9. คำนวณ error ของตำแหน่งคนในภาพและระยะเป้าหมาย
10. ส่ง PWM ซ้าย/ขวาไป Jetson-Arduino serial path
11. Arduino ตรวจ mode/emergency แล้วขับมอเตอร์
```

ถ้าองค์ประกอบใดใช้งานไม่ได้ จะลดระดับตามลำดับโดยประมาณ:

```text
OSNet -> MobileNetV3-Small -> color/HSV matching
YOLO -> MediaPipe ในโหมด auto/fallback
กล้อง + LiDAR -> ใช้ข้อมูลที่ยังน่าเชื่อถือและหยุดเมื่อไม่ปลอดภัย
```

## 7. ค่า environment ที่สำคัญ

| Variable | ค่าเริ่มต้น | หน้าที่ |
|---|---:|---|
| `FOLLOW_DETECTOR` | `yolo` | เลือก `auto`, `yolo` หรือ `mediapipe` |
| `FOLLOW_YOLO_MODEL` | `/home/iai/foolme/yolo11n.pt` | path ของ YOLO model |
| `FOLLOW_YOLO_CONF` | `0.34` | confidence threshold |
| `FOLLOW_YOLO_IMGSZ` | `320` | ขนาดภาพ inference |
| `FOLLOW_YOLO_TRACK` | `1` | เปิด ByteTrack |
| `FOLLOW_YOLO_TRACKER` | `bytetrack.yaml` | tracker config |
| `FOLLOW_OSNET_REID_ENABLE` | `1` | เปิด OSNet ReID |
| `FOLLOW_OSNET_REID_ONNX` | `~/ros2_foxy_ws/models/reid/osnet_x0_25_msmt17_b1.onnx` | path OSNet |
| `FOLLOW_OSNET_REID_WEIGHT` | `0.78` | น้ำหนัก OSNet ในคะแนน match |
| `FOLLOW_OSNET_THREADS` | `2` | จำนวน CPU intra-op threads ของ ONNX |
| `FOLLOW_REID_CUDA` | `0` | ให้ MobileNet fallback ใช้ CUDA หรือไม่ |
| `FOLLOW_CAMERA_MODE` | `snapshot` | `auto`, `snapshot`, `mjpeg` หรือ `usb` |
| `FOLLOW_CAMERA_WIDTH` | `640` | ความกว้างกล้อง |
| `FOLLOW_CAMERA_HEIGHT` | `360` | ความสูงกล้อง |
| `FOLLOW_INFER_WIDTH` | `320` | ความกว้างภาพสำหรับ inference |
| `FOLLOW_PROFILE_DIR` | `~/ros2_foxy_ws/web/dataset` | directory reference profile |
| `FOLLOW_REID_ENABLE` | `1` | เปิดระบบ ReID โดยรวม |

## 8. สิ่งที่มีใน workspace แต่ต้องแยกจากโค้ดที่ใช้งานจริง

- `build/`, `install/`, `log/`: ผลลัพธ์จาก colcon และ runtime ไม่ใช่ source หลัก
- `odom_yaw_node copy*.py`: สำเนาหรือเวอร์ชันทดลอง
- `px4_mavros_bridge` และ `my_mavros_launch`: มี source/launch แต่ต้องตรวจ startup script ว่าถูกเรียกใช้จริงใน deployment ปัจจุบันหรือไม่
- เอกสาร flowchart หลายไฟล์: เป็นเอกสารประกอบ บางไฟล์อาจซ้ำกันหรือเป็น revision คนละช่วง
- `models/reid/*.onnx`: เป็นโมเดล ReID ไม่ใช่ YOLO weight
- `yolo11n.pt`: โค้ดอ้างถึงจาก path ภายนอก และไม่พบไฟล์น้ำหนักนี้ใน source ที่สรุปนี้ จึงต้องเตรียมไฟล์บนเครื่อง Jetson เอง

## 9. จุดที่ควรตรวจเมื่อย้ายไป Jetson เครื่องใหม่

1. ติดตั้ง ROS 2 Foxy และ source environment ให้ถูกต้อง
2. ติดตั้ง package ของ ROS ที่ manifest และ launch ต้องใช้
3. ติดตั้ง Python dependencies: `ultralytics`, `onnxruntime` หรือ runtime ที่เหมาะกับ Jetson, `opencv`, `numpy`, `requests`, `pyserial`, `pymavlink`, `torch`, `torchvision`, `Pillow`, `mediapipe`, `Flask`, `flask-sock`, `websocket-client`
4. วาง `yolo11n.pt` ใน path ที่ `FOLLOW_YOLO_MODEL` ระบุ
5. วาง OSNet ONNX ใน path ที่ `FOLLOW_OSNET_REID_ONNX` ระบุ
6. ตรวจสิทธิ์ serial เช่น `/dev/ttyUSB0` และ Arduino port
7. ตรวจสิทธิ์ RPLIDAR และ baudrate
8. ตรวจกล้องและ URL camera
9. ตรวจว่า ONNX Runtime ใช้ CPU หรือ CUDA/TensorRT ตามที่ต้องการ
10. ทดสอบ emergency, stop, mode switch และการหยุดเมื่อ LiDAR พบ obstacle ก่อนทดสอบวิ่งจริง

## 10. สรุปสั้นที่สุด

- **YOLO11n** หา “คนอยู่ตรงไหน”
- **ByteTrack** ติดตาม “คนเดิมในหลายเฟรม”
- **OSNet** ตรวจว่า “คนนี้ใช่เจ้าของที่เลือกไว้ไหม”
- **MobileNetV3-Small/color matching** เป็น fallback ของ ReID
- **MediaPipe Pose** เป็น fallback ของ detector
- **RPLIDAR** ช่วยวัดระยะและความปลอดภัย
- **ROS 2 Foxy** เชื่อม node และ sensor
- **Jetson** รันระบบประมวลผลทั้งหมด
- **Arduino Mega 2560** รับคำสั่งระดับ PWM และควบคุม hardware จริง
- **Flask + rosbridge + web UI** เป็นช่องทางควบคุมและดูสถานะจากผู้ใช้
