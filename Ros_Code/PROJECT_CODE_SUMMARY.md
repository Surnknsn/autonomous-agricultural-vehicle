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

## 11. ตารางสรุปทุก ROS package และไฟล์หลัก

| Package/ไฟล์ | ภาษา/ชนิด | ใช้ทำอะไร | สถานะในระบบ |
|---|---|---|---|
| `src/lawnmower_control` | Python ROS 2 | ชุดควบคุมรถ, serial, AUTO/FOLLOW และ camera tracking | ส่วนหลัก |
| `lawnmower_node.py` | Python ROS 2 node | คุยกับ Arduino, อ่านสถานะ, ส่ง PWM และจัดการคำสั่งรถ | ส่วนหลัก |
| `follow_tracker_node.py` | Python ROS 2 node | YOLO, ByteTrack, OSNet, LiDAR fusion และ FOLLOW control | ส่วนหลักของ FOLLOW |
| `camera_lane_assist_node.py` | Python ROS 2 node | ประมวลผลภาพ/ช่วยงาน lane assist | ส่วนช่วยของ AUTO |
| `src/rplidar_ros` | C++ ROS 2 driver | คุยกับ RPLIDAR และ publish `LaserScan` | sensor หลักเมื่อเปิด LiDAR |
| `src/amr_description` | URDF/Xacro/CMake | โมเดลตัวรถ, frame, mesh และ RViz visualization | visualization |
| `src/odom_yaw` | Python ROS 2 | ประมวลผล odometry/yaw | ใช้เมื่อ launch เรียก |
| `src/my_mavros_launch` | Python launch | launch ชุด MAVROS/PX4 | integration/launch เสริม |
| `src/px4_mavros_bridge` | C++/Python launch/bridge | bridge ระหว่าง PX4/MAVROS กับระบบรถ | code เสริม/ต้องตรวจ startup |
| `arduino/mega2560_control` | Arduino C++ | ควบคุม hardware ระดับมอเตอร์และอุปกรณ์จริง | controller ฝั่ง hardware |
| `web_app/app_web.py` | Flask Python | HTTP API, camera endpoint, dataset และ web control | web backend |
| `web/templates/index.html` | HTML/JavaScript | หน้า dashboard, map, camera และ controls | web frontend |
| `web/static/` | JavaScript/CSS/assets | icon, PWA, offline page และ client logic | web frontend |
| `web/datalog/` | CSV/JSONL | เก็บ mission/status/training log | data/runtime |
| `models/reid/` | ONNX | เก็บโมเดล OSNet ReID | model asset |

### 11.1 ROS message และข้อมูลที่เกี่ยวข้อง

| Message/API | ใช้ทำอะไร |
|---|---|
| `std_msgs/String` | สถานะ, command, text state และข้อมูลที่ serialize เป็น string |
| `std_msgs/Bool` | flag เช่น enable/disable หรือสถานะเปิดปิด |
| `geometry_msgs` | vector, pose, twist หรือข้อมูลเรขาคณิตของรถ/ทิศทาง |
| `sensor_msgs/LaserScan` | ระยะ LiDAR รอบรถและ obstacle safety |
| `nav_msgs/Odometry` | ตำแหน่ง/ความเร็ว/heading จาก odometry |
| ROS parameters/environment | ปรับ model path, threshold, camera, ReID และ safety โดยไม่แก้ source |

## 12. ตาราง Python library แบบละเอียด

| Library/import | ส่วนที่ใช้ | หน้าที่ในโปรเจกต์ |
|---|---|---|
| `rclpy` | ROS Python nodes | สร้าง node, publisher, subscriber, timer, parameter และ logger |
| `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs` | ROS messages | รูปแบบข้อมูลที่ node รับส่งกัน |
| `tf2_ros` | ROS transform | แปลงความสัมพันธ์ระหว่าง frame ของ robot/sensor |
| `cv2` | camera/follow/web | อ่านภาพ, resize, BGR/RGB conversion, crop, HSV และ image measurement |
| `numpy` | ทุกงาน vision/ReID | array, vector, normalization, dot product, distance และ numerical calculation |
| `ultralytics` | `follow_tracker_node.py` | โหลด `yolo11n.pt`, detect person และเรียก tracker |
| `onnxruntime` | OSNet ReID | เปิด session และรัน `osnet_x0_25_msmt17_b1.onnx` |
| `torch` | ReID fallback | tensor, inference mode, device CPU/CUDA และ model execution |
| `torchvision` | ReID fallback | โหลด `MobileNet_V3_Small_Weights.DEFAULT` และ preprocessing |
| `PIL.Image` | ReID fallback | แปลงภาพ OpenCV เป็นรูปแบบที่ torchvision ใช้ |
| `mediapipe` | detector/pose fallback | ตรวจ pose/person เมื่อเลือกโหมด fallback หรือ auto fallback |
| `requests` | camera/web integration | เรียก snapshot, HTTP endpoint และ API ภายในระบบ |
| `serial` (`pyserial`) | `lawnmower_node.py` | เปิด serial port ไป Arduino และส่ง/รับ command/status |
| `pymavlink.mavutil` | `lawnmower_node.py`/PX4 code | สร้าง connection และสื่อสาร protocol MAVLink |
| `Flask` | `web_app/app_web.py` | web server, route, response, template และ JSON API |
| `flask_sock.Sock` | web backend | เปิด WebSocket ฝั่ง Flask สำหรับการเชื่อมต่อแบบ realtime |
| `websocket` | web/rosbridge | เชื่อม WebSocket ไป rosbridge หรือ service ที่เกี่ยวข้อง |
| `werkzeug.secure_filename` | web upload | ทำชื่อไฟล์ upload ให้ปลอดภัยก่อนบันทึก |
| `json` | ROS/web/datalog | serialize และ parse state/config/profile/command |
| `threading` | web/backend | ทำงานรับส่งหรือ stream บางส่วนแยกจาก request หลัก |
| `pathlib.Path` | ทุก package | จัดการ path ของ model, dataset, profile และไฟล์ config |
| `glob` | camera/serial discovery | ค้นหา device หรือไฟล์ตาม pattern |
| `math`, `statistics`, `re`, `time` | control/vision | มุม, filter, parsing, timing และสถิติของระบบควบคุม |

ไม่มี version ของ Python library เหล่านี้ถูก pin ไว้ใน `package.xml` หรือไฟล์ requirements ที่อยู่ใน source ชุดนี้ ดังนั้นรุ่นที่ใช้จริงต้องตรวจจาก environment ของ Jetson เช่น `pip freeze` เพิ่มเติม

## 13. Arduino: ตารางทุก library, pin และฟังก์ชัน

### 13.1 Library Arduino

| Include | ใช้กับส่วนไหน | หน้าที่ |
|---|---|---|
| `Arduino.h` | core | `setup`, `loop`, GPIO, PWM, timing, string และ utility พื้นฐาน |
| `IBusBM.h` | RC receiver | อ่านค่า iBus channel 1-6 จากรีโมต |
| `Servo.h` | servo 1/2 | สร้าง pulse และสั่งมุม servo |
| `SPI.h` | MCP CAN | เปิด SPI bus ระหว่าง Mega กับ CAN module |
| `mcp_can.h` | battery CAN | อ่าน frame จาก BMS และตั้ง CAN bitrate |

### 13.2 Pin map ทุกจุด

| Macro | Pin | อุปกรณ์/สัญญาณ | หน้าที่ในโค้ด |
|---|---:|---|---|
| `EMER_CHECK_PIN` | 41 | emergency input สำรอง | ใช้เมื่อ `USE_EMER_CHECK_PIN=1`; ปัจจุบันปิดไว้ |
| `D1_PIN` | 48 | mode switch D1 | เลือก AUTO/FOLLOW/MANUAL ร่วมกับ D2 |
| `D2_PIN` | 49 | mode switch D2 | เลือก mode ร่วมกับ D1 |
| `LAMP_AUTO_MODE` | 22 | ไฟแสดง AUTO/FOLLOW | HIGH เมื่อไม่ใช่ MANUAL |
| `MANUAL_STATUS_PIN` | 26 | ไฟแสดง MANUAL | HIGH เมื่ออยู่ MANUAL |
| `L_PWM` | 11 | motor driver ซ้าย PWM | ปรับกำลังมอเตอร์ซ้าย 0-255 |
| `L_INA` | 36 | motor driver ซ้าย direction A | กำหนดทิศทาง |
| `L_INB` | 35 | motor driver ซ้าย direction B | กำหนดทิศทาง |
| `R_PWM` | 10 | motor driver ขวา PWM | ปรับกำลังมอเตอร์ขวา 0-255 |
| `R_INA` | 41 | motor driver ขวา direction A | กำหนดทิศทาง |
| `R_INB` | 40 | motor driver ขวา direction B | กำหนดทิศทาง |
| `BLADE_RELAY_UP` | 5 | relay ใบมีด | สั่งทิศ UP |
| `BLADE_RELAY_DN` | 6 | relay ใบมีด | สั่งทิศ DOWN |
| `WATER_PUMP` | 4 | relay/driver ปั๊มน้ำ | เปิดปิดปั๊มน้ำ |
| `SERVO1_PIN` | 12 | servo 1 | รับ CH3 และจำกัดมุม 60-180 |
| `SERVO2_PIN` | 13 | servo 2 | รับ CH4 และจำกัดมุม 30-150 |
| `ENC_L_A` | 18 | encoder ซ้าย phase A | interrupt นับ encoder |
| `ENC_L_B` | 17 | encoder ซ้าย phase B | ระบุทิศการหมุน |
| `ENC_R_A` | 20 | encoder ขวา phase A | interrupt นับ encoder |
| `ENC_R_B` | 21 | encoder ขวา phase B | ระบุทิศการหมุน |
| `CAN_CS` | 53 | MCP CAN chip select | เลือก CAN module ผ่าน SPI |
| `CAN_INT` | 2 | MCP CAN interrupt | แจ้งว่ามี CAN frame ใหม่ |

ข้อควรระวัง: `EMER_CHECK_PIN` และ `R_INA` ใช้หมายเลข pin `41` เหมือนกัน แต่ emergency input ถูกปิดด้วย `USE_EMER_CHECK_PIN 0` ขณะที่ `R_INA` ถูกใช้เป็น direction output จริง หากเปิด emergency input ต้องตรวจวงจรและแก้ pin conflict ก่อน

### 13.3 ค่าควบคุมที่มีผลต่อพฤติกรรม

| ค่าคงที่ | ค่า | ผล |
|---|---:|---|
| `MANUAL_CENTER` | 1500 | จุดกลาง channel RC |
| `MANUAL_DEADBAND` | 35 | ช่วงนิ่งรอบจุดกลาง |
| `DRIVE_CMD_CONFIRM_CYCLES` | 3 | ต้องเห็นคำสั่งขับต่อเนื่องก่อนเริ่มมอเตอร์ |
| `SERVO_MAX_STEP` | 2 | จำกัดการเปลี่ยนมุม servo ต่อ loop ให้เคลื่อนนุ่ม |
| `CH6_ON_THRESH/OFF_THRESH` | 1700/1300 | hysteresis ของปั๊มน้ำ |
| `CH5_UP_ON_THRESH` | 1680 | ค่า CH5 สำหรับสถานะใบมีด DN ตาม mapping ใน source |
| `CH5_DN_ON_THRESH` | 1320 | ค่า CH5 สำหรับสถานะใบมีด UP ตาม mapping ใน source |
| `CH5_CENTER_LOW/HIGH` | 1420/1580 | ช่วงหยุดใบมีด |
| `MODE_DEBOUNCE_MS` | 80 ms | debounce mode switch |
| `CH6_DEBOUNCE_MS` | 120 ms | debounce ปั๊มน้ำ |
| `CH5_DEBOUNCE_MS` | 120 ms | debounce ใบมีด |
| `WEB_MANUAL_OVERRIDE_HOLD_MS` | 800 ms | อายุคำสั่ง web PWM แบบ emergency |
| `WEB_EMER_ALLOW_RC_FALLBACK` | 0 | ไม่มี RC fallback ขณะ web emergency |
| `JETSON_INVERT_L/R` | 0/0 | ไม่กลับทิศเฉพาะคำสั่ง Jetson |
| `HW_INVERT_L/R` | 0/0 | ไม่กลับทิศระดับ hardware |

## 14. Protocol ระหว่าง Jetson กับ Arduino แบบละเอียด

### 14.1 คำสั่งจาก Jetson ไป Arduino

| รูปแบบ | เงื่อนไข | ผลลัพธ์ |
|---|---|---|
| `PWM,left,right` | AUTO/FOLLOW หรือ web direct path | จำกัดค่าที่ -255..255 แล้วขับมอเตอร์ซ้าย/ขวา |
| `MANUAL,left,right` | web emergency | เปิด software emergency และขับตาม PWM ที่กำหนด |
| `EMG` | ทุก mode | latch emergency, clear command และหยุดมอเตอร์ |
| `RST` | ทุก mode | ยกเลิก software emergency และหยุดก่อนรับคำสั่งใหม่ |

ทุก command จบด้วย newline และ Arduino อ่านด้วย `pollJetsonSerial()` ที่ `115200 baud`

### 14.2 สถานะจาก Arduino ไป Jetson

บรรทัดข้อมูลหลักมี field ตามลำดับ:

```text
mode,encL,encR,pump,blade,manual_led,auto_lamp,emg,can_soc,can_voltage,can_current,can_temp,can_status
```

ความหมาย:

- `mode`: 0 MANUAL, 1 AUTO, 2 FOLLOW
- `encL`, `encR`: encoder ซ้าย/ขวา
- `pump`: 0/1
- `blade`: 0 STOP, 1 UP, 2 DOWN
- `manual_led`, `auto_lamp`: สถานะไฟแสดง mode
- `emg`: software emergency 0/1
- `can_soc`, `can_voltage`, `can_current`, `can_temp`: ค่าจาก BMS
- `can_status`: 0 NORMAL, 1 WARNING/PROTECTION, 2 UNKNOWN

สถานะหลักถูกส่งทุก `100 ms` หรือประมาณ `10 Hz` และ debug log ถูกส่งทุกประมาณ `250 ms`

## 15. รุ่น/ไฟล์ที่ยืนยันได้และสิ่งที่ยังไม่มี version

| รายการ | สิ่งที่ยืนยันจาก source | สิ่งที่ยังต้องตรวจจากเครื่องจริง |
|---|---|---|
| ROS | Foxy ตามชื่อ workspace/เอกสาร | patch release ของ ROS 2 Foxy |
| `rplidar_ros` | package version 2.1.4 | firmware ของ RPLIDAR |
| YOLO | `yolo11n.pt` และ Ultralytics API | version ของ `ultralytics`, checksum/ไฟล์ weight จริง |
| ByteTrack | `bytetrack.yaml` ผ่าน Ultralytics | config ที่ติดตั้งจริงและ version tracker API |
| OSNet | `osnet_x0_25_msmt17_b1.onnx` | ONNX opset, checksum และ provider ที่เครื่องใช้จริง |
| MobileNet | `MobileNet_V3_Small_Weights.DEFAULT` | version ของ torch/torchvision และ weight cache |
| Arduino | Mega 2560, CAN 500 kbps, MCP 8 MHz | version Arduino core และ versions ของ IBusBM/Servo/MCP CAN |
| Web | Flask, flask-sock, websocket และ roslibjs | package versions และ rosbridge version |

## 16. สรุปบทบาทแบบหนึ่งบรรทัดต่อส่วน

- `YOLO11n`: ตรวจจับ **ตำแหน่งคน**
- `ByteTrack`: รักษา **track ต่อเนื่องของ detection ระหว่างเฟรม**
- `OSNet`: ตรวจ **ความเหมือนของบุคคลกับ owner profile**
- `MobileNetV3-Small`: ReID fallback เมื่อ OSNet ใช้ไม่ได้
- `MediaPipe Pose`: detector/pose fallback เมื่อ YOLO ใช้ไม่ได้หรือเลือกโหมดนี้
- `HSV/color`: fallback ระดับ appearance ที่เบาที่สุด
- `OpenCV`: เปิดและแปลงภาพ รวมถึงวัด bounding box
- `NumPy`: คำนวณ vector/score/ระยะ
- `RPLIDAR`: วัด obstacle และระยะรอบรถ
- `ROS 2 Foxy`: เชื่อมทุก node และ message
- `Flask`: เปิด web API และ server
- `rosbridge`: พา browser คุยกับ ROS
- `Arduino Mega 2560`: ตัดสิน mode/emergency และขับ hardware จริง
- `IBusBM`: รับคำสั่งรีโมต
- `Servo`: ขับ servo
- `MCP CAN`: อ่าน battery/BMS
- `Serial`: ช่องทาง command/status ระหว่าง Jetson กับ Arduino
