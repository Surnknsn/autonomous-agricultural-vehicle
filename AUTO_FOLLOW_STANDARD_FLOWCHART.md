# Standard Flowchart: AUTO และ FOLLOW ME

## ประเภทของ Flowchart

ระบบนี้อธิบายได้ว่าเป็น:

1. **System Operational Flowchart**: แสดงลำดับการทำงานตั้งแต่ผู้ใช้สั่งงานจนถึงมอเตอร์
2. **Decision-Based Control Flowchart**: มีจุดตัดสินใจตรวจสอบความพร้อมและความปลอดภัย
3. **Closed-Loop Feedback Control**: รับ feedback จาก GPS, IMU, encoder, LiDAR และกล้อง แล้วปรับคำสั่งมอเตอร์ต่อเนื่อง

## สัญลักษณ์ที่ใช้

| รูปแบบ | ความหมายในแผนผัง |
|---|---|
| วงรี | จุดเริ่มต้นหรือสิ้นสุด |
| สี่เหลี่ยม | ขั้นตอนการประมวลผลหรือการควบคุม |
| สี่เหลี่ยมขนมเปียกปูน | จุดตรวจสอบเงื่อนไข/การตัดสินใจ |
| สี่เหลี่ยมด้านขนาน | ข้อมูลเข้า/ข้อมูลออก |
| ลูกศรวนกลับ | Feedback loop หรือการทำงานซ้ำ |

## 1. Flow หลักของระบบ

```mermaid
flowchart TD
    S([เริ่มต้นระบบ]) --> I[/รับคำสั่งจาก Web UI/]
    I --> D{เลือกโหมดการทำงาน}
    D -->|AUTO| A[AUTO Flow]
    D -->|FOLLOW ME| F[FOLLOW ME Flow]
    D -->|STOP หรือ EMERGENCY| X[หยุดมอเตอร์ทันที]
    A --> O[ส่งคำสั่ง PWM ไป Arduino]
    F --> O
    O --> M[Motor Driver ขับล้อซ้ายและขวา]
    M --> R[/รับ feedback จาก sensor/]
    R --> Q{ยังทำงานและปลอดภัยหรือไม่}
    Q -->|ใช่| D
    Q -->|ไม่ใช่| X
    X --> E([สิ้นสุดการเคลื่อนที่])
```

## 2. Flow มาตรฐานของ AUTO

```mermaid
flowchart TD
    S([เริ่ม AUTO]) --> W[/รับ waypoint จากแผนที่/]
    W --> P[สร้างและตรวจสอบ path]
    P --> G{GPS และ IMU พร้อมหรือไม่}
    G -->|ไม่พร้อม| H[หยุดรถและรอ sensor พร้อม]
    H --> G
    G -->|พร้อม| C{Arduino อยู่โหมด AUTO หรือไม่}
    C -->|ไม่ใช่| H2[หยุดรถและรอ hardware mode]
    H2 --> C
    C -->|ใช่| N{มี waypoint เหลือหรือไม่}
    N -->|ไม่มี| Z[จบภารกิจ AUTO]
    N -->|มี| T[คำนวณ heading และ cross-track error]
    T --> L[/อ่าน LiDAR และ encoder/]
    L --> B{มีสิ่งกีดขวางหรือไม่}
    B -->|มี| Y[หยุด/ชะลอ/หลบตาม safety logic]
    Y --> L
    B -->|ไม่มี| U[คำนวณความเร็วและ PWM ล้อซ้าย/ขวา]
    U --> K[ส่ง PWM ไป Arduino]
    K --> E{ถึง waypoint หรือยัง}
    E -->|ยังไม่ถึง| T
    E -->|ถึงแล้ว| N
    Z --> F([สิ้นสุด AUTO])
```

**คำอธิบายสำหรับรายงาน:**

AUTO เป็น **waypoint-based autonomous navigation** โดยใช้ GPS/IMU สำหรับระบุตำแหน่งและทิศทาง,
ใช้ encoder ตรวจการเคลื่อนที่จริง และใช้ LiDAR เป็นชั้นความปลอดภัยก่อนส่ง PWM ไปยังมอเตอร์

## 3. Flow มาตรฐานของ FOLLOW ME

```mermaid
flowchart TD
    S([เริ่ม FOLLOW ME]) --> O[/เลือก owner profile/]
    O --> P{มี profile และรูปอ้างอิงหรือไม่}
    P -->|ไม่มี| E1[แจ้งให้เลือก owner และหยุดรถ]
    E1 --> Z([สิ้นสุด FOLLOW])
    P -->|มี| C[เปิดกล้องและเตรียม detector/ReID]
    C --> D{กล้องและ detector พร้อมหรือไม่}
    D -->|ไม่พร้อม| E2[ส่ง STOP และแจ้งข้อผิดพลาด]
    E2 --> Z
    D -->|พร้อม| A{Arduino อยู่โหมด FOLLOW หรือไม่}
    A -->|ไม่ใช่| E3[หยุดรถและรอ hardware mode]
    E3 --> A
    A -->|ใช่| I[/รับภาพ, LiDAR และ odometry/]
    I --> H[ตรวจจับคนและเทียบ owner profile]
    H --> M{พบ owner ที่ถูกต้องหรือไม่}
    M -->|ไม่พบ| S1[ส่ง STOP หรือ search ตาม safety policy]
    S1 --> I
    M -->|พบ| R[รวมตำแหน่งภาพกับระยะ LiDAR]
    R --> Q{ระยะและสิ่งกีดขวางปลอดภัยหรือไม่}
    Q -->|ไม่ปลอดภัย| S2[หยุดหรือชะลอรถ]
    S2 --> I
    Q -->|ปลอดภัย| V[คำนวณทิศทางและระยะห่างเป้าหมาย]
    V --> U[สร้างคำสั่ง PWM ล้อซ้าย/ขวา]
    U --> K[ส่ง /follow_cmd ไป lawnmower_node]
    K --> M2{ยังได้รับ command ต่อเนื่องหรือไม่}
    M2 -->|ไม่| S3[หยุดรถจาก command timeout]
    S3 --> I
    M2 -->|ใช่| I
```

**คำอธิบายสำหรับรายงาน:**

FOLLOW ME เป็น **vision-based person following with LiDAR safety** โดยใช้กล้องและ AI
ตรวจจับ/ยืนยัน owner, ใช้ LiDAR ประเมินระยะและตรวจสิ่งกีดขวาง แล้วปรับ PWM แบบต่อเนื่อง
เพื่อรักษาทิศทางและระยะห่างจาก owner

## 4. Flow การประมวลผลภายใน AUTO

แผนผังนี้เน้นการคำนวณภายใน `lawnmower_node` หลังจากระบบได้รับคำสั่งเริ่ม AUTO แล้ว

```mermaid
flowchart TD
    S([เริ่มรอบควบคุม AUTO]) --> I[/รับข้อมูล sensor และคำสั่ง/]
    I --> I1[อ่าน waypoint เป้าหมาย]
    I1 --> I2[อ่าน GPS ตำแหน่งปัจจุบัน]
    I2 --> I3[อ่าน IMU yaw และ yaw rate]
    I3 --> I4[อ่าน encoder และความเร็วล้อ]
    I4 --> I5[อ่าน LiDAR scan]
    I5 --> C{ระบบพร้อมและอยู่โหมด AUTO หรือไม่}
    C -->|ไม่พร้อม| STOP[กำหนด PWM ซ้าย = 0<br/>PWM ขวา = 0]
    STOP --> END1([รอรอบควบคุมถัดไป])
    C -->|พร้อม| D[คำนวณระยะถึง waypoint]
    D --> R{ถึง waypoint แล้วหรือยัง}
    R -->|ถึงแล้ว| NEXT[เปลี่ยนไป waypoint ถัดไป]
    NEXT --> FIN{มี waypoint เหลือหรือไม่}
    FIN -->|ไม่มี| DONE[จบภารกิจและหยุดรถ]
    DONE --> END2([สิ้นสุด AUTO])
    FIN -->|มี| D
    R -->|ยังไม่ถึง| H[คำนวณ target heading]
    H --> E[คำนวณ heading error]
    E --> X[คำนวณ cross-track error]
    X --> L{LiDAR พบสิ่งกีดขวางหรือไม่}
    L -->|พบ| SAFE[หยุด/ชะลอ/หลบตาม safety logic]
    SAFE --> END1
    L -->|ไม่พบ| PID[ประมวลผล PID และกฎการเลี้ยว]
    PID --> V[คำนวณความเร็วพื้นฐานและ steering correction]
    V --> PWM[สร้าง PWM ล้อซ้ายและล้อขวา]
    PWM --> LIMIT[จำกัดค่า PWM และ slew rate]
    LIMIT --> OUT[/ส่ง PWM ผ่าน Serial ไป Arduino/]
    OUT --> FB[รถเคลื่อนที่และเกิด feedback]
    FB --> END1
```

### ข้อมูลที่ใช้ในการคำนวณ AUTO

| ขั้นตอน | ข้อมูลเข้า | ผลลัพธ์ |
|---|---|---|
| ระบุตำแหน่ง | GPS ปัจจุบัน + waypoint | ระยะและทิศทางไปเป้าหมาย |
| ระบุทิศทาง | target heading + IMU yaw | heading error |
| รักษาแนวเส้นทาง | ตำแหน่ง GPS + path | cross-track error |
| ตรวจการเคลื่อนที่ | encoder ซ้าย/ขวา | ความเร็วและระยะที่เคลื่อนที่จริง |
| ตรวจความปลอดภัย | LiDAR `/scan` | หยุด, ชะลอ หรืออนุญาตให้เคลื่อนที่ |
| สร้างคำสั่ง | error ต่าง ๆ + PID | PWM ล้อซ้าย/ขวา |

### สมการเชิงแนวคิดสำหรับรายงาน

```text
heading_error = target_heading - current_yaw
steering_correction = PID(heading_error, cross_track_error)
PWM_left  = base_speed + steering_correction
PWM_right = base_speed - steering_correction
```

ค่าจริงจะถูก normalize, จำกัดช่วง PWM และปรับ ramp ก่อนส่งไป Arduino เพื่อป้องกัน
การเปลี่ยนคำสั่งที่รุนแรงเกินไป

## 4. Feedback Loop ของระบบ

```mermaid
flowchart LR
    C[Controller<br/>lawnmower_node] --> PWM[คำสั่ง PWM]
    PWM --> R[Robot และมอเตอร์]
    R --> S[/Sensor feedback/]
    S --> C
    S --> G[GPS/IMU]
    S --> E[Encoder]
    S --> L[LiDAR]
    S --> V[Camera/AI<br/>เฉพาะ FOLLOW]
```

## 5. ประโยคสรุปสำหรับใส่รายงาน

> ระบบรถตัดหญ้านี้เป็นระบบควบคุมอัตโนมัติแบบแบ่งโหมดการทำงาน โดยโหมด AUTO ใช้การนำทางตาม waypoint และโหมด FOLLOW ME ใช้การตรวจจับบุคคลด้วยกล้องร่วมกับ LiDAR เพื่อกำหนดทิศทางและระยะห่าง การควบคุมเป็นแบบ closed-loop โดยรับข้อมูล feedback จาก GPS, IMU, encoder, LiDAR และกล้อง แล้วคำนวณคำสั่ง PWM สำหรับมอเตอร์อย่างต่อเนื่อง พร้อมมี decision logic สำหรับตรวจสอบความพร้อมและความปลอดภัยของระบบ

## 6. เทคโนโลยีที่ใช้ในระบบ

ส่วนนี้ควรแยกจาก Flowchart หลัก เพราะมีหน้าที่อธิบาย **เครื่องมือและเทคโนโลยีที่ใช้ประมวลผล** ไม่ใช่ลำดับการตัดสินใจของระบบ

| ส่วนระบบ | เทคโนโลยี/อุปกรณ์ | หน้าที่ |
|---|---|---|
| Middleware | ROS 2 Foxy | เชื่อม node, topic และข้อมูล sensor |
| Web control | Flask, HTML/JavaScript, roslibjs, rosbridge | หน้าเว็บ, แผนที่, ปุ่มสั่งงาน และสื่อสารกับ ROS |
| AUTO localization | GPS และ Pixhawk ผ่าน MAVLink | ตำแหน่งและทิศทางของรถ |
| AUTO control | Python, PID, waypoint/path control | คำนวณ heading, cross-track และ PWM |
| Obstacle safety | RPLIDAR และ `sensor_msgs/LaserScan` | ตรวจสิ่งกีดขวางและระยะด้านหน้า |
| FOLLOW detection | OpenCV, YOLO หรือ MediaPipe | รับภาพและตรวจจับบุคคล |
| FOLLOW identification | OSNet ReID, MobileNet และ color matching | ยืนยันว่าเป็น owner ที่เลือกไว้ |
| FOLLOW distance | กล้องร่วมกับ LiDAR | ประเมินทิศทางและระยะห่างจาก owner |
| Motor interface | Arduino, Serial และ PWM | รับคำสั่งล้อและควบคุมมอเตอร์ |
| Motion feedback | Encoder และ `/odom` | ตรวจการเคลื่อนที่จริงและช่วยปรับการควบคุม |

## 7. Technology Architecture

```mermaid
flowchart LR
    subgraph UI[User Interface Layer]
        Browser[Web Browser]
        Flask[Flask Web Server]
        Bridge[rosbridge + roslibjs]
        Browser --> Flask
        Browser --> Bridge
    end

    subgraph ROS[ROS 2 Foxy Layer]
        Main[lawnmower_node<br/>Python + PID]
        Follow[follow_tracker_node<br/>OpenCV + YOLO/MediaPipe + ReID]
        LidarNode[rplidar_node]
        Odom[odom_yaw_node<br/>optional]
        Bridge --> Main
        Bridge --> Follow
        LidarNode -->|/scan| Main
        LidarNode -->|/scan| Follow
        Odom -->|/odom| Follow
        Follow -->|/follow_cmd| Main
    end

    subgraph HW[Hardware Layer]
        Camera[Camera]
        Pixhawk[Pixhawk<br/>GPS + IMU/MAVLink]
        Rplidar[RPLIDAR]
        Arduino[Arduino<br/>Serial + PWM]
        Motors[Motor Driver + Wheels]
    end

    Camera --> Follow
    Pixhawk -->|MAVLink| Main
    Rplidar --> LidarNode
    Main -->|PWM over Serial| Arduino
    Arduino --> Motors
    Arduino -->|mode + encoder + status| Main
    Main -->|status topics| Bridge
    Follow -->|vision status/debug| Bridge
```

## 8. แนวทางจัดวางในรายงาน

เพื่อให้รายงานอ่านง่าย แนะนำแบ่งเป็น 3 รูป:

1. **System Flowchart**: ใช้ตอบว่า ระบบทำงานตามลำดับอย่างไร
2. **Technology Architecture**: ใช้ตอบว่า ระบบใช้ ROS 2, AI, sensor และ hardware อะไร
3. **Feedback Loop**: ใช้ตอบว่า sensor feedback กลับไปปรับคำสั่งมอเตอร์อย่างไร

ไม่ควรใส่ชื่อ library ทุกตัวลงใน Flowchart หลัก เพราะจะทำให้ผู้อ่านมองไม่เห็นลำดับการทำงาน
ให้ใส่ชื่อเทคโนโลยีไว้ใน Technology Architecture และตารางด้านบนแทน

## 9. Flow การควบคุม Manual จากรีโมท RC

ส่วนนี้อ้างอิงจากไฟล์ Arduino:
`C:\Users\Lenovo\Documents\Arduino\selectmode_Final-Emer\selectmode_Final-Emer.ino`

Manual เป็นการควบคุมแบบ **direct radio control** โดย Arduino อ่านสัญญาณ iBus จากรีโมท
แล้วแปลงค่าช่องสัญญาณเป็นทิศทางและ PWM ของล้อโดยตรง ไม่ผ่าน GPS, waypoint, PID หรือ AI

```mermaid
flowchart TD
    S([เริ่มระบบ Arduino]) --> SETUP[ตั้งค่า pin, iBus, encoder, servo และ CAN]
    SETUP --> STOP0[สั่งหยุดมอเตอร์เริ่มต้น]
    STOP0 --> LOOP([เริ่มรอบ loop])
    LOOP --> IBUS[อัปเดตข้อมูลรีโมทด้วย ibus.loop]
    IBUS --> MODE[อ่านสวิตช์โหมด D1/D2]
    MODE --> DEB{โหมดนิ่งเกินเวลา debounce หรือไม่}
    DEB -->|ไม่| KEEP[ใช้โหมดเดิมเพื่อป้องกันสัญญาณรบกวน]
    KEEP --> EMG
    DEB -->|ใช่| EMG{มี Web Emergency override หรือไม่}
    EMG -->|มี| WEBSTOP[ใช้คำสั่ง Web emergency เป็นลำดับความสำคัญสูงสุด]
    WEBSTOP --> MOTOR
    EMG -->|ไม่มี| MAN{โหมดปัจจุบันเป็น MANUAL หรือไม่}
    MAN -->|ไม่ใช่| OTHER[AUTO/FOLLOW รับ PWM จาก Jetson]
    OTHER --> MOTOR
    MAN -->|ใช่| READ[/อ่าน CH1 ถึง CH6 จากรีโมท/]
    READ --> VALID{CH1 และ CH2 ใช้งานได้หรือไม่}
    VALID -->|ไม่| SAFE[หยุดมอเตอร์และหยุดใบมีด]
    SAFE --> AUX
    VALID -->|ใช่| DB[ตั้ง deadband รอบค่ากลาง 1500]
    DB --> MIX[แปลง CH1/CH2 เป็น PWM ล้อซ้าย/ขวา]
    MIX --> MOTOR[ส่งทิศทางและ PWM ไป driver มอเตอร์]
    MOTOR --> AUX[ประมวลผลอุปกรณ์เสริม]
    AUX --> SERVO[CH3/CH4 -> filter -> servo 1/2]
    SERVO --> PUMP[CH6 -> debounce -> ปั๊มน้ำ]
    PUMP --> BLADE[CH5 -> debounce -> รีเลย์ใบมีด UP/DOWN/STOP]
    BLADE --> ENC[อ่าน encoder และคำนวณความเร็วล้อ]
    ENC --> STATUS[/ส่งสถานะ Serial ทุก 100 ms/]
    STATUS --> LOOP
```

### โครงสร้างสัญญาณจากรีโมท

| ช่องรีโมท | หน้าที่ | การประมวลผล |
|---|---|---|
| CH1 | แกนขับล้อขวา/คำสั่งด้านหนึ่ง | จำกัดช่วง `1000-2000` และ map เป็น PWM |
| CH2 | แกนขับล้อซ้าย/คำสั่งอีกด้าน | จำกัดช่วง `1000-2000` และ map เป็น PWM |
| CH3 | Servo 1 | filter, deadband, จำกัดมุม `60-180` องศา และจำกัด step |
| CH4 | Servo 2 | filter, deadband, จำกัดมุม `30-150` องศา และจำกัด step |
| CH5 | ใบมีด | เลือก `UP`, `DN` หรือ `STP` และ debounce `120 ms` |
| CH6 | ปั๊มน้ำ | เปิด/ปิดด้วย threshold และ debounce `120 ms` |

### การคำนวณมอเตอร์ Manual

โค้ดใช้การควบคุมแบบ **differential drive** โดยแยก PWM ของล้อซ้ายและขวา:

```text
pwmL = map(CH2, 1000..2000, 255..-255)
pwmR = map(CH1, 1000..2000, -255..255)
```

ก่อนขับจริงจะทำงานดังนี้:

1. ถ้าค่า CH1 หรือ CH2 ใกล้ค่ากลาง `1500` ภายใน deadband `35` ให้ถือว่าเป็นศูนย์
2. แยกเครื่องหมายของ PWM เป็นทิศทาง `-1`, `0`, `+1`
3. ใช้ค่าสัมบูรณ์เป็นความเร็ว PWM ช่วง `0-255`
4. เรียก `driveHardware()` เพื่อกำหนดขา IN/ PWM ของ motor driver
5. ถ้าทิศทางเป็นศูนย์ จะใช้ active brake และ PWM เป็นศูนย์

ดังนั้น Manual ไม่ได้คำนวณเส้นทางหรือแก้ heading แต่รับตำแหน่งคันบังคับจากผู้ควบคุม
แล้วแปลงเป็นความเร็วล้อทันที

### เงื่อนไขความปลอดภัยของ Manual

- ถ้า CH1 หรือ CH2 ต่ำกว่า `500` ซึ่งหมายถึงสัญญาณรีโมทผิดปกติ ระบบสั่งหยุดมอเตอร์
- การเปลี่ยนโหมดผ่าน D1/D2 ต้องนิ่งอย่างน้อย `80 ms` ก่อนยอมรับโหมดใหม่
- เมื่อเปลี่ยนโหมด ระบบจะสั่งหยุดมอเตอร์ก่อน เว้นแต่กำลังอยู่ใน Web Emergency override
- `EMG` จาก Jetson มี priority สูงกว่า Manual ปกติและ latch สถานะ emergency
- เมื่อไม่มีคำสั่ง Web emergency ใหม่และไม่ได้เปิด RC fallback ระบบจะหยุดรถ

### Feedback ของ Arduino ฝั่ง Manual

Encoder ล้อซ้าย/ขวาถูกอ่านผ่าน interrupt แล้วคำนวณเป็นความเร็วทุก `200 ms`
จากนั้นส่ง status CSV กลับไปยัง Jetson ทุก `100 ms` ประกอบด้วย mode, encoder,
สถานะปั๊ม/ใบมีด/emergency, battery CAN, ความเร็ว และ PWM ปัจจุบัน

## 10. สรุปประเภทการควบคุม Manual

Manual จากรีโมทจัดเป็น **Human-in-the-loop direct control** หรือ
**open-loop command mapping ที่มี safety feedback**:

- ผู้ควบคุมเป็นผู้กำหนดทิศทางและความเร็วผ่านรีโมท
- Arduino แปลงคำสั่งเป็น PWM โดยตรง
- ไม่มีการวางแผนเส้นทางหรือ PID เพื่อรักษา waypoint
- มี feedback สำหรับรายงาน encoder/ความเร็วและมี safety เช่น signal validation,
  deadband, debounce, active brake และ emergency override

## 11. คำอธิบายระบบแบบเข้าใจง่ายสำหรับใส่รายงาน

ส่วนนี้สรุปการทำงานโดยใช้หลักเดียวกันทั้ง 3 โหมด คือ
**รับข้อมูล -> ประมวลผล -> สั่งงาน -> ตรวจสอบผลลัพธ์**

### 11.1 Manual: ผู้ใช้เป็นคนขับ

ในโหมด Manual ผู้ควบคุมกดหรือโยกรีโมทเพื่อบอกว่าต้องการให้รถเดินหน้า ถอยหลัง
เลี้ยว หรือหยุด Arduino รับสัญญาณจากรีโมท แล้วแปลงเป็นความเร็วของล้อซ้ายและล้อขวา
โดยตรง

พูดง่าย ๆ คือ:

```text
รีโมท -> Arduino -> PWM ล้อ -> รถเคลื่อนที่
```

โหมดนี้รถไม่ได้คิดเส้นทางเอง และไม่ได้เลือกว่าจะไปที่ใด ผู้ควบคุมเป็นผู้ตัดสินใจทั้งหมด
Arduino มีหน้าที่อ่านคำสั่ง แปลงคำสั่ง ควบคุมมอเตอร์ และหยุดรถเมื่อสัญญาณผิดปกติ

### 11.2 AUTO: รถเป็นคนขับตามจุดหมาย

ในโหมด AUTO ผู้ใช้กำหนด waypoint บนแผนที่ก่อน จากนั้นรถจะใช้ GPS เพื่อตรวจว่าตัวเองอยู่ที่ใด
และใช้ IMU เพื่อตรวจว่ากำลังหันไปทางไหน

ระบบจะถามตัวเองซ้ำ ๆ ว่า:

1. ตอนนี้รถอยู่ที่ไหน
2. waypoint ถัดไปอยู่ทางไหน
3. รถหันตรงกับเป้าหมายหรือยัง
4. รถเบี่ยงออกจากเส้นทางมากแค่ไหน
5. ด้านหน้ามีสิ่งกีดขวางหรือไม่

จากคำตอบเหล่านี้ ระบบจะคำนวณว่าต้องเพิ่มหรือลดความเร็วล้อด้านใด แล้วส่ง PWM ไป Arduino

พูดง่าย ๆ คือ:

```text
Waypoint + GPS + IMU + Encoder + LiDAR
                 -> ตัวควบคุม AUTO
                 -> PWM ล้อ
                 -> รถเคลื่อนที่
                 -> sensor ส่งผลกลับมาแก้คำสั่ง
```

ดังนั้น AUTO คือการขับตาม waypoint แบบอัตโนมัติ และทำงานเป็นวงรอบจนกว่าจะถึงจุดหมาย
หรือพบเงื่อนไขที่ต้องหยุด

### 11.3 FOLLOW ME: รถเป็นคนตามเจ้าของ

ในโหมด FOLLOW ME ผู้ใช้เลือก owner profile ก่อน ระบบจะใช้กล้องตรวจหาคนในภาพ
แล้วเปรียบเทียบกับรูปอ้างอิงเพื่อยืนยันว่าคนใดคือ owner ที่ต้องติดตาม

หลังจากพบ owner ระบบจะพิจารณา:

1. owner อยู่ทางซ้ายหรือขวาของภาพ
2. owner อยู่ใกล้หรือไกลเกินไปหรือไม่
3. LiDAR ยืนยันตำแหน่งและระยะของ owner ได้หรือไม่
4. ด้านหน้ารถมีสิ่งกีดขวางหรือไม่

จากนั้นจึงสั่งให้รถเลี้ยว เดินหน้า ชะลอ หรือหยุด เพื่อรักษาทิศทางและระยะห่างจาก owner

พูดง่าย ๆ คือ:

```text
กล้อง -> ตรวจจับคน -> ยืนยัน owner -> ประเมินทิศทาง/ระยะ
                                      -> PWM ล้อ
                                      -> รถเคลื่อนที่
                                      -> กล้องและ LiDAR ตรวจซ้ำ
```

ถ้าหาระบุ owner ไม่ได้ กล้องขัดข้อง หรือ LiDAR พบอันตราย ระบบจะไม่สั่งให้รถวิ่งต่อ
แต่จะส่งคำสั่งหยุดหรือเข้าสู่การค้นหาตาม safety policy

### 11.4 เปรียบเทียบทั้ง 3 โหมด

| โหมด | ใครเป็นผู้ตัดสินใจทิศทาง | ข้อมูลหลัก | ลักษณะการควบคุม |
|---|---|---|---|
| Manual | ผู้ควบคุมรีโมท | iBus/RC | ควบคุมตรงแบบมนุษย์ |
| AUTO | โปรแกรมควบคุม | waypoint, GPS, IMU, encoder, LiDAR | นำทางอัตโนมัติตามจุดหมาย |
| FOLLOW ME | โปรแกรม AI และตัวควบคุม | กล้อง, owner profile, LiDAR, odometry | ติดตามบุคคลอัตโนมัติ |

### 11.5 สรุปสำหรับเขียนในรายงาน

ระบบแบ่งการควบคุมออกเป็น 3 โหมดตามแหล่งที่มาของคำสั่ง โหมด Manual รับคำสั่งโดยตรงจากผู้ควบคุมผ่านรีโมทและแปลงเป็น PWM สำหรับล้อ โหมด AUTO รับ waypoint และข้อมูลจาก GPS, IMU, encoder และ LiDAR เพื่อคำนวณทิศทางและความเร็วให้รถเดินทางไปยังจุดหมาย ส่วนโหมด FOLLOW ME ใช้กล้องและเทคโนโลยีตรวจจับบุคคลเพื่อยืนยัน owner และใช้ LiDAR ร่วมกับข้อมูลภาพในการควบคุมทิศทางและระยะห่าง ทั้งสามโหมดส่งคำสั่งสุดท้ายไปยัง Arduino เพื่อควบคุมมอเตอร์ และมีเงื่อนไขด้านความปลอดภัยสำหรับหยุดรถเมื่อข้อมูลไม่พร้อมหรือพบความเสี่ยง

## 12. เอกสารสำหรับนำเสนอในสไลด์

ถ้าต้องการนำเสนอให้เห็นภาพความเป็นวิศวกรรมของระบบ ให้ใช้เอกสารสรุปฉบับนี้:
[`AUTO_FOLLOW_ENGINEERING_SLIDES.md`](AUTO_FOLLOW_ENGINEERING_SLIDES.md)
