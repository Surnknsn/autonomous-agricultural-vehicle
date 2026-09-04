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
