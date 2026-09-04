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
