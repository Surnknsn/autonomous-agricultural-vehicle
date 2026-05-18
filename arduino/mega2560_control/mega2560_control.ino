#include <Arduino.h>
#include <IBusBM.h>
#include <Servo.h>
#include <SPI.h>
#include <mcp_can.h>

/* ================= PINS CONFIGURATION ================= */
#define EMER_CHECK_PIN    41  
#define USE_EMER_CHECK_PIN 0
#define D1_PIN            48
#define D2_PIN            49
#define LAMP_AUTO_MODE    22
#define MANUAL_STATUS_PIN 26 

// SMILE Driver Pins (Mode 1-2)
#define L_PWM 11
#define L_INA 36
#define L_INB 35
#define R_PWM 10
#define R_INA 41 
#define R_INB 40 

// Auxiliary Systems
// Blade switched by 2 relays (UP/DOWN), BTS driver removed.
#define BLADE_RELAY_UP 5
#define BLADE_RELAY_DN 6
#define WATER_PUMP   4
#define SERVO1_PIN   12 
#define SERVO2_PIN   13 

// Encoders
#define ENC_L_A 18
#define ENC_L_B 17
#define ENC_R_A 20
#define ENC_R_B 21

// CAN Debug (Daly/Pylontech)
#define CAN_CS  53
#define CAN_INT 2

// Jetson AUTO/FOLLOW PWM direction fix (manual RC path unaffected)
// Set to 1 to invert that side, 0 to keep as-is.
#define JETSON_INVERT_L 0
#define JETSON_INVERT_R 0
// Hardware direction invert (applies to all control paths)
#define HW_INVERT_L 0
#define HW_INVERT_R 0
// Set to 1 only if you want RC fallback while web-emergency is active and no fresh web cmd.
#define WEB_EMER_ALLOW_RC_FALLBACK 0

/* ================= GLOBAL VARIABLES ================= */
IBusBM ibus;
Servo s1, s2;
MCP_CAN CAN(CAN_CS);

volatile long encL = 0, encR = 0;

String inputString = "";
int current_pwmL = 0, current_pwmR = 0; 
int s1_angle = 90, s2_angle = 90;
bool pump_on = false;
String blade_status = "STP";
bool software_emergency = false;
const bool ENABLE_DEBUG_LOG = true;
String web_emg_manual_cmd = "STOP";
const int WEB_EMG_SPEED = 170;
unsigned long web_manual_override_until = 0;
const unsigned long WEB_MANUAL_OVERRIDE_HOLD_MS = 800;
bool web_emg_pwm_active = false;
int web_emg_pwm_l = 0;
int web_emg_pwm_r = 0;
const int CH6_ON_THRESH = 1700;
const int CH6_OFF_THRESH = 1300;
const unsigned long CH6_DEBOUNCE_MS = 120;
const int CH5_UP_ON_THRESH = 1680;
const int CH5_DN_ON_THRESH = 1320;
const int CH5_CENTER_LOW = 1420;
const int CH5_CENTER_HIGH = 1580;
const unsigned long CH5_DEBOUNCE_MS = 120;
const int MANUAL_CENTER = 1500;
const int MANUAL_DEADBAND = 35;
const int DRIVE_CMD_CONFIRM_CYCLES = 3;
const int SERVO_CENTER = 1500;
const int SERVO_DEADBAND = 25;
const int SERVO_FILTER_NUM = 5;   // filtered = (prev*5 + raw)/6
const int SERVO_FILTER_DEN = 6;
const int SERVO_MAX_STEP = 2;     // smaller = smoother
const int SERVO1_MIN_ANGLE = 60;
const int SERVO1_MAX_ANGLE = 180;
const int SERVO2_MIN_ANGLE = 30;
const int SERVO2_MAX_ANGLE = 150;
bool ch6_state = false;
bool ch6_candidate = false;
unsigned long ch6_candidate_since = 0;
int ch6_last_valid = 1500;
String blade_candidate = "STP";
unsigned long blade_candidate_since = 0;
int dbg_ch1 = 1500;
int dbg_ch2 = 1500;
int dbg_dir_l = 0;
int dbg_dir_r = 0;
int dbg_l_ina = 0;
int dbg_l_inb = 0;
int dbg_r_ina = 0;
int dbg_r_inb = 0;
int drive_cmd_confirm_count[3] = {0, 0, 0};  // index 1..2
int ch3_filtered = SERVO_CENTER;
int ch4_filtered = SERVO_CENTER;
const bool ENABLE_CAN_DEBUG = true;
int can_soc = 0;
int can_soh = 0;
float can_voltage = 0;
float can_current = 0;
float can_temperature = 0;
float can_charge_limit = 0;
float can_discharge_limit = 0;
String can_status_text = "UNKNOWN";

/* ===== MODE ===== */
enum Mode { MODE_MANUAL = 0, MODE_AUTO = 1, MODE_FOLLOW = 2 };
Mode currentMode = MODE_MANUAL, lastMode = MODE_MANUAL;
const unsigned long MODE_DEBOUNCE_MS = 80;

/* ================= FORWARD DECLARATIONS ================= */
void stopMotors();
void sendStatusSerial();
void handleAuto();
void handleManual();
Mode readMode();
Mode readModeRaw(bool &valid);
String getModeName(Mode m);
void processMotor(int motor, long val);
void driveHardware(int motor, int dir, int speed);
void driveJetsonPwm(int pL, int pR);
void pollJetsonSerial();
void processJetsonCommand(const String& cmd);
void applyWebEmergencyManual();
void handleEmergencyControl();
bool readCh6Stable();
void initCanDebug();
void pollCanDebug();
String decodeCanStatus(const unsigned char *buf);

/* ===== TIMERS ===== */
unsigned long lastLogTime = 0;
unsigned long lastStatusSend = 0;
unsigned long lastRxRawLogTime = 0;

/* ================= ENCODER ISR ================= */
void isrLA() { if (digitalRead(ENC_L_B)) encL--; else encL++; }
void isrRA() { if (digitalRead(ENC_R_B)) encR--; else encR++; }

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  ibus.begin(Serial1);
  initCanDebug();

#if USE_EMER_CHECK_PIN
  pinMode(EMER_CHECK_PIN, INPUT_PULLUP);
#endif
  pinMode(D1_PIN, INPUT_PULLUP);
  pinMode(D2_PIN, INPUT_PULLUP);
  pinMode(LAMP_AUTO_MODE, OUTPUT);
  pinMode(MANUAL_STATUS_PIN, OUTPUT);

  pinMode(L_PWM, OUTPUT); pinMode(L_INA, OUTPUT); pinMode(L_INB, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_INA, OUTPUT); pinMode(R_INB, OUTPUT);
  pinMode(BLADE_RELAY_UP, OUTPUT);
  pinMode(BLADE_RELAY_DN, OUTPUT);
  pinMode(WATER_PUMP, OUTPUT);
  digitalWrite(BLADE_RELAY_UP, LOW);
  digitalWrite(BLADE_RELAY_DN, LOW);

  s1.attach(SERVO1_PIN); 
  s2.attach(SERVO2_PIN);

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRA, RISING);

  stopMotors();
  Serial.print("# SYSTEM READY v12.5 (EMER-WEB+RC) ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
}

/* ================= MAIN LOOP ================= */
void loop() {
  // Keep iBus parser updated every cycle to avoid stale/drop channel reads.
  ibus.loop();
  pollJetsonSerial();
  pollCanDebug();

  bool webEmergencyActive = software_emergency || (millis() < web_manual_override_until);
  bool emergencyActive = webEmergencyActive;

  currentMode = readMode();
  digitalWrite(MANUAL_STATUS_PIN, (currentMode == MODE_MANUAL));
  digitalWrite(LAMP_AUTO_MODE, (currentMode != MODE_MANUAL));

  if (currentMode != lastMode) {
    // In web-emergency state, mode switch must not interrupt joystick override.
    if (!webEmergencyActive) {
      stopMotors();
    }
    lastMode = currentMode;
    // Keep parser buffer intact; clearing here can drop partial MANUAL packets.
  }

  if (webEmergencyActive) {
    // Highest priority override: cut MANUAL/AUTO/FOLLOW logic.
    // In this state, web joystick has priority; if no recent web command, allow RC manual fallback.
    handleEmergencyControl();
  } else if (currentMode == MODE_MANUAL) {
    handleManual();
  } else {
    handleAuto();
  }

  /* ===== SEND STATUS TO JETSON (10Hz - เสถียรกว่า 20Hz) ===== */
  if (millis() - lastStatusSend >= 100) {
    lastStatusSend = millis();
    sendStatusSerial();
  }

  /* ===== DEBUG LOG (Every 250ms) ===== */
  if (ENABLE_DEBUG_LOG && millis() - lastLogTime > 250) {
    lastLogTime = millis();
    Serial.print("# LOG ["); Serial.print(getModeName(currentMode)); Serial.print("]");
    Serial.print(" EMG:"); Serial.print(emergencyActive ? 1 : 0);
    Serial.print(" CH:"); Serial.print(dbg_ch1); Serial.print(","); Serial.print(dbg_ch2);
    Serial.print(" DIR:"); Serial.print(dbg_dir_l); Serial.print(","); Serial.print(dbg_dir_r);
    Serial.print(" PWM:"); Serial.print(current_pwmL); Serial.print(","); Serial.print(current_pwmR);
    Serial.print(" PINL:"); Serial.print(dbg_l_ina); Serial.print(","); Serial.print(dbg_l_inb);
    Serial.print(" PINR:"); Serial.print(dbg_r_ina); Serial.print(","); Serial.print(dbg_r_inb);
    Serial.print(" ENC:"); Serial.print(encL); Serial.print(","); Serial.println(encR);
  }
}

/* ================= FUNCTIONS ================= */

void sendStatusSerial() {
  // ส่งข้อมูลสถานะ:
  // mode,encL,encR,pump,blade,manual_led,auto_lamp,emg,can_soc,can_voltage,can_current,can_temp,can_status
  // blade: 0=STOP, 1=UP, 2=DOWN
  // can_status: 0=NORMAL, 1=WARNING/PROTECTION, 2=UNKNOWN
  int bladeCode = 0;
  if (blade_status == "UP") bladeCode = 1;
  else if (blade_status == "DN") bladeCode = 2;

  int canStatusCode = 2;
  if (can_status_text == "NORMAL") canStatusCode = 0;
  else if (can_status_text == "WARNING / PROTECTION") canStatusCode = 1;

  Serial.print((int)currentMode);
  Serial.print(",");
  Serial.print(encL);
  Serial.print(",");
  Serial.print(encR);
  Serial.print(",");
  Serial.print(pump_on ? 1 : 0);
  Serial.print(",");
  Serial.print(bladeCode);
  Serial.print(",");
  Serial.print((currentMode == MODE_MANUAL) ? 1 : 0);
  Serial.print(",");
  Serial.print((currentMode != MODE_MANUAL) ? 1 : 0);
  Serial.print(",");
  Serial.print(software_emergency ? 1 : 0);
  Serial.print(",");
  Serial.print(can_soc);
  Serial.print(",");
  Serial.print(can_voltage, 2);
  Serial.print(",");
  Serial.print(can_current, 2);
  Serial.print(",");
  Serial.print(can_temperature, 1);
  Serial.print(",");
  Serial.println(canStatusCode);

  // Human-readable debug line (Arduino side)
  if (ENABLE_DEBUG_LOG) {
    Serial.print("# TX mode="); Serial.print((int)currentMode);
    Serial.print(" encL="); Serial.print(encL);
    Serial.print(" encR="); Serial.print(encR);
    Serial.print(" pump="); Serial.print(pump_on ? 1 : 0);
    Serial.print(" blade="); Serial.print(bladeCode);
    Serial.print(" man_led="); Serial.print((currentMode == MODE_MANUAL) ? 1 : 0);
    Serial.print(" auto_lamp="); Serial.print((currentMode != MODE_MANUAL) ? 1 : 0);
    Serial.print(" emg="); Serial.print(software_emergency ? 1 : 0);
    Serial.print(" can_soc="); Serial.print(can_soc);
    Serial.print(" can_v="); Serial.print(can_voltage, 2);
    Serial.print(" can_i="); Serial.print(can_current, 2);
    Serial.print(" can_t="); Serial.print(can_temperature, 1);
    Serial.print(" can_st="); Serial.println(canStatusCode);
  }
}

void handleAuto() {
  // Auto/FOLLOW motor control is handled in processJetsonCommand() from Serial lines.
}

// --- ฟังก์ชันอื่นๆ คงเดิมตามสถาปัตยกรรมของคุณ ---

Mode readMode() {
  static Mode stableMode = MODE_MANUAL;
  static Mode candidateMode = MODE_MANUAL;
  static unsigned long candidateSince = 0;

  bool valid = false;
  Mode raw = readModeRaw(valid);
  unsigned long now = millis();

  if (!valid) {
    // Invalid electrical state (both LOW) during switch transition/noise.
    // Keep last stable mode to prevent random jumps.
    return stableMode;
  }

  if (raw != candidateMode) {
    candidateMode = raw;
    candidateSince = now;
  }

  if (raw != stableMode && (now - candidateSince) >= MODE_DEBOUNCE_MS) {
    stableMode = raw;
  }

  return stableMode;
}

Mode readModeRaw(bool &valid) {
  bool d1 = digitalRead(D1_PIN);
  bool d2 = digitalRead(D2_PIN);
  valid = true;
  // Switch mapping (confirmed in your setup):
  // 0,0 = MANUAL | 0,1 = AUTO | 1,0 = FOLLOW
  if (!d1 && !d2) return MODE_MANUAL; // 0,0
  if (!d1 && d2) return MODE_AUTO;    // 0,1
  if (d1 && !d2) return MODE_FOLLOW;  // 1,0
  // 1,1 can appear from pull-up/floating state; keep MANUAL as safe fallback.
  return MODE_MANUAL;
}

String getModeName(Mode m) {
  if (m == MODE_MANUAL) return "MAN";
  if (m == MODE_AUTO) return "AUTO";
  return "FOLW";
}

void handleManual() {
  long ch1 = ibus.readChannel(0); 
  long ch2 = ibus.readChannel(1); 
  dbg_ch1 = (int)ch1;
  dbg_ch2 = (int)ch2;
  if (ch1 < 500 || ch2 < 500) {
    stopMotors();
    digitalWrite(BLADE_RELAY_UP, LOW);
    digitalWrite(BLADE_RELAY_DN, LOW);
    blade_status = "STP";
    return;
  }

  // Manual direct mapping as requested:
  // Forward  : ch1=2000, ch2=2000 -> PWM -255, +255
  // Backward : ch1=1000, ch2=1000 -> PWM +255, -255
  if (abs((int)ch1 - MANUAL_CENTER) <= MANUAL_DEADBAND) ch1 = MANUAL_CENTER;
  if (abs((int)ch2 - MANUAL_CENTER) <= MANUAL_DEADBAND) ch2 = MANUAL_CENTER;

  // Swap CH1/CH2 contribution for steering axis while keeping FWD/BACK behavior.
  int pwmL = map((int)constrain(ch2, 1000, 2000), 1000, 2000, 255, -255);
  int pwmR = map((int)constrain(ch1, 1000, 2000), 1000, 2000, -255, 255);

  dbg_dir_l = (pwmL > 0) - (pwmL < 0);
  dbg_dir_r = (pwmR > 0) - (pwmR < 0);
  driveHardware(1, dbg_dir_l, abs(pwmL));
  driveHardware(2, dbg_dir_r, abs(pwmR));

  long ch3 = ibus.readChannel(2);
  long ch4 = ibus.readChannel(3);
  if (ch3 < 900 || ch3 > 2100) ch3 = ch3_filtered;
  if (ch4 < 900 || ch4 > 2100) ch4 = ch4_filtered;

  ch3_filtered = (ch3_filtered * SERVO_FILTER_NUM + (int)ch3) / SERVO_FILTER_DEN;
  ch4_filtered = (ch4_filtered * SERVO_FILTER_NUM + (int)ch4) / SERVO_FILTER_DEN;
  if (abs(ch3_filtered - SERVO_CENTER) <= SERVO_DEADBAND) ch3_filtered = SERVO_CENTER;
  if (abs(ch4_filtered - SERVO_CENTER) <= SERVO_DEADBAND) ch4_filtered = SERVO_CENTER;

  // Direct mapping: CH3 -> Servo1, CH4 -> Servo2
  // CH3 up should move servo toward the opposite (counter-clockwise) side.
  int target_s1 = map((int)constrain(ch3_filtered, 1000, 2000), 1000, 2000, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
  int target_s2 = map((int)constrain(ch4_filtered, 1000, 2000), 1000, 2000, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
  int step1 = constrain(target_s1 - s1_angle, -SERVO_MAX_STEP, SERVO_MAX_STEP);
  int step2 = constrain(target_s2 - s2_angle, -SERVO_MAX_STEP, SERVO_MAX_STEP);
  s1_angle = constrain(s1_angle + step1, SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
  s2_angle = constrain(s2_angle + step2, SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
  s1.write(s1_angle);
  s2.write(s2_angle);

  pump_on = readCh6Stable();
  digitalWrite(WATER_PUMP, pump_on);

  long ch5 = ibus.readChannel(4);
  String target_blade = blade_status;
  if (ch5 >= CH5_UP_ON_THRESH) {
    target_blade = "DN";
  } else if (ch5 <= CH5_DN_ON_THRESH) {
    target_blade = "UP";
  } else if (ch5 >= CH5_CENTER_LOW && ch5 <= CH5_CENTER_HIGH) {
    target_blade = "STP";
  }

  if (target_blade != blade_candidate) {
    blade_candidate = target_blade;
    blade_candidate_since = millis();
  }

  if (blade_status != blade_candidate && (millis() - blade_candidate_since) >= CH5_DEBOUNCE_MS) {
    blade_status = blade_candidate;
  }

  if (blade_status == "UP") {
    digitalWrite(BLADE_RELAY_UP, HIGH);
    digitalWrite(BLADE_RELAY_DN, LOW);
  } else if (blade_status == "DN") {
    digitalWrite(BLADE_RELAY_UP, LOW);
    digitalWrite(BLADE_RELAY_DN, HIGH);
  } else {
    digitalWrite(BLADE_RELAY_UP, LOW);
    digitalWrite(BLADE_RELAY_DN, LOW);
  }
}

bool readCh6Stable() {
  long raw = ibus.readChannel(5);
  unsigned long now = millis();

  if (raw >= 900 && raw <= 2100) {
    ch6_last_valid = (int)raw;
  } else {
    raw = ch6_last_valid;
  }

  bool target = ch6_state;
  if (!ch6_state && raw >= CH6_ON_THRESH) {
    target = true;
  } else if (ch6_state && raw <= CH6_OFF_THRESH) {
    target = false;
  }

  if (target != ch6_candidate) {
    ch6_candidate = target;
    ch6_candidate_since = now;
  }

  if (ch6_state != ch6_candidate && (now - ch6_candidate_since) >= CH6_DEBOUNCE_MS) {
    ch6_state = ch6_candidate;
  }

  return ch6_state;
}

void processMotor(int motor, long val) {
  int speed = 0, dir = 0;
  if (val > 1540) { speed = map(val, 1540, 2000, 0, 255); dir = 1; }
  else if (val < 1460) { speed = map(val, 1460, 1000, 0, 255); dir = -1; }

  // Require a short consecutive command streak before applying motion.
  if (dir != 0 && speed > 0) {
    if (drive_cmd_confirm_count[motor] < DRIVE_CMD_CONFIRM_CYCLES) {
      drive_cmd_confirm_count[motor]++;
      dir = 0;
      speed = 0;
    }
  } else {
    drive_cmd_confirm_count[motor] = 0;
  }

  if (motor == 1) dbg_dir_l = dir;
  else dbg_dir_r = dir;
  driveHardware(motor, dir, speed);
}

void driveHardware(int motor, int dir, int speed) {
  int pwmPin = (motor == 1) ? L_PWM : R_PWM;
  int inaPin = (motor == 1) ? L_INA : R_INA;
  int inbPin = (motor == 1) ? L_INB : R_INB;
  int finalSpeed = constrain(speed, 0, 255);

#if HW_INVERT_L
  if (motor == 1) dir = -dir;
#endif
#if HW_INVERT_R
  if (motor == 2) dir = -dir;
#endif

  if (dir == 1) {
    digitalWrite(inaPin, HIGH); digitalWrite(inbPin, LOW);
    analogWrite(pwmPin, finalSpeed);
    if (motor == 1) { dbg_l_ina = 1; dbg_l_inb = 0; }
    else { dbg_r_ina = 1; dbg_r_inb = 0; }
  } 
  else if (dir == -1) {
    digitalWrite(inaPin, LOW); digitalWrite(inbPin, HIGH);
    analogWrite(pwmPin, finalSpeed);
    if (motor == 1) { dbg_l_ina = 0; dbg_l_inb = 1; }
    else { dbg_r_ina = 0; dbg_r_inb = 1; }
  } 
  else {
    // Active brake on neutral
    digitalWrite(inaPin, HIGH); digitalWrite(inbPin, HIGH);
    analogWrite(pwmPin, 0);
    if (motor == 1) { dbg_l_ina = 1; dbg_l_inb = 1; }
    else { dbg_r_ina = 1; dbg_r_inb = 1; }
  }

  if (motor == 1) current_pwmL = (dir == -1 ? -finalSpeed : dir == 0 ? 0 : finalSpeed);
  else current_pwmR = (dir == -1 ? -finalSpeed : dir == 0 ? 0 : finalSpeed);
}

void stopMotors() {
  driveHardware(1, 0, 0);
  driveHardware(2, 0, 0);
}

void pollJetsonSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inputString.trim();
      if (inputString.length() > 0) processJetsonCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
      if (inputString.length() > 80) inputString = "";
    }
  }
}

void processJetsonCommand(const String& cmd) {
  // Avoid serial flood: logging every incoming command can add visible control latency.
  if (ENABLE_DEBUG_LOG && millis() - lastRxRawLogTime > 200) {
    lastRxRawLogTime = millis();
    Serial.print("# RXRAW "); Serial.println(cmd);
  }

  if (cmd == "EMG") {
    // Latch emergency once. If EMG keeps arriving, do not keep forcing STOP
    // because it can override joystick MANUAL,<cmd> packets.
    if (!software_emergency) {
      software_emergency = true;
      web_emg_manual_cmd = "STOP";
      web_emg_pwm_active = false;
      web_emg_pwm_l = 0;
      web_emg_pwm_r = 0;
      web_manual_override_until = 0;
      stopMotors();
    } else {
      software_emergency = true;
    }
    return;
  }
  if (cmd == "RST") {
    software_emergency = false;
    web_emg_manual_cmd = "STOP";
    web_emg_pwm_active = false;
    web_emg_pwm_l = 0;
    web_emg_pwm_r = 0;
    web_manual_override_until = 0;
    stopMotors();
    return;
  }

  // Web emergency direct command: MANUAL,<left>,<right>
  if (cmd.startsWith("MANUAL,")) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      int pL = cmd.substring(c1 + 1, c2).toInt();
      int pR = cmd.substring(c2 + 1).toInt();
      pL = constrain(pL, -255, 255);
      pR = constrain(pR, -255, 255);
      software_emergency = true;
      web_emg_pwm_active = true;
      web_emg_pwm_l = pL;
      web_emg_pwm_r = pR;
      web_manual_override_until = millis() + WEB_MANUAL_OVERRIDE_HOLD_MS;
      web_emg_manual_cmd = "STOP";
      Serial.print("# RX "); Serial.println(cmd);
      driveJetsonPwm(pL, pR);
      return;
    }
    // Non-numeric MANUAL command is ignored in direct mode.
    return;
  }

  // Web emergency analog/differential command: PWM,<left>,<right>
  if (cmd.startsWith("PWM,")) {
    int c1 = cmd.indexOf(',');
    int c2 = cmd.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      int pL = cmd.substring(c1 + 1, c2).toInt();
      int pR = cmd.substring(c2 + 1).toInt();
      pL = constrain(pL, -255, 255);
      pR = constrain(pR, -255, 255);
      software_emergency = true;
      web_emg_pwm_active = true;
      web_emg_pwm_l = pL;
      web_emg_pwm_r = pR;
      web_manual_override_until = millis() + WEB_MANUAL_OVERRIDE_HOLD_MS;
      web_emg_manual_cmd = "STOP";
      driveJetsonPwm(pL, pR);
      return;
    }
  }

  if (software_emergency) return;

  // Accept PWM command only in AUTO/FOLLOW mode.
  int i = cmd.indexOf(',');
  if (i <= 0) return;
  if (currentMode == MODE_MANUAL) return;

  int pL = cmd.substring(0, i).toInt();
  int pR = cmd.substring(i + 1).toInt();
  driveJetsonPwm(pL, pR);
}

void applyWebEmergencyManual() {
  if (web_emg_manual_cmd == "FORWARD") {
    driveJetsonPwm(-255, 255);
  } else if (web_emg_manual_cmd == "BACK") {
    driveJetsonPwm(255, -255);
  } else if (web_emg_manual_cmd == "LEFT") {
    driveJetsonPwm(-255, -255);
  } else if (web_emg_manual_cmd == "RIGHT") {
    driveJetsonPwm(255, 255);
  } else {
    stopMotors();
  }
}

void handleEmergencyControl() {
  if (web_emg_pwm_active) {
    if (millis() < web_manual_override_until) {
      driveJetsonPwm(web_emg_pwm_l, web_emg_pwm_r);
      return;
    }
    web_emg_pwm_active = false;
    web_emg_pwm_l = 0;
    web_emg_pwm_r = 0;
  }

  // Priority 1: active web emergency command.
  // Keep command alive until explicit STOP/RESET so web latency/touch jitter won't drop motion.
  if (web_emg_manual_cmd != "STOP") {
    applyWebEmergencyManual();
    return;
  }

#if WEB_EMER_ALLOW_RC_FALLBACK
  // Priority 2: RC fallback (optional).
  handleManual();
#else
  // No fresh web command => stop. Prevent RC/manual loop from fighting web emergency.
  stopMotors();
#endif
}

void driveJetsonPwm(int pL, int pR) {
#if JETSON_INVERT_L
  pL = -pL;
#endif
#if JETSON_INVERT_R
  pR = -pR;
#endif
  driveHardware(1, (pL > 0) - (pL < 0), abs(pL));
  driveHardware(2, (pR > 0) - (pR < 0), abs(pR));
}

String decodeCanStatus(const unsigned char *buf) {
  if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0) {
    return "NORMAL";
  }
  return "WARNING / PROTECTION";
}

void initCanDebug() {
  if (!ENABLE_CAN_DEBUG) return;

  SPI.begin();
  pinMode(CAN_INT, INPUT);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println("# CAN INIT OK");
  } else {
    Serial.println("# CAN INIT FAIL");
  }
}

void pollCanDebug() {
  if (!ENABLE_CAN_DEBUG) return;
  if (digitalRead(CAN_INT) != LOW) return;

  unsigned long rxId = 0;
  unsigned char len = 0;
  unsigned char buf[8] = {0};
  CAN.readMsgBuf(&rxId, &len, buf);

  if (rxId == 0x355 && len >= 4) {
    can_soc = (int)(buf[0] | (buf[1] << 8));
    can_soh = (int)(buf[2] | (buf[3] << 8));
  }

  if (rxId == 0x356 && len >= 6) {
    int raw_voltage = (int)(buf[0] | (buf[1] << 8));
    int raw_current = (int)(buf[2] | (buf[3] << 8));
    int raw_temp = (int)(buf[4] | (buf[5] << 8));
    can_voltage = raw_voltage / 100.0f;
    can_current = raw_current / 10.0f;
    can_temperature = raw_temp / 10.0f;
  }

  if (rxId == 0x351 && len >= 4) {
    int raw_charge = (int)(buf[0] | (buf[1] << 8));
    int raw_discharge = (int)(buf[2] | (buf[3] << 8));
    can_charge_limit = raw_charge / 10.0f;
    can_discharge_limit = raw_discharge / 100.0f;
  }

  if (rxId == 0x359 && len >= 4) {
    can_status_text = decodeCanStatus(buf);
  }

  if (rxId == 0x356) {
    Serial.print("# CAN {");
    Serial.print("\"soc\":"); Serial.print(can_soc); Serial.print(",");
    Serial.print("\"soh\":"); Serial.print(can_soh); Serial.print(",");
    Serial.print("\"voltage\":"); Serial.print(can_voltage); Serial.print(",");
    Serial.print("\"current\":"); Serial.print(can_current); Serial.print(",");
    Serial.print("\"temperature\":"); Serial.print(can_temperature); Serial.print(",");
    Serial.print("\"charge_limit\":"); Serial.print(can_charge_limit); Serial.print(",");
    Serial.print("\"discharge_limit\":"); Serial.print(can_discharge_limit); Serial.print(",");
    Serial.print("\"status\":\""); Serial.print(can_status_text); Serial.print("\"");
    Serial.println("}");
  }
}
