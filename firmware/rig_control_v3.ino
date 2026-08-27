/*
  ==========================================================================
  ACTIVE QUARTER-CAR RIG — CONTROL FIRMWARE v3
  ==========================================================================
  Integrated skyhook controller: MPU6500 IMU + VL53L1X ToF + MG90S servo.

  SENSOR ROLES (per design decision Aug 2026):
    - ToF (VL53L1X):  POSITION source (suspension travel) and, via filtered
                      differentiation, VELOCITY source for the control law.
    - IMU (MPU6500):  ACCELERATION source (body vertical accel) — logged in
                      telemetry, available for analysis/digital twin, not
                      integrated (avoids drift problem entirely).

  HONEST PHYSICS NOTE: textbook skyhook wants ABSOLUTE body velocity z1_dot.
  The ToF measures RELATIVE distance (crossbar to whatever it faces), so its
  derivative is relative suspension velocity. Damping on relative velocity is
  closer to "adjustable semi-active damper" than pure skyhook — still a real,
  demonstrable active control effect, and the right pragmatic choice here
  since it's drift-free. When the rig is assembled, compare both: this mode
  vs. IMU-integrated velocity, and see which behaves better in practice.

  CONTROL LOOP: 100 Hz (10 ms period).

  SERIAL COMMANDS (115200 baud, line ending = Newline):
    SKY:1      enable skyhook control
    SKY:0      disable (servo returns to center — passive mode)
    GAIN:8.5   set c_sky live (any positive float)
    PULSE      fire a brief servo kick as a manual disturbance
    CENTER     force servo to center, disables control
    STAT       print current settings and sensor snapshot
    HEALTH     run sensor/servo health check, machine-readable H: lines
    STREAM:1   turn on telemetry streaming (default ON)
    STREAM:0   quiet mode — only responses to commands

  TELEMETRY (streamed at 20 Hz, tab-separated, one line per sample):
    t_ms  pos_mm  vel_mms  accel_g  u_cmd  servo_deg  sky_on
  ==========================================================================
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <ESP32Servo.h>

// ------------------------- PINS -------------------------
#define SDA_PIN     21
#define SCL_PIN     22
#define SERVO_PIN   18
#define MPU_ADDR    0x68

// ---------------- PHYSICAL PARAMETERS (EDIT ME) ----------------
// Placeholder values from the locked design — change as measured.
float M1_KG        = 0.35;    // sprung (body) mass, kg  [current est. 310-385g]
float K1_NPM       = 172.0;   // main suspension stiffness, N/m (twins in series)
float K_SERVO_NPM  = 196.0;   // servo series spring stiffness, N/m

// Skyhook gain c_sky (N·s/m). Reasonable starting point: a fraction of
// critical damping c_crit = 2*sqrt(k1*m1) ≈ 2*sqrt(172*0.35) ≈ 15.5 N·s/m.
// Start ~30% of critical; tune live with GAIN:x.
float C_SKY        = 4.5;

// ---------------- SERVO GEOMETRY & LIMITS ----------------
// 10 mm arm, 2 cm total mechanical travel available (±10 mm).
// Arm horizontal throw: dx = ARM_MM * sin(theta). ±90° would use the full
// ±10 mm, but near ±90° the linkage geometry degrades (arm nearly vertical,
// tiny leverage, binding risk). Cap at ±60° -> ±8.7 mm, safe and effective.
const float ARM_MM          = 10.0;
const int   SERVO_CENTER    = 90;
const int   SERVO_MAX_DELTA = 60;     // degrees away from center, each way

// ---------------- LOOP TIMING ----------------
const uint32_t CONTROL_PERIOD_US   = 10000;  // 100 Hz control loop
const uint32_t TELEMETRY_PERIOD_MS = 50;     // 20 Hz telemetry stream

// ---------------- FILTERING ----------------
// ToF-derived velocity is a differentiated noisy signal — must be smoothed.
// Single-pole low-pass on the raw derivative. ALPHA closer to 1 = smoother
// but laggier. At 100 Hz, 0.15 gives ~ 2-3 Hz of usable bandwidth: enough
// for a 3.4 Hz suspension mode while killing mm-quantization chatter.
const float VEL_LPF_ALPHA = 0.15;
// Light smoothing on position too (ToF is mm-quantized).
const float POS_LPF_ALPHA = 0.35;

// ==========================================================================
VL53L1X tof;
Servo servo;

bool  skyOn        = false;
bool  streamOn     = true;
float posMM        = 0;      // filtered suspension position (ToF), mm
float posRawPrev   = 0;
float velMMS       = 0;      // filtered suspension velocity, mm/s
float accelG       = 0;      // body vertical accel from IMU, g
float uCmd         = 0;      // control force command, N
int   servoDeg     = SERVO_CENTER;
float posRefMM     = 0;      // reference (rest) position captured at boot

uint32_t lastControlUS   = 0;
uint32_t lastTelemetryMS = 0;
uint32_t pulseUntilMS    = 0;

// --------------------------------------------------------------------------
// IMU: direct register access (works on MPU6050 and MPU6500 identically)
// --------------------------------------------------------------------------
void imuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // wake from sleep
  Wire.endTransmission();
  delay(50);
}

// Returns Z-axis acceleration in g. If the IMU is mounted with a different
// axis vertical on the body plate, change which pair of bytes is used below.
float imuReadAccelZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3F);  // ACCEL_ZOUT_H  (X=0x3B, Y=0x3D if remapping needed)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  if (Wire.available() < 2) return accelG;  // keep last value on read hiccup
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 16384.0;  // ±2g default range
}

// --------------------------------------------------------------------------
// Servo mapping: force command -> series-spring deflection -> arm angle
// --------------------------------------------------------------------------
// u [N] pushed through the series spring requires deflection dx = u / k_servo
// (in meters -> convert to mm). Arm angle from dx: theta = asin(dx / ARM_MM).
void servoApplyForce(float u_newtons) {
  float dx_mm = (u_newtons / K_SERVO_NPM) * 1000.0;

  // Clamp to reachable throw before asin() (avoids NaN past ±ARM_MM)
  float maxThrow = ARM_MM * sin(radians(SERVO_MAX_DELTA));
  dx_mm = constrain(dx_mm, -maxThrow, maxThrow);

  float deltaDeg = degrees(asin(dx_mm / ARM_MM));
  servoDeg = SERVO_CENTER + (int)round(deltaDeg);
  servoDeg = constrain(servoDeg, SERVO_CENTER - SERVO_MAX_DELTA,
                                 SERVO_CENTER + SERVO_MAX_DELTA);
  servo.write(servoDeg);
}

void servoCenter() {
  servoDeg = SERVO_CENTER;
  servo.write(SERVO_CENTER);
}

// --------------------------------------------------------------------------
// Health check — machine-readable output for the dashboard (H: prefix)
// --------------------------------------------------------------------------
bool i2cPresent(byte addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void healthCheck() {
  // IMU: on bus? identity? live reading?
  if (i2cPresent(MPU_ADDR)) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);  // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 1);
    byte who = Wire.available() ? Wire.read() : 0;
    float az = imuReadAccelZ();
    Serial.print("H:IMU:OK:0x");
    Serial.print(who, HEX);
    Serial.print(":");
    Serial.println(az, 3);
  } else {
    Serial.println("H:IMU:FAIL:notfound:0");
  }

  // ToF: on bus? fresh reading?
  if (i2cPresent(0x29)) {
    uint16_t mm = tof.read();  // blocking read of latest measurement
    if (tof.timeoutOccurred()) {
      Serial.println("H:TOF:WARN:timeout:0");
    } else {
      Serial.print("H:TOF:OK:reading:");
      Serial.println(mm);
    }
  } else {
    Serial.println("H:TOF:FAIL:notfound:0");
  }

  // Servo: brief wiggle so it's visually/audibly verifiable, then restore
  int before = servoDeg;
  servo.write(before + 12); delay(180);
  servo.write(before - 12); delay(180);
  servo.write(before);      delay(120);
  servoDeg = before;
  Serial.print("H:SERVO:OK:wiggled:");
  Serial.println(before);

  Serial.println("H:DONE");
}

// --------------------------------------------------------------------------
// Serial command handling
// --------------------------------------------------------------------------
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "SKY:1") {
    skyOn = true;
    Serial.println("# skyhook ON");
  } else if (cmd == "SKY:0") {
    skyOn = false;
    servoCenter();
    Serial.println("# skyhook OFF, servo centered (passive mode)");
  } else if (cmd.startsWith("GAIN:")) {
    float g = cmd.substring(5).toFloat();
    if (g > 0) {
      C_SKY = g;
      Serial.print("# c_sky = ");
      Serial.println(C_SKY);
    } else {
      Serial.println("# GAIN must be a positive number, e.g. GAIN:4.5");
    }
  } else if (cmd == "PULSE") {
    pulseUntilMS = millis() + 150;   // 150 ms kick
    Serial.println("# pulse fired");
  } else if (cmd == "CENTER") {
    skyOn = false;
    servoCenter();
    Serial.println("# centered, control off");
  } else if (cmd == "STAT") {
    Serial.println("# --- STATUS ---");
    Serial.print("#  skyhook: "); Serial.println(skyOn ? "ON" : "OFF");
    Serial.print("#  c_sky:   "); Serial.println(C_SKY);
    Serial.print("#  m1(kg):  "); Serial.println(M1_KG);
    Serial.print("#  k1(N/m): "); Serial.println(K1_NPM);
    Serial.print("#  pos(mm): "); Serial.println(posMM, 1);
    Serial.print("#  vel(mm/s): "); Serial.println(velMMS, 1);
    Serial.print("#  accel(g):  "); Serial.println(accelG, 3);
    Serial.print("#  servo(deg): "); Serial.println(servoDeg);
  } else if (cmd == "HEALTH") {
    Serial.println("# running health check...");
    healthCheck();
  } else if (cmd == "STREAM:1") {
    streamOn = true;
    Serial.println("# telemetry ON");
  } else if (cmd == "STREAM:0") {
    streamOn = false;
    Serial.println("# telemetry OFF");
  } else if (cmd.length() > 0) {
    Serial.println("# commands: SKY:1 SKY:0 GAIN:x PULSE CENTER STAT STREAM:1 STREAM:0");
  }
}

// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("# ===== QUARTER-CAR RIG CONTROL FIRMWARE v3 =====");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // fast-mode I2C: both sensors support it, halves bus time

  imuInit();
  Serial.println("# IMU awake");

  tof.setTimeout(100);
  if (!tof.init()) {
    Serial.println("# TOF INIT FAILED — check wiring. Halting.");
    while (1) delay(1000);
  }
  tof.setDistanceMode(VL53L1X::Short);        // short mode: better precision
  tof.setMeasurementTimingBudget(20000);      // 20 ms budget -> 50 Hz capable
  tof.startContinuous(20);
  Serial.println("# ToF running continuous @ 50 Hz");

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servoCenter();
  Serial.println("# servo centered");

  // Capture rest position as reference — rig should be still at power-on
  delay(300);
  long sum = 0;
  int  n   = 0;
  for (int i = 0; i < 10; i++) {
    uint16_t mm = tof.read();
    if (!tof.timeoutOccurred()) { sum += mm; n++; }
    delay(25);
  }
  posRefMM = (n > 0) ? (float)sum / n : 0;
  posMM = 0;
  posRawPrev = 0;
  Serial.print("# rest reference captured: ");
  Serial.print(posRefMM, 1);
  Serial.println(" mm");
  Serial.println("# type STAT for status, SKY:1 to enable control");
  Serial.println("# t_ms\tpos_mm\tvel_mms\taccel_g\tu_cmd\tservo_deg\tsky");

  lastControlUS = micros();
}

// --------------------------------------------------------------------------
void loop() {
  // ---- serial commands, any time ----
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // ---- fixed-rate control loop, 100 Hz ----
  uint32_t nowUS = micros();
  if (nowUS - lastControlUS >= CONTROL_PERIOD_US) {
    float dt = (nowUS - lastControlUS) / 1e6;  // actual elapsed, seconds
    lastControlUS = nowUS;

    // --- position from ToF (non-blocking check: data ready ~every 20 ms) ---
    if (tof.dataReady()) {
      uint16_t raw = tof.read(false);          // non-blocking read
      if (!tof.timeoutOccurred() && raw > 0) {
        float posRaw = (float)raw - posRefMM;  // mm, zero at rest

        // filtered position
        posMM = POS_LPF_ALPHA * posRaw + (1 - POS_LPF_ALPHA) * posMM;

        // velocity = filtered derivative of raw position
        // (ToF updates every ~20ms; only differentiate on fresh samples)
        float velRaw = (posRaw - posRawPrev) / 0.020;  // mm/s at 50Hz ToF rate
        posRawPrev = posRaw;
        velMMS = VEL_LPF_ALPHA * velRaw + (1 - VEL_LPF_ALPHA) * velMMS;
      }
    }

    // --- acceleration from IMU (every control tick, it's fast) ---
    accelG = imuReadAccelZ();

    // --- control law ---
    if (millis() < pulseUntilMS) {
      // manual disturbance overrides control briefly
      servo.write(SERVO_CENTER + 40);
      servoDeg = SERVO_CENTER + 40;
      uCmd = 0;
    } else if (skyOn) {
      // skyhook on ToF-derived velocity: u = -c_sky * v
      // velMMS is mm/s -> convert to m/s for SI force
      uCmd = -C_SKY * (velMMS / 1000.0);
      servoApplyForce(uCmd);
    } else {
      uCmd = 0;
      if (servoDeg != SERVO_CENTER) servoCenter();
    }
  }

  // ---- telemetry stream, 20 Hz ----
  uint32_t nowMS = millis();
  if (streamOn && nowMS - lastTelemetryMS >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMS = nowMS;
    Serial.print(nowMS);        Serial.print('\t');
    Serial.print(posMM, 1);     Serial.print('\t');
    Serial.print(velMMS, 1);    Serial.print('\t');
    Serial.print(accelG, 3);    Serial.print('\t');
    Serial.print(uCmd, 3);      Serial.print('\t');
    Serial.print(servoDeg);     Serial.print('\t');
    Serial.println(skyOn ? 1 : 0);
  }
}
