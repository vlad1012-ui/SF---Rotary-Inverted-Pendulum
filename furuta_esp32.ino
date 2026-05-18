/*
  furuta_esp32(3).ino — same firmware as furuta_esp32/furuta_esp32.ino
  ====================================================================
  If you edit behavior, update BOTH copies or use one folder sketch only.

  Furuta Pendulum — ESP32 Angle Streamer v3
  Board  : DOIT ESP32 DEVKIT V1
  Sensor : AS5048A in PWM mode → GPIO34
  Baud   : 115200

  Angle convention (after ZERO at upright):
     180°  = straight UP   ← balance / inverted target
       0°  = straight DOWN  (hanging)

  Commands:
    ZERO   → hold pendulum UPRIGHT → raw offset; stream reads ~180° at this pose
    RAW    → one raw reading (0–360°, debug)
    STATUS → calibration state + offset
    CLEAR  → erase saved calibration (flash + RAM)

  NVS: offset survives USB/Serial resets so you are not stuck on A:RAW:153
  until you ZERO again (unless you CLEAR or reflash with erase).
*/

#include <Preferences.h>

#define PWM_PIN 34

static Preferences s_prefs;
static const char *kNs = "furuta";

float  g_zero_offset = 0.0;
bool   g_calibrated  = false;
String serialBuffer  = "";

static void saveCalibration() {
  s_prefs.begin(kNs, false);
  s_prefs.putFloat("zoff", g_zero_offset);
  s_prefs.putBool("cal", true);
  s_prefs.end();
}

static void loadCalibration() {
  s_prefs.begin(kNs, true);
  if (s_prefs.getBool("cal", false)) {
    g_zero_offset = s_prefs.getFloat("zoff", 0.0f);
    g_calibrated  = true;
    Serial.print("Loaded calibration from flash, offset=");
    Serial.println(g_zero_offset, 2);
  }
  s_prefs.end();
}

static void clearCalibrationNvs() {
  s_prefs.begin(kNs, false);
  s_prefs.clear();
  s_prefs.end();
  g_zero_offset = 0.0;
  g_calibrated  = false;
}

float readRawAngle() {
  unsigned long t_high = pulseIn(PWM_PIN, HIGH, 10000);
  unsigned long t_low  = pulseIn(PWM_PIN, LOW,  10000);

  if (t_high == 0 || t_low == 0) return -1.0;

  float period = (float)(t_high + t_low);
  float angle  = ((float)t_high / period) * 360.0;

  if (angle < 0.0)   angle = 0.0;
  if (angle >= 360.0) angle = 359.99;

  return angle;
}

float getCalibratedAngle(float raw) {
  float angle = fmod((raw - g_zero_offset + 540.0), 360.0) - 180.0;
  angle += 180.0;
  if (angle > 180.0) angle -= 360.0;
  if (angle <= -180.0) angle += 360.0;
  return angle;
}

void parseSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuffer.trim();

      if (serialBuffer == "ZERO") {
        float raw = readRawAngle();
        if (raw < 0) {
          Serial.println("ZERO:ERR sensor timeout");
        } else {
          g_zero_offset = raw;
          g_calibrated  = true;
          saveCalibration();
          Serial.print("ZERO:OK offset=");
          Serial.println(g_zero_offset, 2);
          float check = getCalibratedAngle(readRawAngle());
          Serial.print("ZERO:CHECK angle=");
          Serial.println(check, 2);
        }

      } else if (serialBuffer == "RAW") {
        float raw = readRawAngle();
        Serial.print("RAW:");
        Serial.println(raw < 0 ? -1 : raw, 2);

      } else if (serialBuffer == "STATUS") {
        Serial.print("STATUS:calibrated=");
        Serial.print(g_calibrated ? "yes" : "no");
        Serial.print(" offset=");
        Serial.println(g_zero_offset, 2);

      } else if (serialBuffer == "CLEAR") {
        clearCalibrationNvs();
        Serial.println("CLEAR:OK - send ZERO again (pendulum upright).");
      }

      serialBuffer = "";
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 20) serialBuffer = "";
    }
  }
}

void setup() {
  pinMode(PWM_PIN, INPUT);
  Serial.begin(115200);
  delay(200);
  loadCalibration();
  Serial.println("ESP32 ready (v3 + NVS cal).");
  if (!g_calibrated) {
    Serial.println("Hold pendulum UPRIGHT then send: ZERO");
    Serial.println("Send RAW to verify full 0-360 sweep before calibrating.");
  }
}

void loop() {
  parseSerial();

  float raw = readRawAngle();

  if (raw < 0) {
    Serial.println("A:ERR");
    return;
  }

  if (g_calibrated) {
    Serial.print("A:");
    Serial.println(getCalibratedAngle(raw), 2);
  } else {
    Serial.print("A:RAW:");
    Serial.println(raw, 2);
  }
}