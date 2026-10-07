/*
 * AquaFlare - solar-powered fire detection, suppression and irrigation plot
 * Board: Arduino UNO R3 (ATmega328P)   IDE: Arduino IDE 2.x   Library: Servo
 *
 * Pin map
 *   A0  tank water-level sensor      D2, D4  L298N IN1, IN2 (fire pump)
 *   A1  flame sensor A (front half)  D5      L298N ENA (fire pump)
 *   A2  flame sensor B (rear half)   D7, D8  L298N IN3, IN4 (irrigation pump)
 *   A3  soil-moisture sensor 1       D6      L298N ENB (irrigation pump)
 *   A4  soil-moisture sensor 2       D9      scanner servo A (goBILDA Speed)
 *                                    D10     scanner servo B (goBILDA Torque)
 *                                    D11     hose rotation servo (Axon MAX)
 *                                    D3      hose tilt servo (Axon MAX)
 */
#include <Servo.h>

// ---------------- Pins ----------------
const uint8_t PIN_LEVEL = A0, PIN_FLAME_A = A1, PIN_FLAME_B = A2;
const uint8_t PIN_SOIL_1 = A3, PIN_SOIL_2 = A4;
const uint8_t PIN_FIRE_IN1 = 2, PIN_FIRE_IN2 = 4, PIN_FIRE_EN = 5;
const uint8_t PIN_IRR_IN3 = 7, PIN_IRR_IN4 = 8, PIN_IRR_EN = 6;
const uint8_t PIN_SCAN_A = 9, PIN_SCAN_B = 10, PIN_PAN = 11, PIN_TILT = 3;

// ---------------- Calibration (set on the prototype) ----------------
const int FLAME_ON  = 400;  // ADC below: infrared source present
const int FLAME_OFF = 550;  // ADC above: source gone (hysteresis)
const int SOIL_DRY  = 620;  // ADC above: soil too dry, start watering
const int SOIL_WET  = 480;  // ADC below: soil wet enough, stop
const int LEVEL_MIN = 250;  // ADC below: tank empty, pumps locked
const int LEVEL_OK  = 320;  // ADC above: tank refilled, lock released

const float US_PER_DEG_SCAN = 6.67;  // goBILDA: 2000 us over 300 deg
const float US_PER_DEG_HOSE = 5.71;  // Axon MAX: 2000 us over 350 deg
const int SCAN_STEP = 2;             // each scanner sweeps 0..180 deg
const int PAN_MIN = 5, PAN_MAX = 345;      // usable hose rotation
const int TILT_NEAR = 60, TILT_FAR = 100;  // hose tilt sweep limits

// ---------------- Timing (ms) ----------------
const unsigned long T_SCAN_STEP = 15, T_CONFIRM = 400, T_AIM = 500;
const unsigned long T_FIRE_MAX = 10000, T_FLAME_GONE = 1500;
const unsigned long T_TILT_STEP = 20, T_IRR_BURST = 3000;
const unsigned long T_IRR_SOAK = 30000, T_TELEMETRY = 1000;

enum State : uint8_t { MONITOR, CONFIRM, AIM, EXTINGUISH, TANK_EMPTY };
const char* const STATE_NAME[] =
  {"MONITOR", "CONFIRM", "AIM", "EXTINGUISH", "TANK_EMPTY"};

Servo scanA, scanB, hosePan, hoseTilt;
State state = MONITOR;
unsigned long tState = 0, tStep = 0, tGone = 0, tIrr = 0, tTele = 0;
unsigned long tTilt = 0;
int scanAngle = 0, scanDir = SCAN_STEP;
int best = 1023, bestAngle = 90;   // strongest reading of the sweep
bool bestIsB = false;              // which sensor produced it
int targetBearing = -1;            // 0..359 deg around the mast, -1 = none
uint8_t watchPin = PIN_FLAME_A;    // sensor that keeps watching the flame
int tilt = TILT_NEAR, tiltDir = 1;
bool irrigating = false, soilDry = false;

// 90 deg = 1500 us. Servo::write() is avoided: it assumes a 180 deg servo.
void writeDeg(Servo& s, float deg, float usPerDeg) {
  int us = (int)(1500 + (deg - 90.0) * usPerDeg);
  s.writeMicroseconds(constrain(us, 500, 2500));
}

// 4-sample average, about 0.45 ms on the ATmega328P
int readAvg(uint8_t pin) {
  int sum = 0;
  for (uint8_t i = 0; i < 4; i++) sum += analogRead(pin);
  return sum / 4;
}

void firePump(bool on) {
  digitalWrite(PIN_FIRE_IN1, on);
  analogWrite(PIN_FIRE_EN, on ? 255 : 0);
}

void irrPump(bool on) {
  digitalWrite(PIN_IRR_IN3, on);
  analogWrite(PIN_IRR_EN, on ? 255 : 0);
  irrigating = on;
}

void enter(State s) { state = s; tState = millis(); }

// One scanner step. Returns true when a full sweep has just finished.
// Sensor A looks at bearings 0..180, sensor B (back to back) at 180..360.
bool scanStep() {
  if (millis() - tStep < T_SCAN_STEP) return false;
  tStep = millis();
  int a = readAvg(PIN_FLAME_A), b = readAvg(PIN_FLAME_B);
  if (a < best) { best = a; bestAngle = scanAngle; bestIsB = false; }
  if (b < best) { best = b; bestAngle = scanAngle; bestIsB = true; }
  scanAngle += scanDir;
  bool done = (scanAngle >= 180 || scanAngle <= 0);
  if (done) {
    scanDir = -scanDir;
    targetBearing = -1;
    if (best < FLAME_ON) {
      targetBearing = (bestIsB ? 180 + bestAngle : bestAngle) % 360;
      watchPin = bestIsB ? PIN_FLAME_B : PIN_FLAME_A;
    }
    best = 1023;
  }
  writeDeg(scanA, scanAngle, US_PER_DEG_SCAN);
  writeDeg(scanB, scanAngle, US_PER_DEG_SCAN);
  return done;
}

// Hose rotation: 0..350 deg of servo travel mapped on 500..2500 us.
void aimHose(int bearing) {
  int pan = constrain(bearing, PAN_MIN, PAN_MAX);
  hosePan.writeMicroseconds(500 + (int)(pan * US_PER_DEG_HOSE));
}

// While the pump runs, the tilt servo sweeps the jet from near to far.
void sweepTilt() {
  if (millis() - tTilt < T_TILT_STEP) return;
  tTilt = millis();
  tilt += tiltDir;
  if (tilt >= TILT_FAR || tilt <= TILT_NEAR) tiltDir = -tiltDir;
  writeDeg(hoseTilt, tilt, US_PER_DEG_HOSE);
}

void irrigation() {
  int soil = (readAvg(PIN_SOIL_1) + readAvg(PIN_SOIL_2)) / 2;
  if (soil > SOIL_DRY) soilDry = true;
  if (soil < SOIL_WET) soilDry = false;
  if (irrigating) {
    if (millis() - tIrr >= T_IRR_BURST || !soilDry) {
      irrPump(false);
      tIrr = millis();
    }
  } else if (soilDry && (tIrr == 0 || millis() - tIrr >= T_IRR_SOAK)) {
    irrPump(true);
    tIrr = millis();
  }
}

void telemetry() {
  if (millis() - tTele < T_TELEMETRY) return;
  tTele = millis();
  Serial.print(STATE_NAME[state]);        Serial.print(',');
  Serial.print(analogRead(PIN_LEVEL));    Serial.print(',');
  Serial.print(analogRead(PIN_FLAME_A));  Serial.print(',');
  Serial.print(analogRead(PIN_FLAME_B));  Serial.print(',');
  Serial.print(analogRead(PIN_SOIL_1));   Serial.print(',');
  Serial.print(analogRead(PIN_SOIL_2));   Serial.print(',');
  Serial.println(targetBearing);
}

void setup() {
  const uint8_t outs[] = {PIN_FIRE_IN1, PIN_FIRE_IN2, PIN_FIRE_EN,
                          PIN_IRR_IN3, PIN_IRR_IN4, PIN_IRR_EN};
  for (uint8_t i = 0; i < sizeof(outs); i++) {
    pinMode(outs[i], OUTPUT);
    digitalWrite(outs[i], LOW);
  }
  scanA.attach(PIN_SCAN_A, 500, 2500);
  scanB.attach(PIN_SCAN_B, 500, 2500);
  hosePan.attach(PIN_PAN, 500, 2500);
  hoseTilt.attach(PIN_TILT, 500, 2500);
  aimHose(180);
  writeDeg(hoseTilt, TILT_NEAR, US_PER_DEG_HOSE);
  Serial.begin(9600);
  Serial.println(F("state,level,flameA,flameB,soil1,soil2,bearing"));
}

void loop() {
  int level = readAvg(PIN_LEVEL);

  // Dry-run protection has priority over every other state.
  if (level < LEVEL_MIN && state != TANK_EMPTY) {
    firePump(false);
    irrPump(false);
    enter(TANK_EMPTY);
  }

  switch (state) {
    case MONITOR:
      irrigation();
      if (scanStep() && targetBearing >= 0) {
        irrPump(false);                 // fire has priority over watering
        // point the scanner that saw the flame back at it and look again
        Servo& sc = (watchPin == PIN_FLAME_B) ? scanB : scanA;
        writeDeg(sc, targetBearing % 180, US_PER_DEG_SCAN);
        enter(CONFIRM);
      }
      break;

    case CONFIRM:                       // still there after 400 ms?
      if (millis() - tState >= T_CONFIRM) {
        if (readAvg(watchPin) < FLAME_OFF) {
          aimHose(targetBearing);
          tilt = TILT_NEAR;
          tiltDir = 1;
          writeDeg(hoseTilt, tilt, US_PER_DEG_HOSE);
          enter(AIM);
        } else {
          enter(MONITOR);               // false alarm (reflection, sun)
        }
      }
      break;

    case AIM:                           // let the hose servos settle
      if (millis() - tState >= T_AIM) {
        firePump(true);
        tGone = 0;
        enter(EXTINGUISH);
      }
      break;

    case EXTINGUISH: {
      sweepTilt();
      if (readAvg(watchPin) < FLAME_OFF) tGone = 0;
      else if (tGone == 0) tGone = millis();
      bool out = (tGone != 0 && millis() - tGone >= T_FLAME_GONE);
      bool timeout = (millis() - tState >= T_FIRE_MAX);
      if (out || timeout) {             // stop, then rescan to verify
        firePump(false);
        best = 1023;
        enter(MONITOR);
      }
      break;
    }

    case TANK_EMPTY:                    // pumps stay locked until refilled
      scanStep();
      if (level > LEVEL_OK) enter(MONITOR);
      break;
  }
  telemetry();
}
