/*
 * AquaFlare - battery-powered (solar-charged) fire detection, suppression and irrigation plot
 * Board: Arduino UNO R3 (ATmega328P)   IDE: Arduino IDE 2.x   Library: Servo
 * RoboChallenge 2026 - Freestyle Showcase High School - Team RaSky
 *
 * Pin map
 *   A0  tank water-level sensor      D2, D4  L298N IN1, IN2 (irrigation pump)
 *   A1  flame sensor A               D5      L298N ENA (irrigation pump)
 *   A2  flame sensor B               D7, D8  L298N IN3, IN4 (fire pump)
 *   A3  soil-moisture sensor 1       D6      L298N ENB (fire pump)
 *   A4  soil-moisture sensor 2       D9      scanner servo (goBILDA, 300 deg),
 *   A5  free (no encoder). Used ONLY  both flame sensors back to back
 *       if USE_END_SWITCHES = true   D11     hose rotation, goBILDA 1 (continuous)
 *                                    D10     hose rotation, goBILDA 2 (continuous)
 *                                    D3      hose tilt servo (goBILDA)
 *                                    D12     free; used ONLY if
 *                                            USE_END_SWITCHES = true
 * No encoder is used: the hose position is estimated from time.
 *
 * Hose rotation: two goBILDA servos in CONTINUOUS mode, each winding a
 * string tied to the hose, pulling in opposite directions.
 *   - direction +1: servo 1 winds (pulls), servo 2 unwinds (pays out)
 *   - direction -1: servo 2 winds (pulls), servo 1 unwinds (pays out)
 * The pulling servo always runs at HOSE_SPEED; the other one unwinds at
 * PAYOUT_x times that speed, so the strings never fight each other.
 * String 1 is tight, string 2 is looser. At power-on servo 2 winds for
 * SLACK_MS_2 with servo 1 standing still: string 1 holds the hose, so only
 * the slack of string 2 is taken up and both strings end up tight. After
 * that the program keeps count of any slack (in deg) and, when a string
 * has slack, its servo first winds it tight before the hose moves.
 * There is no position sensor: the position comes from how long the hose
 * has really moved, with a separate travel time for each direction.
 * HOSE_RANGE (230) is the travel chosen so that the hose never reaches the
 * components behind it. Two optional end switches (A5 and D12) stop the
 * hose at the ends and correct its position, so even a wrong estimate can
 * never push it into those components; without switches the hose keeps a
 * safety margin from both ends.
 * The strings are tied to points on the hose (not wound on a drum), so the
 * hose angle does not grow evenly with the string wound in. The directions
 * are therefore calibrated at several points (CAL_BEARING / CAL_POS) and
 * the program interpolates between them.
 *
 * Flame scanner: one goBILDA servo sweeps the two sensors 230 deg left-right
 * (115 deg each side of its centre), so each sensor covers 230 deg and
 * together they cover the full circle with two 50 deg overlap zones.
 * After power-on the scanner sweeps a few times and learns the "landscape":
 * the normal reading of each sensor at every angle. A flame is a reading
 * clearly LOWER than what that sensor normally sees at that angle. Start the
 * system with no flame in front of the sensors. The landscape keeps
 * updating slowly (daylight).
 */
#include <Servo.h>

// ---------------- Pins ----------------
const uint8_t PIN_LEVEL = A0, PIN_FLAME_A = A1, PIN_FLAME_B = A2;
const uint8_t PIN_SOIL_1 = A3, PIN_SOIL_2 = A4, PIN_HOME = A5, PIN_END = 12;
const uint8_t PIN_IRR_IN1 = 2, PIN_IRR_IN2 = 4, PIN_IRR_EN = 5;    // OUT1/OUT2
const uint8_t PIN_FIRE_IN3 = 7, PIN_FIRE_IN4 = 8, PIN_FIRE_EN = 6; // OUT3/OUT4
const uint8_t PIN_SCAN = 9;            // one servo, both flame sensors
const uint8_t PIN_ROT_1 = 11, PIN_ROT_2 = 10, PIN_TILT = 3;

// ---------------- Calibration (set on the prototype) ----------------
const int FLAME_DELTA = 120; // drop below the landscape that means a flame
const int FLAME_KEEP  = 60;  // drop that still counts as "flame still there"
const uint8_t LEARN_SWEEPS = 4;  // sweeps used to learn the landscape (~7 s)
// Soil thresholds: look at soil1/soil2 in the Serial Monitor with the probes
// in dry soil and in wet soil, then put SOIL_DRY near the dry value and
// SOIL_WET near the wet value.
// Our probes read LOW in dry soil (about 130-220), so SOIL_HIGH_IS_DRY = false:
// watering starts below SOIL_DRY and stops above SOIL_WET.
const int SOIL_DRY  = 300;  // dry soil reads past this: start watering
const int SOIL_WET  = 450;  // wet soil reads past this: stop
const bool SOIL_HIGH_IS_DRY = false; // true if your sensor reads HIGHER when dry
const int LEVEL_MIN = 250;  // ADC below: tank empty, pumps locked
const int LEVEL_OK  = 320;  // ADC above: tank refilled, lock released
const bool LEVEL_CHECK = true; // false ONLY for a bench test without water

// The L298N is fed on +12V from the 7.4 V servo battery (8.4 V when full).
// Its outputs are NOT regulated: about 1.8 V is lost in the driver, so a
// pump would get up to ~6.6 V. The pumps are rated 3-6 V, so the enable
// pins use PWM: 190 of 255 gives an average of about 4.2-5 V.
const int PUMP_PWM = 190;   // 0..255; lower = weaker jet, higher = stronger

// ---- Flame scanner: goBILDA, angles 0..300 of its own travel ----
const float US_PER_DEG_GB = 2000.0 / 300.0;  // 500 us = 0 deg, 2500 us = 300
const int SCAN_CENTER = 150;                 // middle of travel (1500 us)
const int SCAN_RANGE  = 230;                 // total left-right sweep
const int SCAN_MIN = SCAN_CENTER - SCAN_RANGE / 2;   // 35 deg
const int SCAN_MAX = SCAN_CENTER + SCAN_RANGE / 2;   // 265 deg
const int SCAN_STEP = 2;
const float SCAN_MS_PER_DEG = 3.4;           // 0.20 s / 60 deg at 6 V
// Bearing of sensor A when the scanner is in the middle (sensor B = +180).
// SCAN_SIGN = -1 if the bearing printed for a flame moves the wrong way.
const int BEARING_A_CENTER = 90;
const int SCAN_SIGN = 1;

const int TILT_NEAR = 60, TILT_FAR = 100;  // hose tilt sweep limits

// ---- Hose rotation: two goBILDA servos in continuous mode ----
// Position of the hose: 0 .. HOSE_RANGE deg. At power-on the hose must be
// at the "0" end (or use "z" when it is there, or fit the home switch).
const float HOSE_RANGE = 230.0;          // deg the hose turns end to end
const int HOSE_SPEED = 300;    // us away from stop for the PULLING servo
                               // (continuous mode: 900..2100 us, stop ~1500)
const int STOP_US_1 = 1500;    // pulse where servo 1 stands still (trim)
const int STOP_US_2 = 1500;    // pulse where servo 2 stands still (trim)
const int SERVO1_SIGN = 1;     // -1 if servo 1 unwinds when it should wind
const int SERVO2_SIGN = -1;    // -1: servo 2 is mounted mirrored to servo 1
// Unwinding speed of the servo that pays out, as a fraction of the pulling
// speed. It must never be SLOWER than the hose moves, or the two strings
// pull against each other and the servos stall. A little faster (1.10) is
// safe: the extra string becomes slack, which the program counts and winds
// up at the next reversal. Try live with "j" and "k": the hose must turn
// smoothly and neither servo may buzz or stop.
const float PAYOUT_1 = 1.10;   // servo 1 unwinding while servo 2 pulls
// (as a fraction of each servo's own pulling speed, set by HOSE_SPEED)
const float PAYOUT_2 = 1.10;   // servo 2 unwinding while servo 1 pulls
// Time the hose needs from one end to the other, measured AFTER it starts
// moving, for each direction (the two strings are not equal).
// MEASURE: "z" at the 0 end, "+", "m" when the hose starts to move, "0" at
// the other end -> prints both times. Then the same with "-".
const unsigned long TRAVEL_MS_UP   = 6000;   // EXAMPLE - measure it ("+")
const unsigned long TRAVEL_MS_DOWN = 6000;   // EXAMPLE - measure it ("-")
// Slack of each string at power-on: the time its servo winds before the
// hose starts to move. MEASURE right after power-on (with TENSION_AT_START
// = false): "-", then "m" the moment the hose starts moving -> SLACK_MS_2.
const unsigned long SLACK_MS_1 = 0;      // tight string
const unsigned long SLACK_MS_2 = 600;    // looser string: EXAMPLE - measure
const bool TENSION_AT_START = true;      // wind string 2 tight at power-on
// Direction calibration: the hose position (0..HOSE_RANGE) at which the
// hose points at each bearing (bearing of sensor A with the scanner in the
// middle = 90, sensor B = 270). Several points, because the strings make
// the movement uneven. List them in order of position; a bearing may go
// past 360 (e.g. 400 = 40) so that the list keeps going the same way.
// MEASURE: point the hose at a known bearing with "+", "-", "0", then type
// "c <bearing>": it prints the pair to copy here. 3 to 7 points.
const uint8_t N_CAL = 5;
const int   CAL_BEARING[N_CAL] = { 65, 122, 180, 237, 295};  // EXAMPLE
const float CAL_POS[N_CAL]     = {  0,  57, 115, 172, 230};  // EXAMPLE
const bool HOSE_SWEEP = true;    // no fire: hose turns back and forth
// Optional end switches (microswitches to GND, pressed at the ends of the
// travel): A5 at the "0" end, D12 at the HOSE_RANGE end. Not fitted now, so
// false: A5 and D12 are not used at all.
const bool USE_END_SWITCHES = false;
// Without switches, the hose stays this far from both ends (the position
// is estimated, so it may be a few degrees off).
const float SAFE_MARGIN = 8.0;

// ---------------- Timing (ms) ----------------
const unsigned long T_SCAN_STEP = 15, T_CONFIRM = 400;
const unsigned long T_AIM_MAX = SLACK_MS_2 +
    (TRAVEL_MS_UP > TRAVEL_MS_DOWN ? TRAVEL_MS_UP : TRAVEL_MS_DOWN) + 1000;
const unsigned long T_FIRE_MAX = 10000, T_FLAME_GONE = 1500;
const unsigned long T_TILT_STEP = 20, T_IRR_BURST = 30000;
const unsigned long T_IRR_SOAK = 30000, T_TELEMETRY = 1000;
const unsigned long T_STARTUP = 1000;  // servos reach their start positions

enum State : uint8_t { MONITOR, CONFIRM, AIM, EXTINGUISH, TANK_EMPTY };
const char* const STATE_NAME[] =
  {"MONITOR", "CONFIRM", "AIM", "EXTINGUISH", "TANK_EMPTY"};

Servo scanner, rot1, rot2, hoseTilt;
State state = MONITOR;
unsigned long tState = 0, tStep = 0, tGone = 0, tIrr = 0, tTele = 0;
unsigned long tTilt = 0, tConfirmWait = T_CONFIRM;
int scanAngle = SCAN_MIN, scanDir = SCAN_STEP;
int bestDrop = 0, bestAngle = SCAN_CENTER; // largest drop below the landscape
const uint8_t N_SCAN = (SCAN_MAX - SCAN_MIN) / SCAN_STEP + 1;   // 116
// Landscape: normal reading per angle (stored /4 to save RAM), kept
// separately for each sweep direction ([0] = angle rising, [1] = falling),
// because the servo lags a little behind the command.
uint8_t baseA[2][N_SCAN], baseB[2][N_SCAN];
uint8_t sweepsDone = 0;            // < LEARN_SWEEPS: still learning
uint8_t hotCount = 0;              // angles of this sweep that look like flame
int prevAngle = -1;                // flame candidate of the previous sweep
bool prevIsB = false;
bool bestIsB = false;              // which sensor produced it
int targetBearing = -1;            // 0..359 deg around the mast, -1 = none
uint8_t watchPin = PIN_FLAME_A;    // sensor that keeps watching the flame
int tilt = TILT_NEAR, tiltDir = 1;
bool irrigating = false, soilDry = false;
unsigned long tManual = 0;         // pump test from the Serial Monitor
int soilNow = 0;                   // mean of the two soil sensors

// hose
float hosePos = 0;                 // where the hose is now, 0..HOSE_RANGE deg
int hoseDir = 0;                   // +1, -1 or 0 (stopped)
float hoseTarget = -1;             // where it must stop, -1 = no target
unsigned long tHose = 0, tRun = 0, tMoveStart = 0;
unsigned long slackLeft = 0;       // ms of slack still to take up
float slack1 = 0, slack2 = 0;      // deg of slack in string 1 and string 2
bool hoseManual = false;           // "+" / "-" running without limits
bool hoseSweepOn = HOSE_SWEEP;     // "+", "-", "p", "g" stop it, "s" restarts
float sweepEnd = HOSE_RANGE;       // end the idle movement goes to now
float payout1 = PAYOUT_1, payout2 = PAYOUT_2;   // can be changed live

// ---------------- Helpers ----------------
// goBILDA positional servo: 0..300 deg of its own travel.
void writeDeg(Servo& s, float deg, float usPerDeg) {
  int us = (int)(500 + deg * usPerDeg + 0.5);
  s.writeMicroseconds(constrain(us, 500, 2500));
}

// 4-sample average, about 0.45 ms on the ATmega328P
int readAvg(uint8_t pin) {
  int sum = 0;
  for (uint8_t i = 0; i < 4; i++) sum += analogRead(pin);
  return sum / 4;
}

void firePump(bool on) {
  digitalWrite(PIN_FIRE_IN3, on);
  digitalWrite(PIN_FIRE_IN4, LOW);
  analogWrite(PIN_FIRE_EN, on ? PUMP_PWM : 0);
}

void irrPump(bool on) {
  digitalWrite(PIN_IRR_IN1, on);
  digitalWrite(PIN_IRR_IN2, LOW);
  analogWrite(PIN_IRR_EN, on ? PUMP_PWM : 0);
  irrigating = on;
}

void enter(State s) { state = s; tState = millis(); }

int wrap360(int a) { a %= 360; return a < 0 ? a + 360 : a; }

// ---------------- Hose rotation (two goBILDA, continuous) ----------------
// Sets the two servo speeds. dir +1: servo 1 pulls; dir -1: servo 2 pulls.
// While slack is being taken up, the paying-out servo stands still.
void writeHoseServos(int dir, bool takingSlack) {
  int s1 = 0, s2 = 0;
  if (dir > 0) {
    s1 = HOSE_SPEED;
    s2 = takingSlack ? 0 : -(int)(HOSE_SPEED * payout2);
  } else if (dir < 0) {
    s2 = HOSE_SPEED;
    s1 = takingSlack ? 0 : -(int)(HOSE_SPEED * payout1);
  }
  // s > 0 means "wind my string"
  rot1.writeMicroseconds(constrain(STOP_US_1 + SERVO1_SIGN * s1, 500, 2500));
  rot2.writeMicroseconds(constrain(STOP_US_2 + SERVO2_SIGN * s2, 500, 2500));
}

unsigned long travelMs(int dir) { return dir > 0 ? TRAVEL_MS_UP : TRAVEL_MS_DOWN; }

void driveHose(int dir) {
  if (slackLeft > 0 && dir != hoseDir) {
    // stopped or reversed before the slack was all taken up: give it back
    float back = slackLeft * HOSE_RANGE / travelMs(hoseDir);
    if (hoseDir > 0) slack1 += back; else slack2 += back;
    slackLeft = 0;
  }
  if (dir != 0 && dir != hoseDir) {
    // starting or reversing: the pulling string first has to be wound
    // tight if it has slack; the hose does not move during that time
    float sl = (dir > 0) ? slack1 : slack2;
    slackLeft = (unsigned long)(sl * travelMs(dir) / HOSE_RANGE);
    if (dir > 0) slack1 = 0; else slack2 = 0;
    tMoveStart = 0;
  }
  hoseDir = dir;
  tHose = millis();
  writeHoseServos(dir, slackLeft > 0);
}

// Called every loop: takes up slack, then updates where the hose is from
// the time it really moved, and stops it at its target or at the ends.
void hoseUpdate() {
  unsigned long now = millis();
  unsigned long dt = now - tHose;
  tHose = now;
  if (hoseDir == 0) return;
  if (slackLeft > 0) {                    // string still slack: no movement
    if (dt < slackLeft) { slackLeft -= dt; return; }
    dt -= slackLeft;
    slackLeft = 0;
    writeHoseServos(hoseDir, false);      // now the other servo pays out
  }
  if (tMoveStart == 0) tMoveStart = now;
  float moved = dt * HOSE_RANGE / travelMs(hoseDir);
  hosePos += hoseDir * moved;
  // the servo that unwinds faster than the hose moves makes slack. Each
  // servo's own winding speed is known from its measured travel time.
  float vUp = HOSE_RANGE / TRAVEL_MS_UP, vDown = HOSE_RANGE / TRAVEL_MS_DOWN;
  float extra = (hoseDir > 0) ? payout2 * vDown - vUp : payout1 * vUp - vDown;
  if (extra > 0) { if (hoseDir > 0) slack2 += extra * dt; else slack1 += extra * dt; }
  if (USE_END_SWITCHES) {                 // at an end: exact position, stop
    if (hoseDir < 0 && digitalRead(PIN_HOME) == LOW) {
      hosePos = 0; hoseTarget = -100; driveHose(0); return;
    }
    if (hoseDir > 0 && digitalRead(PIN_END) == LOW) {
      hosePos = HOSE_RANGE; hoseTarget = -100; driveHose(0); return;
    }
  }
  if (hoseManual) return;                 // calibration: no limits
  if (hoseTarget > -50 &&
      ((hoseDir > 0 && hosePos >= hoseTarget) ||
       (hoseDir < 0 && hosePos <= hoseTarget))) {
    hosePos = constrain(hoseTarget, 0, HOSE_RANGE);
    hoseTarget = -100;
    driveHose(0);
  }
}

// Start turning the hose towards a position (0..HOSE_RANGE deg).
// With end switches, a move to an end goes on until the switch is pressed.
// Without them, the hose keeps SAFE_MARGIN away from the ends.
void hoseGoTo(float pos) {
  hoseManual = false;
  float lo = USE_END_SWITCHES ? (pos <= 0 ? -20 : 0) : SAFE_MARGIN;
  float hi = USE_END_SWITCHES ? (pos >= HOSE_RANGE ? HOSE_RANGE + 20 : HOSE_RANGE)
                              : HOSE_RANGE - SAFE_MARGIN;
  pos = constrain(pos, lo, hi);
  if (fabs(pos - hosePos) < 1.0) { hoseTarget = -100; driveHose(0); return; }
  hoseTarget = pos;
  driveHose(pos > hosePos ? 1 : -1);
}

bool hoseStopped() { return hoseDir == 0; }

// Hose position for a bearing, interpolated in the calibration table.
// A bearing outside the travel goes to the nearer end.
float posForBearing(int bearing) {
  const int first = CAL_BEARING[0], last = CAL_BEARING[N_CAL - 1];
  const int lo = first < last ? first : last, hi = first < last ? last : first;
  // the bearing as it is written in the table (it may be past 360)
  long b = bearing;
  while (b < lo) b += 360;
  while (b - 360 >= lo) b -= 360;
  if (b > hi) {                         // not reachable: nearer end
    long toLo = lo + 360 - b, toHi = b - hi;
    b = (toHi <= toLo) ? hi : lo;
  }
  for (uint8_t k = 0; k + 1 < N_CAL; k++) {
    long b0 = CAL_BEARING[k], b1 = CAL_BEARING[k + 1];
    if ((b >= b0 && b <= b1) || (b <= b0 && b >= b1)) {
      if (b1 == b0) return CAL_POS[k];
      return CAL_POS[k] + (CAL_POS[k + 1] - CAL_POS[k]) * (b - b0) / (float)(b1 - b0);
    }
  }
  return (b == first) ? CAL_POS[0] : CAL_POS[N_CAL - 1];
}

void aimHose(int bearing) { hoseGoTo(posForBearing(bearing)); }

// No fire: the hose turns back and forth between the two ends, nonstop.
void sweepHose() {
  if (!hoseStopped()) return;
  sweepEnd = (sweepEnd > HOSE_RANGE / 2) ? 0 : HOSE_RANGE;
  hoseGoTo(sweepEnd);
}

// ---------------- Flame scanner ----------------
uint8_t scanIndex(int angle) {
  return constrain((angle - SCAN_MIN) / SCAN_STEP, 0, N_SCAN - 1);
}

int bearingOf(int angle, bool isB) {
  int b = BEARING_A_CENTER + SCAN_SIGN * (angle - SCAN_CENTER);
  return wrap360(isB ? b + 180 : b);
}

// How much lower than the landscape a sensor reads now at this angle.
int flameDrop(uint8_t pin, int angle) {
  // the scanner stands still here: use the mean of the two directions
  uint8_t i = scanIndex(angle);
  int base = (pin == PIN_FLAME_B) ? (baseB[0][i] + baseB[1][i]) * 2
                                  : (baseA[0][i] + baseA[1][i]) * 2;
  return base - readAvg(pin);
}

bool learning() { return sweepsDone < LEARN_SWEEPS; }

// The only place that moves the scanner, so scanAngle is always where the
// scanner really is (the sweep continues from there after an alarm).
void setScanner(int angle) {
  scanAngle = constrain(angle, SCAN_MIN, SCAN_MAX);
  writeDeg(scanner, scanAngle, US_PER_DEG_GB);
}

// One scanner step. Returns true when a full sweep has just finished.
bool scanStep() {
  if (millis() < T_STARTUP) return false;
  if (millis() - tStep < T_SCAN_STEP) return false;
  tStep = millis();
  uint8_t i = scanIndex(scanAngle);
  uint8_t d = (scanDir > 0) ? 0 : 1;
  int a = readAvg(PIN_FLAME_A), b = readAvg(PIN_FLAME_B);
  if (learning()) {
    // first sweep in each direction copies, the next ones average in
    if (sweepsDone < 2) { baseA[d][i] = a / 4; baseB[d][i] = b / 4; }
    else { baseA[d][i] = (baseA[d][i] + a / 4) / 2; baseB[d][i] = (baseB[d][i] + b / 4) / 2; }
  } else {
    int dA = baseA[d][i] * 4 - a, dB = baseB[d][i] * 4 - b;
    if (dA > FLAME_DELTA || dB > FLAME_DELTA) hotCount++;
    if (dA > bestDrop) { bestDrop = dA; bestAngle = scanAngle; bestIsB = false; }
    if (dB > bestDrop) { bestDrop = dB; bestAngle = scanAngle; bestIsB = true; }
    // slow update of the landscape where nothing looks like a flame
    if (dA < FLAME_KEEP) baseA[d][i] += (a / 4 - baseA[d][i]) / 8;
    if (dB < FLAME_KEEP) baseB[d][i] += (b / 4 - baseB[d][i]) / 8;
  }
  int next = scanAngle + scanDir;
  bool done = (next >= SCAN_MAX || next <= SCAN_MIN);
  setScanner(next);
  if (done) {
    scanDir = -scanDir;
    targetBearing = -1;
    if (learning()) {
      sweepsDone++;
      if (!learning()) Serial.println(F("landscape learned, watching"));
    } else if (hotCount > N_SCAN / 3) {
      // a flame is small: if a large part of the view changed at once,
      // it is the light that changed, so learn the landscape again
      sweepsDone = 0;
      prevAngle = -1;
      if (irrigating) { irrPump(false); tIrr = millis(); }
      Serial.println(F("light changed, learning the landscape again"));
    } else if (bestDrop > FLAME_DELTA) {
      // accept a flame only if two sweeps in a row see it at the same place
      if (prevAngle >= 0 && prevIsB == bestIsB && abs(prevAngle - bestAngle) <= 10) {
        targetBearing = bearingOf(bestAngle, bestIsB);
        Serial.print(F("FLAME sensor "));          // to tune FLAME_DELTA
        Serial.print(bestIsB ? 'B' : 'A');
        Serial.print(F(" scan angle "));
        Serial.print(bestAngle);
        Serial.print(F(" bearing "));
        Serial.print(targetBearing);
        Serial.print(F(" drop "));
        Serial.println(bestDrop);
        watchPin = bestIsB ? PIN_FLAME_B : PIN_FLAME_A;
        prevAngle = -1;
      } else {
        prevAngle = bestAngle;
        prevIsB = bestIsB;
      }
    } else {
      prevAngle = -1;
    }
    bestDrop = 0;
    hotCount = 0;
  }
  return done;
}

// While the pump runs, the tilt servo (goBILDA) sweeps the jet near to far.
// TILT values keep the old scale: 90 = middle of the servo (1500 us).
void sweepTilt() {
  if (millis() - tTilt < T_TILT_STEP) return;
  tTilt = millis();
  tilt += tiltDir;
  if (tilt >= TILT_FAR || tilt <= TILT_NEAR) tiltDir = -tiltDir;
  writeDeg(hoseTilt, tilt + 60, US_PER_DEG_GB);
}

void irrigation() {
  int soil = (readAvg(PIN_SOIL_1) + readAvg(PIN_SOIL_2)) / 2;
  soilNow = soil;
  if (!SOIL_HIGH_IS_DRY) soil = 1023 - soil;   // turn it so that high = dry
  int dry = SOIL_HIGH_IS_DRY ? SOIL_DRY : 1023 - SOIL_DRY;
  int wet = SOIL_HIGH_IS_DRY ? SOIL_WET : 1023 - SOIL_WET;
  if (soil > dry) soilDry = true;
  if (soil < wet) soilDry = false;
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

// ---------------- Serial: telemetry and calibration ----------------
// Commands (Serial Monitor, 9600 baud, "Newline"):
//   +        servo 1 pulls the hose (position rising) until "0"; no limits
//   -        servo 2 pulls the hose the other way until "0"
//   m        type it when the hose REALLY starts to move: prints the slack time
//   t 600    string 2 has 600 ms of slack now (e.g. after re-tying it)
//   0        stop; prints total time, time since "m" (= travel) and position
//   z        the hose is at the "0" end now: position = 0
//   p 115    turn the hose to position 115 deg (0..230)
//   g 90     turn the hose to bearing 90
//   c 90     the hose points at bearing 90 now: prints a calibration point
//   s        restart the back-and-forth movement of the hose
//   j 105    servo 1 unwinds at 105 % of the pulling speed (PAYOUT_1)
//   k 105    servo 2 unwinds at 105 % of the pulling speed (PAYOUT_2)
//   i        test: irrigation pump on for 30 s (ignores the soil sensors)
//   x        test: fire pump on for 30 s
unsigned long tMark = 0;
void serialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  int val = line.substring(1).toInt();
  if (cmd == '+' || cmd == '-') {
    hoseSweepOn = false;
    hoseManual = true;
    hoseTarget = -100;
    tRun = millis();
    tMark = 0;
    driveHose(cmd == '+' ? 1 : -1);
    Serial.println(F("hose turning: type m when it starts moving, 0 to stop"));
    return;
  }
  if (cmd == 'm') {
    tMark = millis();
    Serial.print(F("slack time: "));
    Serial.print(tMark - tRun);
    Serial.println(F(" ms  (SLACK_MS_1 for +, SLACK_MS_2 for -)"));
    return;
  }
  if (cmd == '0') {
    unsigned long now = millis();
    hoseUpdate();
    driveHose(0);
    hoseManual = false;
    hoseSweepOn = false;
    Serial.print(F("stopped: total "));
    Serial.print(now - tRun);
    Serial.print(F(" ms"));
    if (tMark) {
      Serial.print(F(", moving "));
      Serial.print(now - tMark);
      Serial.print(F(" ms (end to end = TRAVEL_MS_UP / TRAVEL_MS_DOWN)"));
    }
    Serial.print(F(", position "));
    Serial.print(hosePos, 0);
    Serial.println(F(" deg"));
    return;
  }
  if (cmd == 'z') { hosePos = 0; Serial.println(F("position = 0")); return; }
  if (cmd == 't') { slack2 = val * HOSE_RANGE / TRAVEL_MS_DOWN; return; }
  if (cmd == 'p') { hoseSweepOn = false; hoseGoTo(val); }
  if (cmd == 'g') { hoseSweepOn = false; aimHose(val); }
  if (cmd == 's') { hoseSweepOn = true; Serial.println(F("hose sweep on")); return; }
  if (cmd == 'c') {                    // one point of the calibration table
    Serial.print(F("calibration point: bearing "));
    Serial.print(val);
    Serial.print(F(" -> position "));
    Serial.print(hosePos, 0);
    Serial.println(F("   (CAL_BEARING / CAL_POS)"));
    return;
  }
  if (cmd == 'j' || cmd == 'k') {
    float r = constrain(val, 0, 200) / 100.0;
    if (cmd == 'j') payout1 = r; else payout2 = r;
    writeHoseServos(hoseDir, slackLeft > 0);    // apply it now
    Serial.print(cmd == 'j' ? F("servo 1") : F("servo 2"));
    Serial.print(F(" unwinds at "));
    Serial.print(val);
    Serial.println(F(" % of the pulling speed"));
    return;
  }
  if (cmd == 'i' && state != TANK_EMPTY) {
    firePump(false); irrPump(true); tManual = millis();
  }
  if (cmd == 'x' && state != TANK_EMPTY) {
    irrPump(false); firePump(true); tManual = millis();
  }
  if ((cmd == 'i' || cmd == 'x') && state == TANK_EMPTY)
    Serial.println(F("TANK_EMPTY: pumps locked, check the water-level sensor"));
  Serial.print(F("soil="));
  Serial.print(soilNow);
  Serial.print(soilDry ? F(" (dry) ") : F(" (not dry) "));
  Serial.print(F("hose="));
  Serial.print(hosePos, 0);
  Serial.println(F(" deg"));
}

void telemetry() {
  if (millis() - tTele < T_TELEMETRY) return;
  tTele = millis();
  Serial.print(learning() ? "LEARNING" : STATE_NAME[state]);
  Serial.print(',');
  Serial.print(analogRead(PIN_LEVEL));    Serial.print(',');
  Serial.print(analogRead(PIN_FLAME_A));  Serial.print(',');
  Serial.print(analogRead(PIN_FLAME_B));  Serial.print(',');
  Serial.print(analogRead(PIN_SOIL_1));   Serial.print(',');
  Serial.print(analogRead(PIN_SOIL_2));   Serial.print(',');
  Serial.print(targetBearing);            Serial.print(',');
  Serial.print(hosePos, 0);               Serial.print(',');
  Serial.print(hoseDir);                  Serial.print(',');
  Serial.println(scanAngle);
}

void setup() {
  const uint8_t outs[] = {PIN_IRR_IN1, PIN_IRR_IN2, PIN_IRR_EN,
                          PIN_FIRE_IN3, PIN_FIRE_IN4, PIN_FIRE_EN};
  for (uint8_t i = 0; i < sizeof(outs); i++) {
    pinMode(outs[i], OUTPUT);
    digitalWrite(outs[i], LOW);
  }
  if (USE_END_SWITCHES) {               // A5 and D12 stay untouched otherwise
    pinMode(PIN_HOME, INPUT_PULLUP);
    pinMode(PIN_END, INPUT_PULLUP);
  }
  scanner.attach(PIN_SCAN, 500, 2500);
  rot1.attach(PIN_ROT_1, 500, 2500);
  rot2.attach(PIN_ROT_2, 500, 2500);
  hoseTilt.attach(PIN_TILT, 500, 2500);
  driveHose(0);                         // hose stopped
  writeDeg(hoseTilt, TILT_NEAR + 60, US_PER_DEG_GB);
  setScanner(SCAN_MIN);
  Serial.begin(9600);
  Serial.setTimeout(50);
  Serial.println(F("state,level,flameA,flameB,soil1,soil2,bearing,hosePos,hoseDir,scan"));
  // initial slack of the strings (deg)
  slack1 = SLACK_MS_1 * HOSE_RANGE / TRAVEL_MS_UP;
  slack2 = SLACK_MS_2 * HOSE_RANGE / TRAVEL_MS_DOWN;
  if (TENSION_AT_START && SLACK_MS_2 > 0) {
    // servo 2 winds its loose string while servo 1 stands still and holds
    // the hose (the only delay() in the program, at power-on)
    rot1.writeMicroseconds(STOP_US_1);
    rot2.writeMicroseconds(constrain(STOP_US_2 + SERVO2_SIGN * HOSE_SPEED, 500, 2500));
    delay(SLACK_MS_2);
    rot2.writeMicroseconds(STOP_US_2);
    slack2 = 0;
    Serial.println(F("string 2 tensioned"));
  }
  tHose = millis();
  if (USE_END_SWITCHES) {               // find the 0 end on the switch
    hosePos = HOSE_RANGE;
    sweepEnd = HOSE_RANGE;              // first sweep goes to 0
  }
}

void loop() {
  int level = readAvg(PIN_LEVEL);

  // Dry-run protection has priority over every other state.
  if (!LEVEL_CHECK) level = 1023;
  if (level < LEVEL_MIN && state != TANK_EMPTY) {
    firePump(false);
    irrPump(false);
    tManual = 0;
    enter(TANK_EMPTY);
  }

  hoseUpdate();                       // where is the hose, stop at target

  switch (state) {
    case MONITOR:
      if (tManual) {                    // pump test running
        if (millis() - tManual >= T_IRR_BURST) {
          firePump(false); irrPump(false); tManual = 0; tIrr = millis();
        }
      } else if (!learning()) irrigation();
      if (scanStep() && targetBearing >= 0) {
        irrPump(false);                 // fire has priority over watering
        firePump(false);
        tManual = 0;
        // Return the scanner to where it saw the flame and look again,
        // after the time the servo needs to get there.
        tConfirmWait = (unsigned long)(abs(scanAngle - bestAngle) *
                                       SCAN_MS_PER_DEG) + T_CONFIRM;
        setScanner(bestAngle);
        enter(CONFIRM);
      }
      break;

    case CONFIRM:                       // still there after the wait?
      if (millis() - tState >= tConfirmWait) {
        if (flameDrop(watchPin, scanAngle) > FLAME_KEEP) {
          aimHose(targetBearing);       // the hose turns towards the flame
          tilt = TILT_NEAR;
          tiltDir = 1;
          writeDeg(hoseTilt, tilt + 60, US_PER_DEG_GB);
          enter(AIM);
        } else {
          enter(MONITOR);               // false alarm (reflection, sun)
        }
      }
      break;

    case AIM:                           // wait until the hose is there
      if (hoseStopped() || millis() - tState >= T_AIM_MAX) {
        driveHose(0);
        hoseTarget = -100;
        firePump(true);
        tGone = 0;
        enter(EXTINGUISH);
      }
      break;

    case EXTINGUISH: {
      sweepTilt();
      if (flameDrop(watchPin, scanAngle) > FLAME_KEEP) tGone = 0;
      else if (tGone == 0) tGone = millis();
      bool out = (tGone != 0 && millis() - tGone >= T_FLAME_GONE);
      bool timeout = (millis() - tState >= T_FIRE_MAX);
      if (out || timeout) {             // stop, then rescan to verify
        firePump(false);
        bestDrop = 0;
        enter(MONITOR);                 // the hose starts moving again
      }
      break;
    }

    case TANK_EMPTY:                    // pumps stay locked until refilled
      scanStep();
      if (level > LEVEL_OK) enter(MONITOR);
      break;
  }
  serialCommands();
  // no fire: keep the hose turning back and forth
  if (hoseSweepOn && !hoseManual && (state == MONITOR || state == TANK_EMPTY))
    sweepHose();
  telemetry();
}
