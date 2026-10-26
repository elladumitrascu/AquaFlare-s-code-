# AquaFlare

**Battery-powered plot with fire detection, fire suppression and irrigation, with solar top-up charging**

RoboChallenge 2026 – Freestyle Showcase High School · Team **RaSky**

## General information

- **Category:** Freestyle Showcase High School
- **School:** Colegiul Național „Elena Cuza”, Craiova, 10th grade
- **Team:** RaSky
- **Coordinator:** prof. Vlăduțoiu Fleur

---

## Description

AquaFlare is a stationary demonstrator of an automated plot of land: a wooden box of 50 × 50 × 25 cm filled with agricultural soil, with sensors, pumps and electronics. A wooden platform crosses the middle of the box and carries the electronics and the water hose. The water reserve is a container under the soil, fed by a 2 L bottle in each corner of the box.

The same water reserve is used for two jobs:

- **Fire protection:** two infrared flame sensors are mounted back to back on one goBILDA servo that sweeps 230° left and right, so together they watch the full circle around the plot. When a flame is found, the hose is turned towards it by **two goBILDA servos in continuous mode, each with a string spool; both strings are tied to the hose** and pull it in opposite directions. A third goBILDA servo tilts the hose up and down, and a submersible pump sprays water until the flame is gone.
- **Irrigation:** two soil-moisture sensors decide when a second pump waters the soil, in bursts with pauses in between.

The water-level sensor stops the pumps when it no longer detects water. Fire always has priority over irrigation.

**Power:** AquaFlare runs from **two Li-ion batteries**, its main energy source: one for the logic and one for the servos and pumps. Small solar panels only top the batteries up. The system is **not fully autonomous**: the batteries must be charged, the water reserve refilled, and the hose must be at its starting end when the system is switched on.

---

## Hardware

| Component | Qty | Role |
|---|---|---|
| Arduino UNO | 1 | Controller, powered with 5 V on the 5V and GND pins |
| Flame sensor module | 2 | Fire detection (analog output) |
| Soil-moisture sensor | 2 | Irrigation decision |
| Water-level sensor | 1 | Stops the pumps when there is no water |
| goBILDA 2000 Series Dual Mode servo (25-2, Torque) | 4 | Flame scanner and hose tilt (servo mode); hose rotation ×2 (continuous mode) |
| String spool (3D-printed) | 2 | One on each continuous servo; the strings are tied to the hose |
| L298N motor driver | 1 | Switches the two pumps, fed at 7.4 V from the servo and pump battery |
| Submersible water pump, 3–6 V | 2 | Fire jet and irrigation |
| Li-ion cell 3.7 V, 2200 mAh | 2 | Logic battery, 2S: 7.4 V, 2200 mAh |
| Li-ion cell 3.7 V, 3000 mAh | 4 | Servo and pump battery, 2S2P: 7.4 V, 6000 mAh |
| 2S BMS module | 2 | Battery protection |
| Buck converter | 1 | 7.4 V → 5 V for the logic |
| XL4016 buck converter with voltmeter | 1 | 7.4 V → 6 V for the servos |
| MPPT solar charge module | 2 | Top-up charging of each battery |
| Solar panel 6 V, 160 mA | 5 | Top-up charging |
| Solar panel 9 V | 1 | Top-up charging |

### Power systems

```
Logic:          2S Li-ion 7.4 V, 2200 mAh + BMS  (main source)
                → buck converter 7.4 V → 5 V → Arduino 5V/GND pins, 5 sensors
                top-up: 2 × 6 V + 1 × 9 V panels (21 V, 160 mA) → MPPT module

Servos & pumps: 2S2P Li-ion 7.4 V, 6000 mAh + BMS  (main source)
                → XL4016 buck converter → 6 V → 4 goBILDA servos
                → 7.4 V direct → L298N +12V input → pumps (PWM 190/255)
                top-up: 3 × 6 V panels (18 V, 160 mA) → MPPT module
```

Both batteries share only GND with the Arduino.

---

## Wiring

| Arduino pin | Connected to |
|---|---|
| A0 | Water-level sensor |
| A1 | Flame sensor A |
| A2 | Flame sensor B (back to back with A) |
| A3, A4 | Soil-moisture sensors 1 and 2 |
| D2, D4, D5 | L298N IN1, IN2, ENA – **irrigation pump** (OUT1/OUT2) |
| D7, D8, D6 | L298N IN3, IN4, ENB – **fire pump** (OUT3/OUT4) |
| D9 | Scanner servo (goBILDA, servo mode) – carries both flame sensors |
| D11 | Hose spool servo 1 (goBILDA, continuous) – tight string |
| D10 | Hose spool servo 2 (goBILDA, continuous) – looser string |
| D3 | Hose tilt servo (goBILDA, servo mode) |
| A5, D12, D13 | Free (A5 and D12 can take optional end switches for the hose) |
| 5V, GND | 5 V output of the logic buck converter |

---

## How the hose turns

- Direction **+**: servo 1 winds its string (pulls), servo 2 unwinds its string 10 % faster.
- Direction **−**: servo 2 winds (pulls), servo 1 unwinds 10 % faster.
- The faster unwinding never lets the strings pull against each other; the extra string becomes slack, which the program counts and winds up at the next reversal (the hose does not move during that time).
- At power-on, servo 2 winds its looser string for `SLACK_MS_2` while servo 1 holds the hose.
- There is **no position sensor**: the hose position (0–230°) is estimated from the time it really moved, with a measured travel time for each direction. The hose must be at its "0" end when the system is switched on, and it keeps 8° away from both ends so that it never reaches the components behind it.
- The direction of the hose is calibrated at 5 points (`CAL_BEARING` / `CAL_POS`) because the strings make the movement uneven.

---

## Software

The code is written in **C++ in the Arduino IDE** and is a single sketch, `AquaFlare.ino`. The only library used is **Servo.h**. The program is a non-blocking state machine (all timing uses `millis()`; the only `delay()` is the 0.6 s string tensioning at power-on).

| State | What happens |
|---|---|
| `MONITOR` | The scanner sweeps 230° in 2° steps every 15 ms (about 1.7 s per sweep). The first 4 sweeps learn the normal reading of each sensor at every angle; then a reading more than 120 below that is a flame, accepted only if two sweeps in a row see it in the same direction. Irrigation runs and the hose moves back and forth |
| `CONFIRM` | The scanner turns back to the flame; it must still be seen after the servo arrives + 400 ms |
| `AIM` | The spool servos turn the hose to the direction of the flame (max 7.6 s) |
| `EXTINGUISH` | The fire pump runs and the tilt servo moves the jet up and down, until the flame has been gone for 1.5 s (max 10 s) |
| `TANK_EMPTY` | No water detected: both pumps stay off until the container is refilled |

**Irrigation:** our soil probes read **low** when the soil is dry. Watering starts below `SOIL_DRY` (300) and stops above `SOIL_WET` (450), in 30 s bursts with 30 s pauses.

**Telemetry:** once per second the board prints:
`state, level, flameA, flameB, soil1, soil2, bearing, hosePos, hoseDir, scan`

### Calibration (Serial Monitor, 9600 baud, "Newline")

| Command | What it does |
|---|---|
| `+` / `-` | Servo 1 / servo 2 pulls the hose, no limits |
| `m` | Type it when the hose really starts moving: prints the slack time |
| `0` | Stop: prints total time, moving time and position |
| `z` | The hose is at the "0" end now |
| `p 115` | Move the hose to position 115 |
| `g 90` | Point the hose at bearing 90 |
| `c 90` | The hose points at bearing 90 now: prints a calibration point |
| `s` | Restart the back-and-forth movement |
| `j 110` / `k 110` | Servo 1 / servo 2 unwinds at 110 % of the pulling speed |
| `t 600` | String 2 has 600 ms of slack now |
| `i` / `x` | Test: irrigation pump / fire pump on for 30 s |

Values to measure on the robot (marked `EXAMPLE` in the code): `TRAVEL_MS_UP`, `TRAVEL_MS_DOWN`, `SLACK_MS_2`, `CAL_BEARING` / `CAL_POS`. Also check the soil, level and flame thresholds in the telemetry.

---

## Upload

1. Install **Arduino IDE 2.x**.
2. Open `AquaFlare.ino` (it must be in a folder with the same name).
3. Select the board **Arduino Uno** and the correct port.
4. Click **Upload**.
5. Switch the system on with the hose at its "0" end and with no infrared source in front of the sensors for the first seconds.

No extra libraries are needed: `Servo.h` comes with the Arduino IDE.

---

## Team

- Dumitrașcu Elena Camelia
- Ionică Erik Andrei
- Sanda Dan Valentin

Colegiul Național „Elena Cuza”, Craiova · Team RaSky · Coordinator: prof. Vlăduțoiu Fleur
