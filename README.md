# AquaFlare-s-code-
# AquaFlare

**Solar-powered autonomous plot with fire detection, fire suppression and irrigation**

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

- **Fire protection:** two infrared flame sensors are mounted back to back on one servo that sweeps 180° in about 10 s, so together they watch the full circle around the plot. When a flame is found, an Axon MAX servo turns the hose towards it, a goBILDA servo moves the hose up and down, and a submersible pump sprays water until the flame is gone.
- **Irrigation:** two soil-moisture sensors decide when a second pump waters the soil, in short bursts with pauses in between.

The water-level sensor stops the pumps when it no longer detects water, so that the pumps are not damaged. Fire always has priority over irrigation.

The whole system runs on solar energy, with two separate battery systems: one for the logic and one for the servos.

---

## Hardware

| Component | Qty | Role |
|---|---|---|
| Arduino UNO | 1 | Controller, powered with 5 V on the 5V and GND pins |
| Flame sensor module | 2 | Fire detection (analog output) |
| Soil-moisture sensor | 2 | Irrigation decision |
| Water-level sensor | 1 | Stops the pumps when there is no water |
| goBILDA 2000 Series Dual Mode servo (25-2, Torque) | 2 | Flame scanner (both sensors) and hose tilt |
| Axon MAX servo | 1 | Hose rotation (~350°) |
| L298N motor driver (OKY3195) | 1 | Switches the two pumps |
| Submersible water pump, 3–6 V | 2 | Fire jet and irrigation |
| Li-ion cell 3.7 V, 2200 mAh | 2 | Logic battery, 2S: 7.4 V, 2200 mAh |
| Li-ion cell 3.7 V, 3000 mAh | 4 | Servo battery, 2S2P: 7.4 V, 6000 mAh |
| 2S BMS module | 2 | Battery protection |
| MPPT solar charge module | 2 | Charges each battery from its panels |
| Buck converter | 1 | 7.4 V → 5 V for the logic |
| XL4016 buck converter with voltmeter | 1 | 7.4 V → 6 V for the servos |
| Solar panel 6 V, 160 mA | 5 | Energy source |
| Solar panel 9 V | 1 | Energy source |

### Power systems

```
Logic:  2 × 6 V + 1 × 9 V panels in series (21 V, 160 mA)
        → MPPT charge module → 2S BMS + 7.4 V 2200 mAh battery
        → buck converter 7.4 V → 5 V
        → Arduino 5V/GND pins, 5 sensors, L298N motor driver

Servos: 3 × 6 V panels in series (18 V, 160 mA)
        → MPPT charge module → 2S BMS + 7.4 V 6000 mAh battery
        → XL4016 buck converter → 6 V
        → scanner servo, hose tilt servo, hose rotation servo
```

---

## Wiring

| Arduino pin | Connected to |
|---|---|
| A0 | Water-level sensor |
| A1 | Flame sensor A (front half, 0°–180°) |
| A2 | Flame sensor B (back half, 180°–360°) |
| A3, A4 | Soil-moisture sensors 1 and 2 |
| D2, D4, D5 | L298N IN1, IN2, ENA – fire pump |
| D7, D8, D6 | L298N IN3, IN4, ENB – irrigation pump |
| D9 | Scanner servo (goBILDA Torque) |
| D10 | Hose tilt servo (goBILDA Torque) |
| D11 | Hose rotation servo (Axon MAX) |
| 5V, GND | 5 V output of the logic buck converter |

The sensor VCC wires are joined into one 5V pin and their GND wires into one GND pin. The servos take their power from the 6 V output of the XL4016; only their signal wires go to the Arduino.

---

## Software

The code is written in **C++ in the Arduino IDE** and is a single sketch, `AquaFlare.ino`. The only library used is **Servo.h**.

The program is a non-blocking state machine (no `delay()`; all timing uses `millis()`):

| State | What happens |
|---|---|
| `MONITOR` | The scanner sweeps 180° in 2° steps (about 10 s per sweep); the irrigation task runs |
| `CONFIRM` | The scanner turns back to the flame; it must still be seen after 400 ms |
| `AIM` | The Axon MAX turns the hose towards the flame (500 ms) |
| `EXTINGUISH` | The fire pump runs and the tilt servo moves the jet up and down, until the flame has been gone for 1.5 s (max. 10 s) |
| `TANK_EMPTY` | No water detected: both pumps stay off until the container is refilled |

**Irrigation:** when the mean of the two soil sensors is drier than `SOIL_DRY`, the irrigation pump runs for 3 s, then waits 30 s before checking again. It stops when the soil is wetter than `SOIL_WET`.

**Calibration:** all thresholds (`FLAME_ON`, `FLAME_OFF`, `SOIL_DRY`, `SOIL_WET`, `LEVEL_MIN`, `LEVEL_OK`) are constants at the top of the file. Read the real sensor values in the Serial Monitor (9600 baud) and adjust them for your sensors.

**Telemetry:** once per second the board prints one line over Serial:
`state, level, flameA, flameB, soil1, soil2, bearing`

---

## Upload

1. Install **Arduino IDE 2.x**.
2. Open `AquaFlare.ino`.
3. Select the board **Arduino Uno** and the correct port.
4. Click **Upload**.

No extra libraries are needed: `Servo.h` comes with the Arduino IDE.

---

## Team

- Dumitrașcu Elena Camelia
- Ionică Erik Andrei
- Sanda Dan Valentin

Colegiul Național „Elena Cuza”, Craiova · Team RaSky · Coordinator: prof. Vlăduțoiu Fleur
