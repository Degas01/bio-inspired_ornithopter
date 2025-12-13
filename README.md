# R.O.B.I.N. - Robot Ornithopter Bio-Inspired by Nature

<p align="center">
  <img width="2048" height="758" alt="IMG-20250319-WA0011" src="https://github.com/user-attachments/assets/657438b4-bb4d-4284-b9f2-4693eaa32f16" />
</p>

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue)]()
[![CAD](https://img.shields.io/badge/Fusion_360-0696D7?logo=autodesk&logoColor=white)](https://www.autodesk.com/products/fusion-360)
[![CFD](https://img.shields.io/badge/ANSYS-FFB71B?logo=ansys&logoColor=black)](https://www.ansys.com/)
[![Build Status](https://img.shields.io/badge/build-prototype-yellow)](.)
[![Hardware](https://img.shields.io/badge/Hardware-Arduino%20%7C%20BLDC-orange)]()
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![King's College London](https://img.shields.io/badge/Institution-King's_College_London-blue.svg)](https://www.kcl.ac.uk/)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

*A bio-inspired flying robot designed for seamless wildlife monitoring and conservation*

---

## Project Overview

This repository contains the full implementation of my MSc Robotics Engineering Group Project at King’s College London:

> **R.O.B.I.N. - Robot Ornithopter Bio-Inspired by Nature**

*MSc Robotics Engineering Group Project | King’s College London | 2025*

This project develops a bio-inspired flapping-wing aerial robot combining:

- **Lightweight mechanical design and CAD-based prototyping**
- **Transverse crank-driven flapping-wing actuation**
- **Aerodynamic analysis via transient CFD simulations**
- **Embedded control using Arduino-based electronics**
- **Experimental lift, glide and propulsion testing**

The system was designed, simulated, prototyped and experimentally validated as a low-disturbance aerial platform for environmental monitoring and conservation applications.

---

## Project Description

R.O.B.I.N. is a bio-inspired flapping-wing aerial robot developed to investigate low-disturbance flight for environmental monitoring applications. The system integrates lightweight mechanical design with a transverse crank-based flapping mechanism to generate symmetric wing motion, enabling lift and forward propulsion without conventional rotors. Aerodynamic behaviour is analysed through transient CFD simulations to characterise lift generation, flow separation and wake dynamics. It is validated via physical prototyping and experimental testing. The platform incorporates embedded control using Arduino-based electronics, RF communication and inertial sensing to support controlled flight experiments and performance evaluation. R.O.B.I.N. serves as an experimental testbed for studying bio-inspired aerial locomotion, aerodynamics and system-level design trade-offs in flapping-wing robotics.

## Table of Contents

- [1. Project Motivation] (#project-motivation)
- [2. Key Features](#features)
- [3. System Architecture](#system-architecture)
- [4. Mechanical Design](#mechanical-design)
- [5. Electronics & Control](#electronics--control)
- [6. Software](#software)
- [7. CFD Analysis](#cfd-analysis)
- [8. Testing & Results](#testing--results)
- [9. Installation & Setup](#installation--setup)
- [10. Future Work](#future-work)
- [11. Bill of Materials](#bill-of-materials)
- [12. Sustainability & Impact](#sustainability-&-impact)
- [13. Team](#team)
- [14. Acknowledgments](#acknowledgments)
- [15. License](#license)
- [16. References](#references)

---

## 1. Project Motivation

Traditional UAVs used in wildlife monitoring suffer from critical issues:
- **Noise pollution**: Causes anti-predatory responses in animals
- **Stress induction**: Increases mortality rates in wildlife
- **Data quality**: Animals flee before adequate observation
- **Environmental disruption**: High-frequency motor sounds

**Solution:** A bio-inspired approach that:
- Mimics natural bird flight for seamless habitat integration
- Reduces acoustic signature by 70% compared to traditional drones
- Enables closer wildlife observation without behavioral disruption
- Supports **UN SDG 15: Life on Land**

---

## 2. Key Features

### Mechanical Innovation
- **Dual Transverse Shaft Mechanism**: Converts rotary to flapping motion
- **Adjustable Crank Offset**: Variable flapping amplitude (formula-driven design)
- **Lightweight Construction**: 248g total weight with optimized structure
- **Bio-inspired Wing Profile**: Flexible steel-frame + nylon membrane

### Control Systems
- **RF Wireless Control**: 2000bps bidirectional communication
- **IMU Stabilization**: MPU6050 with complementary filter (α=0.98)
- **Dual Servo Tail**: Independent pitch and yaw control
- **Brushless Motor**: 2300KV BLDC with ESC integration

### Analysis & Validation
- **CFD Simulation**: Laminar flow analysis at Re≈2.3×10⁴
- **Lift Testing**: Spring scale measurements up to 204 RPM
- **Aerodynamic Optimization**: Mesh refinement with boundary layer analysis
- **Material Testing**: Iterative prototyping with PLA, steel, nylon

---

## 3. System Architecture
```
┌─────────────────────────────────────────────────────────┐
│                     R.O.B.I.N. System                    │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐         ┌──────────────┐            │
│  │   RF Remote  │────────▶│  Arduino     │            │
│  │  Controller  │  2000bps│  Nano        │            │
│  │  (Joystick + │         │  (Receiver)  │            │
│  │   Pot)       │         └──────┬───────┘            │
│  └──────────────┘                │                     │
│                                   │                     │
│                    ┌──────────────┼──────────────┐     │
│                    ▼              ▼              ▼     │
│            ┌──────────┐   ┌──────────┐   ┌──────────┐ │
│            │  BLDC    │   │  Servo   │   │  MPU6050 │ │
│            │  Motor   │   │  (Tail)  │   │  (IMU)   │ │
│            │  +ESC    │   │  x2      │   │          │ │
│            └────┬─────┘   └────┬─────┘   └────┬─────┘ │
│                 │              │              │       │
│                 ▼              ▼              ▼       │
│         ┌──────────────────────────────────────────┐  │
│         │       Flapping        Tail       Feedback│  │
│         │       Mechanism      Control      Loop   │  │
│         └──────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## 4. Mechanical Design

### Flapping Mechanism

The heart of R.O.B.I.N. is a **dual transverse shaft mechanism** inspired by avian biomechanics.

**Design Equations:**
```
Gear Reduction = (Motor KV × Battery Voltage) / (Flapping Rate × 60)
                = (2300 × 7.4V) / (6Hz × 60) ≈ 50:1

Flapping Angle φ = sin⁻¹(Crank Offset / 12mm)

Amplitude = 2 × Wing Span × sin(φ)
```

**Key Components:**
- 50:1 gear train (PLA gears + steel shafts)
- 6× precision bearings (heat stress mitigation)
- 2300KV BLDC motor
- 2S LiPo battery (7.4V)

<p align="center"> 
  <img width="1299" height="390" alt="Front View of Flapping Mechanism in Fusion Model" src="https://github.com/user-attachments/assets/6423d892-c1e5-4924-b0e5-9fa7ea47bd0a" />
  <br>
  <em> Front View of Flapping Mechanism in Fusion Model </em

<p align="center"> 
  <img width="567" height="143" alt="Side View of Flapping Mechanism in Fusion Model" src="https://github.com/user-attachments/assets/5020e6da-8483-4296-bdb0-393b5a0ad988" />
  <br>
  <em> Side View of Flapping Mechanism in Fusion Model </em

<p align="center">
  <img width="2048" height="758" alt="IMG-20250319-WA0011" src="https://github.com/user-attachments/assets/7ab812d9-0fe6-4694-9234-c8e4b08a3551" />
  <br>
  <em> CAD model of flapping mechanism </em>

<div align="center">
  <h3>Mechanism in motion</h3>
  <video src="https://github.com/user-attachments/assets/8695b5c1-417e-4c3c-990f-2ece7f1de090" width="400" controls></video>
</div>

### Wing Design

**Specifications:**
- **Wingspan**: 1020mm (optimized for lift-to-weight ratio)
- **Frame Material**: 1.8mm steel rods (5× 500mm + bracing)
- **Membrane**: Nylon sheet (semi-elliptical shape)
- **Wing Loading**: ~0.8 g/cm² (avian-inspired)
- **Degrees of Freedom**: Single z-axis motion

**Biomimetic Features:**
- Flexible trailing edge (natural deformation during flapping)
- Single support batten (mimics bird wing bone structure)
- Cambered profile potential for lift optimization

<p align="center">
  <img width="797" height="364" alt="Schematic of intended wing design" src="https://github.com/user-attachments/assets/e6237917-c4bd-4384-9f37-893f42f72111" />
  <br>
  <em> Wing schematic </em>

<p align="center">
  <img width="173" height="130" alt="Fully assembled wings" src="https://github.com/user-attachments/assets/764cd4c6-9663-49bf-9a39-94cf113974e5" />
  <br>
  <em> Fully assembled wings </em>

<p align="center">
  <img width="667" height="177" alt="Mechanical properties of wing membrane and frame materials" src="https://github.com/user-attachments/assets/96584f28-e8a6-408b-90d8-65fb5cd79b45" />
  <br>
  <em> Material comparison table </em>

**Material Selection Analysis:**

| Material | Young's Modulus | Tensile Strength | Density | Selected |
|----------|----------------|------------------|---------|----------|
| Elastane | 10-100 MPa | 500-1000 MPa | 1210-1350 kg/m³ | ✓ (ideal) |
| Nylon | - | - | - | ✓ (used) |
| Carbon Fiber | 200-500 GPa | 3-7 GPa | 1600-2000 kg/m³ | Future work |

### Body Structure

**Center of Gravity Optimization:**
- Positioned 26% of chord length from nose
- Balanced for pitch stability during flight
- Total mass: 248g (97g body + 151g electronics/wings)

<p align="center">
  <img width="743" height="207" alt="CAD location of the Centre of Gravity of the Prototype, computed through Fusion 360" src="https://github.com/user-attachments/assets/3e955248-b013-4668-a8c9-271d97fb3b96" />
  <br>
  <em> CAD location of the Centre of Gravity of the Prototype, computed through Fusion 360 </em>

<p align="center">
  <img width="1360" height="1020" alt="Fully assembled robot" src="https://github.com/user-attachments/assets/c97cf330-4a4b-4643-ba8c-cf2d684f8967" />
  <br>
  <em> Fully assembled prototype </em>

<p align="center">
  <img width="1360" height="1020" alt="Full assembled robot (top view)" src="https://github.com/user-attachments/assets/fea09813-e538-47bc-9bfb-91b7a05e8661" />
  <br>
  <em> Full assembled prototype (top view) </em>

<p align="center">
  <img width="1360" height="1020" alt="Close-up of gearbox and electronics" src="https://github.com/user-attachments/assets/78fe04db-24a3-4239-8907-3e8d7037a7f5" />
  <br>
  <em> Close-up of gearbox and electronics </em>

---

## 5. Electronics & Control

### Hardware Components

| Component | Model | Quantity | Function |
|-----------|-------|----------|----------|
| Microcontroller | Arduino Nano | 2 | Processing & control |
| Motor | 2300KV BLDC | 1 | Flapping actuation |
| ESC | 30A | 1 | Motor speed control |
| IMU | MPU6050 | 1 | Orientation sensing |
| Servos | Micro 9g | 2 | Tail control |
| RF Module | RH_ASK 433MHz | 2 | Wireless communication |
| Battery | 2S LiPo (7.4V) | 1 | Power supply |
| Joystick | Analog 2-axis | 1 | Manual control |
| Potentiometer | 10kΩ | 1 | Speed adjustment |

---

### Control System

**Manual Control:**
- Potentiometer: Motor PWM (1000-2000µs range)
- Joystick X-axis: Tail servo angle (30-150°)

**Autonomous Stabilization:**
- MPU6050 reads pitch angle
- Complementary filter (α=0.98): Combines gyro + accelerometer
- Servo correction: Opposes pitch deviation (2× multiplier)
- Mixed control: 70% manual + 30% feedback

#### Circuit Diagrams

##### Transmitter Circuit

<p align="center">
  <img width="567" height="284" alt="Circuit Diagram of Joystick Controller" src="https://github.com/user-attachments/assets/7c228ba1-b534-427a-8ec9-77dcebd14a7b" />
  <br>
  <em> Circuit Diagram of Joystick Controller </em>

##### Receiver Circuit

<p align="center">
  <img width="557" height="374" alt="Circuit Diagram of BLCD Motor integration on Arduino" src="https://github.com/user-attachments/assets/c18c936a-3683-4933-8008-7dfd48e22d60" />
  <br>
  <em> Circuit Diagram of BLCD Motor integration on Arduino </em>

Adapted from A. Raj (2018)

<p align="center">
  <img width="198" height="111" alt="Circuit Diagram of MPU, tail Feedback system" src="https://github.com/user-attachments/assets/6584123d-b3ef-4e1e-8b5f-943d729ecdf1" />
  <br>
  <em> Circuit Diagram of MPU, tail Feedback system </em>

---

## 6. Software

### Control Code

#### Transmitter (Remote Controller)
```cpp
// Key Features:
// - Reads potentiometer (motor speed) and joystick (tail control)
// - Maps analog values to PWM ranges
// - Transmits via RF module at 2000bps
```

```cpp
// --- Program for the Wireless Transmitter / Controller ---

#include <RH_ASK.h>  // RadioHead ASK library for RF communication
#include <SPI.h>     // Required even if not directly used

// --- RF Module Setup ---
// RH_ASK(bitrate, rxPin, txPin, pttPin)
RH_ASK rf_driver(2000, 11, 12, 0);

// --- Input Pins ---
const int PotPin = A1;       // Potentiometer connected to analog pin A1
const int JoystickPin = A0;  // Joystick axis (e.g., horizontal) connected to A0

// --- Data Structure for Transmission ---
// Groups potentiometer and joystick values into a single packet
struct ControlData {
  uint16_t potValue;  // Motor PWM value (mapped 1000–2000 for ESC)
  uint8_t joyValue;   // Servo angle (mapped 30–150 for tail control)
};

void setup() {
  Serial.begin(9600);  // Start Serial for debugging output

  // Initialize RF transmitter
  if (!rf_driver.init()) {
    Serial.println("Transmitter initialization failed!");
  } else {
    Serial.println("Transmitter ready");
  }
}

void loop() {
  // --- Read Raw Analog Inputs ---
  int raw_JoystickValue = analogRead(JoystickPin);  // 0–1023
  int raw_PotValue = analogRead(PotPin);            // 0–1023

  // --- Map Inputs to Application Ranges ---
  // Convert joystick input to an angle between 30° and 150°
  uint8_t JoystickValue = map(raw_JoystickValue, 0, 1023, 30, 150);

  // Convert potentiometer input to PWM pulse width between 1000–2000μs
  uint16_t PotValue = map(raw_PotValue, 0, 1023, 1000, 2000);

  // --- Create a Control Packet ---
  ControlData data = { PotValue, JoystickValue };

  // --- Transmit the Data Packet via RF ---
  rf_driver.send((uint8_t *)&data, sizeof(data));  // Send binary data
  rf_driver.waitPacketSent();                      // Ensure data is fully sent

  // --- Debug Output to Serial Monitor ---
  Serial.print("Sent -> Motor PWM: ");
  Serial.print(PotValue);
  Serial.print(" | Tail Servo: ");
  Serial.println(JoystickValue);

  delay(50);  // Optional small delay to control transmission rate
}
```

**Control Flow Diagram:**
```
┌─────────────────────────────────────────────┐
│          RF Transmitter Loop                │
├─────────────────────────────────────────────┤
│                                             │
│  ┌──────────────┐      ┌──────────────┐   │
│  │ Read         │      │ Read         │   │
│  │ Potentiometer│      │ Joystick     │   │
│  │ (A1)         │      │ (A0)         │   │
│  │ 0-1023       │      │ 0-1023       │   │
│  └──────┬───────┘      └──────┬───────┘   │
│         │                     │            │
│         ▼                     ▼            │
│  ┌──────────────┐      ┌──────────────┐   │
│  │ Map to       │      │ Map to       │   │
│  │ 1000-2000μs  │      │ 30-150°      │   │
│  └──────┬───────┘      └──────┬───────┘   │
│         │                     │            │
│         └──────────┬──────────┘            │
│                    ▼                       │
│           ┌────────────────┐               │
│           │ Pack into      │               │
│           │ ControlData    │               │
│           │ struct         │               │
│           └────────┬───────┘               │
│                    ▼                       │
│           ┌────────────────┐               │
│           │ RF Transmit    │               │
│           │ (433MHz)       │               │
│           │ 2000bps        │               │
│           └────────────────┘               │
│                                             │
└─────────────────────────────────────────────┘
```

**Pin Configuration:**

| Component | Pin | Type | Range |
|-----------|-----|------|-------|
| Potentiometer | A1 | Analog Input | 0-1023 (raw) → 1000-2000µs (PWM) |
| Joystick (X-axis) | A0 | Analog Input | 0-1023 (raw) → 30-150° (servo) |
| RF TX | D12 | Digital Output | 433MHz @ 2000bps |
| RF RX | D11 | Digital Input | (unused on transmitter) |

**Dependencies:**
```cpp
RadioHead Library (RH_ASK) - Install via Arduino Library Manager
```

**Usage:**
1. Connect potentiometer to A1 with 10kΩ pull-down
2. Connect joystick X-axis to A0
3. Connect 433MHz RF transmitter module to D12
4. Power Arduino with USB or 9V battery
5. Open Serial Monitor (9600 baud) to view transmitted values

---

## 7. CFD Analysis

### Simulation Setup

**Objectives:**
- Validate aerodynamic performance
- Analyze flow separation and vortex formation
- Optimize angle of attack for maximum lift

**Parameters:**
- Reynolds Number: Re ≈ 2.3×10⁴
- Inflow Velocity: U = 4 m/s
- Mean Chord: L = 0.085 m
- Air Density: ρ = 1.225 kg/m³
- Dynamic Viscosity: μ = 1.81×10⁻⁵ kg/(m·s)

**Mesh Configuration:**
- Boundary Layer Thickness: δ ≈ 2.8 mm
- First Layer Height: h₁ ≈ 0.108 mm
- Number of Prism Layers: 10
- Growth Rate: r = 1.2

**Time Stepping:**
- Flapping Frequency: f = 0.57 Hz
- Time Step: Δt = 2.125×10⁻³ s
- Steps per Cycle: ~825

### Results

**Key Findings:**
- **Peak Lift**: Initial transient spike due to flow establishment
- **Average Lift**: 0.51 N (lower than peak, indicates stall approach)
- **Flow Behavior**: Leading-edge vortex formation, periodic oscillations
- **Conclusion**: High angle of attack leads to separation; flexible wings needed

<p align="center">
  <img width="1701" height="607" alt="Mesh Layout and Aerodynamic Force Distribution of Flapping-Wing Simulation" src="https://github.com/user-attachments/assets/7905de34-1746-476c-a512-be234e4a6688" />
  <br>
  <em> Mesh Layout and Aerodynamic Force Distribution of Flapping-Wing Simulation </em>

<p align="center">
  <img width="1547" height="330" alt="Pressure Field, Velocity Field, and Aerodynamics" src="https://github.com/user-attachments/assets/d9b00036-1429-4547-b5c0-26b34ab4a695" />
  <br>
  <em> Pressure Field, Velocity Field and Aerodynamics </em>

---

## 8. Testing & Results

### Lift Generation Tests

**Test Setup:**
- Spring scale (inverted mounting)
- PLA stabilization frame
- Incremental RPM testing (80 → 204 RPM)

**Results:**
- **Maximum Lift**: 0.7 N @ 204 RPM (Test 1)
- **Relationship**: Quadratic correlation between lift and RPM
- **Challenges**: Vibration at high speeds, material fatigue

<p align="center">
  <img width="269" height="149" alt="Initial testing rig" src="https://github.com/user-attachments/assets/00dc9c70-5d54-4379-a8ae-08381f36674b" />
  <br>
  <em> Pressure Field, Velocity Field and Aerodynamics </em>

<p align="center">
  <img width="305" height="112" alt="Frame to stabilise robot during testing" src="https://github.com/user-attachments/assets/fffa1931-d5e7-4d57-b705-891a252dad8f" />
  <br>
  <em> Frame to stabilise robot during testing </em

<p align="center">
  <img width="1299" height="583" alt="Conceptualized Approach" src="https://github.com/user-attachments/assets/10332b74-eb14-4e65-9ee2-528e623e46d7" />
  <br>
  <em> (a) Conceptualized 2nd Test Approach, (b) In practice Test Rig/Approach, (c) Spring Scale Representation </em

<p align="center">
  **Lift vs. RPM Curve**
  <img width="343" height="209" alt="(Left) Lift vs  RPM Curve" src="https://github.com/user-attachments/assets/18791ab8-37ac-4c6e-b86f-a41641bb3a61" />
  <br>
</p>

<p align="center">
  **Comparative results of Test 1 and Test 2**
  <img width="345" height="212" alt="(Right) Comparative results of Test 1 and Test 2" src="https://github.com/user-attachments/assets/ccea99aa-524d-4bb7-935c-9adb90479094" />
  <br>
</p>

---

### Gliding Tests

**Methodology:**
- Electronics removed for weight reduction
- Launch angle: 30-45°
- Tail angle optimization: 15-20°
- Distance measurement: 3-4 meters

**Observations:**
- **Critical Factors**: Launch angle and tail trim
- **Front-Heavy Issue**: Requires precise pitch control
- **Optimal Configuration**: 35° launch + 17° tail angle

---

### Performance Metrics

| Metric | Target | Achieved | Status |
|--------|--------|----------|--------|
| Wingspan | 1000mm | 1020mm | ✅ |
| Total Weight | <200g | 248g | ⚠️ |
| Lift Force | >0.5N | 0.7N | ✅ |
| Glide Distance | >3m | 3-4m | ✅ |
| Flapping Rate | 6Hz | 3.4Hz (204RPM) | ⚠️ |
| Flight Duration | 30s | - | ❌ |

**Conclusion:**
- Lift generation successful
- Aerodynamics validated
- Weight optimization needed
- Sustained flight requires material upgrades

---

## 9. Installation & Setup

### Prerequisites
- Arduino IDE 1.8.19+
- Fusion 360 (for CAD files)
- ANSYS (for CFD analysis)

### Hardware Assembly

1. **3D Print Components**
```bash
   # Print settings:
   - Material: PLA
   - Infill: 100% (flapping mechanism)
   - Layer Height: 0.2mm
   - Supports: Yes (gear train)
```

2. **Wing Construction**
   - Cut steel rods to dimensions (Table in docs/)
   - Form U-shapes for pivots
   - Attach nylon membrane
   - See: `docs/assembly_guide.md`

3. **Electronics Integration**
   - Follow circuit diagrams in `hardware/schematics/`
   - Solder connections on perfboard
   - Mount components in body frame```

### Calibration

1. **ESC Calibration**
```
   1. Set potentiometer to maximum
   2. Power on ESC
   3. Wait for beep sequence
   4. Set potentiometer to minimum
   5. Wait for confirmation beeps
```

2. **Servo Centering**
```cpp
   // Run servo_calibration.ino
   // Adjust mechanical linkages to achieve:
   - Tail centered at 90°
   - Full range: 30° to 150°
```

3. **MPU6050 Offset**
```cpp
   // Run mpu_calibration.ino
   // Place robot on flat surface
   // Record offset values
```

---

## 10. Future Work

### Next-Generation Design

**Proposed Improvements:**

1. **Foldable Wings**
   - Four-bar linkage mechanism
   - Reduces drag on upstroke by 40%
   - Increases net lift per cycle

2. **Elliptical Gears**
   - Variable gear ratio per cycle
   - Faster upstroke, slower downstroke
   - Improved motor efficiency

3. **Independent Roll Control**
   - Lateral wing frame sliding
   - Differential lift generation
   - Enhanced maneuverability

4. **Material Upgrades**
   - Carbon fiber wing frame (−60% weight)
   - Machined metal gears (−95% backlash)
   - ABS mechanism components (+200% heat resistance)

5. **Advanced Control**
   - PID controller implementation
   - Setpoint regulation for hovering
   - Trajectory tracking for autonomous flight
  
<p align="center">
  <img width="808" height="416" alt="Feedback control Using PID" src="https://github.com/user-attachments/assets/000ab7e5-8c98-4427-8e43-9a0f76e83eaf" />
  <br>
  <em> Feedback control Using PID </em

---

## 11. Bill of Materials

### Main Prototype (£206 total)

<details>
<summary><b>Control Electronics (£43)</b></summary>

| Item | Price | Qty | Total | Weight |
|------|-------|-----|-------|--------|
| BLDC Motor* | £14 | 1 | £14 | 97g |
| ESC* | £10 | 1 | £10 | - |
| 2S Battery* | £6 | 1 | £6 | - |
| Micro Servos | £5 | 2 | £10 | - |
| MPU6050* | £3 | 1 | £3 | - |

</details>

<details>
<summary><b>Body Components (£31)</b></summary>

| Item | Price | Qty | Total | Weight |
|------|-------|-----|-------|--------|
| PLA Filament | £14 | 1 | £14 | 77g |
| Bearings* | £2 | 6 | £12 | - |
| Steel Shafts | £5 | 1 | £5 | - |

</details>

<details>
<summary><b>Wings & Tail (£10)</b></summary>

| Item | Price | Qty | Total | Weight |
|------|-------|-----|-------|--------|
| Nylon Sheet | £9 | 1 | £9 | 74g |
| PLA Filament | £14 | 0.1 | £1 | - |
| Steel Rods | N/A | 2 | N/A | - |

</details>

*Components marked with * were sourced from personal inventory or previous projects

---

## 12. Sustainability & Impact

### Life Cycle Assessment

**Environmental Impact:** 327.7 kg CO₂ eq

| Component | Impact (kg CO₂ eq) | Percentage |
|-----------|-------------------|------------|
| Battery | 231.6 | 71% |
| Wings (Steel + Nylon) | 52.5 | 16% |
| Body (PLA) | 17.9 | 5% |
| Other | 25.7 | 8% |

**Mitigation Strategies:**
- Solar panel integration (under development)
- Recyclable material selection
- Long operational lifespan design

### SDG Alignment

**UN Sustainable Development Goal 15: Life on Land**

This robot supports:
- Wildlife habitat monitoring
- Anti-poaching surveillance
- Biodiversity data collection
- Ecosystem health assessment

---

## 13. Team

**Department of Engineering, King's College London**

- Imranur Ahmed
- Abdullah Bhuiyan Begum
- Jed Gawan
- Dev Ajay Ateya
- Mohamed Mohamed | k24113059@kcl.ac.uk | Control Software |
- Tan Guo
- Nii Tettey 

**Supervisors:**
- Dr. Juan Li
- Dr. Francesco Ciriello

---

## 14. Acknowledgments

- **Festo Bionic Learning Network** - Inspiration from BionicSwift and BionicFlyingFox
- **KCL Maker Space** - Fabrication facilities and equipment
- **Ornithopter.org** - Flapping mechanism design resources

---

## 15. License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Citation

If you use this work in your research, please cite:
```bibtex
@misc{robin2025,
  author = {Ahmed, I. and Begum, A.B. and Gawan, J. and Masone, G.D. and 
            Ateya, D.A. and Mohamed, M. and Guo, T. and Tettey, N.},
  title = {R.O.B.I.N.: Robot Ornithopter Bio-Inspired by Nature},
  year = {2025},
  publisher = {King's College London},
  howpublished = {\url{https://github.com/Degas01/bio-inspired_ornithopter}},
}
```

---

## 16. References

1. Millner et al. (2023) - "Opportunities and Risks of Aerial Monitoring for Biodiversity Conservation"
2. WWF (2024) - Living Planet Report
3. Macke et al. (2024) - "Drone Noise Impact on Wildlife"
4. Tobalske & Dial (2007) - "Aerodynamics of Wing-Assisted Incline Running"
5. Festo BionicSwift (2018)
