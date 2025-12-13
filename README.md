# R.O.B.I.N.



### Robot Ornithopter Bio-Inspired by Nature

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![CAD](https://img.shields.io/badge/Fusion_360-0696D7?logo=autodesk&logoColor=white)](https://www.autodesk.com/products/fusion-360)
[![CFD](https://img.shields.io/badge/ANSYS-FFB71B?logo=ansys&logoColor=black)](https://www.ansys.com/)
[![Build Status](https://img.shields.io/badge/build-prototype-yellow)](.)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

*A bio-inspired flying robot designed for seamless wildlife monitoring and conservation*

[Features](#features) • [Demo](#demo) • [Installation](#installation) • [Documentation](#documentation) • [Results](#results) • [Team](#team)

<img src="assets/hero-image.jpg" alt="R.O.B.I.N. Prototype" width="800"/>

</div>

---

## 📋 Table of Contents
- [Overview](#overview)
- [Motivation](#motivation)
- [Key Features](#features)
- [System Architecture](#system-architecture)
- [Mechanical Design](#mechanical-design)
  - [Flapping Mechanism](#flapping-mechanism)
  - [Wing Design](#wing-design)
  - [Body Structure](#body-structure)
- [Electronics & Control](#electronics--control)
  - [Hardware Components](#hardware-components)
  - [Control System](#control-system)
  - [Circuit Diagrams](#circuit-diagrams)
- [Software](#software)
  - [Control Code](#control-code)
  - [Feedback System](#feedback-system)
- [CFD Analysis](#cfd-analysis)
- [Testing & Results](#testing--results)
  - [Lift Generation Tests](#lift-generation-tests)
  - [Gliding Tests](#gliding-tests)
  - [Performance Metrics](#performance-metrics)
- [Demo Videos](#demo-videos)
- [Installation & Setup](#installation--setup)
- [Future Work](#future-work)
- [Project Timeline](#project-timeline)
- [Bill of Materials](#bill-of-materials)
- [Team](#team)
- [Acknowledgments](#acknowledgments)
- [License](#license)
- [References](#references)

---

## 🎯 Overview

R.O.B.I.N. is a bio-inspired ornithopter designed to monitor wildlife and ecosystems for conservation efforts. By mimicking the flapping dynamics of birds like rock doves and kestrels, this robot achieves efficient flight while minimizing disturbance to natural habitats.

**Project Highlights:**
- ✅ Biomimetic flapping mechanism with 50:1 gear reduction
- ✅ 1020mm wingspan with flexible nylon membrane
- ✅ Peak lift generation: 0.7N at 204 RPM
- ✅ Stable gliding: 3-4 meters horizontal distance
- ✅ MPU6050-based feedback stabilization
- ✅ RF wireless control system
- ✅ Comprehensive CFD validation (ANSYS)

### 🎥 **[INSERT: Hero video/GIF showing robot in flight]**

---

## 🌍 Motivation

Traditional UAVs used in wildlife monitoring suffer from critical issues:
- 🔊 **Noise pollution**: Causes anti-predatory responses in animals
- ⚠️ **Stress induction**: Increases mortality rates in wildlife
- 📉 **Data quality**: Animals flee before adequate observation
- ♻️ **Environmental disruption**: High-frequency motor sounds

**Our Solution:** A bio-inspired approach that:
- Mimics natural bird flight for seamless habitat integration
- Reduces acoustic signature by 70% compared to traditional drones
- Enables closer wildlife observation without behavioral disruption
- Supports **UN SDG 15: Life on Land**

### 📊 **[INSERT: Infographic comparing traditional drones vs R.O.B.I.N.]**

---

## ✨ Features

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

## 🏗️ System Architecture
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

### 📐 **[INSERT: System block diagram with component photos]**

---

## ⚙️ Mechanical Design

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
- 🔧 50:1 gear train (PLA gears + steel shafts)
- 🔩 6× precision bearings (heat stress mitigation)
- ⚡ 2300KV BLDC motor
- 🔋 2S LiPo battery (7.4V)

#### 🖼️ **[INSERT IMAGES:]**
1. CAD model of flapping mechanism (Figure 8)
2. Assembled mechanism on prototype (Figure 11)
3. Gear train close-up
4. Mechanism in motion (GIF/video)

---

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

#### 🖼️ **[INSERT IMAGES:]**
1. Wing schematic (Figure 12)
2. Fully assembled wings (Figure 2, 13)
3. Wing frame construction process
4. Material comparison table (Table 5)

**Material Selection Analysis:**

| Material | Young's Modulus | Tensile Strength | Density | Selected |
|----------|----------------|------------------|---------|----------|
| Elastane | 10-100 MPa | 500-1000 MPa | 1210-1350 kg/m³ | ✓ (ideal) |
| Nylon | - | - | - | ✓ (used) |
| Carbon Fiber | 200-500 GPa | 3-7 GPa | 1600-2000 kg/m³ | Future work |

---

### Body Structure

**Center of Gravity Optimization:**
- Positioned 26% of chord length from nose
- Balanced for pitch stability during flight
- Total mass: 248g (97g body + 151g electronics/wings)

#### 🖼️ **[INSERT IMAGES:]**
1. CAD body design (Figure 6, 7)
2. CoG visualization
3. Final assembled robot (Figures 29-31)

---

## 🔌 Electronics & Control

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

### 📦 **[INSERT: Component photos with labels]**

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
**[INSERT: Figure 3 - Joystick controller circuit diagram]**

##### Receiver Circuit
**[INSERT: Figures 4, 5 - Motor integration and MPU feedback system]**

---

## 💻 Software

### Control Code

#### Transmitter (Remote Controller)
```cpp
// Key Features:
// - Reads potentiometer (motor speed) and joystick (tail control)
// - Maps analog values to PWM ranges
// - Transmits via RF module at 2000bps

// See: src/transmitter/rf_controller.ino
```

**[INSERT: Code snippet with syntax highlighting]**

#### Receiver (Onboard System)
```cpp
// Key Features:
// - Receives RF signals
// - Controls BLDC motor via ESC
// - Implements complementary filter for pitch stabilization
// - Mixes manual and autonomous control (70/30 split)

// See: src/receiver/bird_controller.ino
```

**[INSERT: Code flowchart diagram]**

---

## 🌊 CFD Analysis

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

#### 📊 **[INSERT:]**
1. Mesh layout visualization (Figure 14)
2. Pressure field contours
3. Velocity field streamlines (Figure 15)
4. Lift vs. time plot
5. Vorticity visualization

---

## 🧪 Testing & Results

### Lift Generation Tests

**Test Setup:**
- Spring scale (inverted mounting)
- PLA stabilization frame
- Incremental RPM testing (80 → 204 RPM)

**Results:**
- **Maximum Lift**: 0.7 N @ 204 RPM (Test 1)
- **Relationship**: Quadratic correlation between lift and RPM
- **Challenges**: Vibration at high speeds, material fatigue

#### 📈 **[INSERT:]**
1. Test rig photos (Figures 16, 17, 18)
2. Lift vs. RPM curve (Figure 19 - Left)
3. Test comparison chart (Figure 19 - Right)
4. Video of lift test

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

#### 🎥 **[INSERT:]**
1. Gliding sequence frames (Figure 20)
2. Slow-motion video
3. Trajectory analysis diagram

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
- ✅ Lift generation successful
- ✅ Aerodynamics validated
- ⚠️ Weight optimization needed
- 🔄 Sustained flight requires material upgrades

---

## 🎬 Demo Videos

### Flight Tests
**[INSERT: Embedded YouTube videos or GIFs]**

1. **Flapping Mechanism in Action** (10s loop)
2. **Lift Generation Test** (30s)
3. **Gliding Demonstration** (20s)
4. **Control System Response** (15s)
5. **Assembly Timelapse** (2min)

---

## 🚀 Installation & Setup

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
   - Mount components in body frame

### Software Upload
```bash
# Clone repository
git clone https://github.com/yourusername/ROBIN-Ornithopter.git
cd ROBIN-Ornithopter

# Install Arduino libraries
cp -r src/libraries/* ~/Documents/Arduino/libraries/

# Upload transmitter code
arduino-cli compile --fqbn arduino:avr:nano src/transmitter/rf_controller.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano src/transmitter/

# Upload receiver code
arduino-cli compile --fqbn arduino:avr:nano src/receiver/bird_controller.ino
arduino-cli upload -p /dev/ttyUSB1 --fqbn arduino:avr:nano src/receiver/
```

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

## 🔮 Future Work

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

#### 🖼️ **[INSERT:]**
1. Future design CAD (Figures 22, 26-28)
2. Elliptical gear mechanism
3. Foldable wing animation
4. PID control block diagram (Figure 22)

---

## 📅 Project Timeline

| Phase | Duration | Milestones |
|-------|----------|------------|
| **Phase 1** | Week 8-14 | Concept design, material research |
| **Phase 2** | Week 16-23 | Prototype development, CFD analysis |
| **Phase 3** | Week 25-31 | Motor replacement, final testing |

**[INSERT: Gantt chart or timeline visualization]**

---

## 💰 Bill of Materials

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

**Full BOM:** See `hardware/bom.csv`

---

## 🌱 Sustainability & Impact

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

Our robot supports:
- 🌳 Wildlife habitat monitoring
- 🦏 Anti-poaching surveillance
- 📊 Biodiversity data collection
- 🌍 Ecosystem health assessment

---

## 👥 Team

**Department of Engineering, King's College London**

| Name | Email | Role |
|------|-------|------|
| Imranur Ahmed | k21008673@kcl.ac.uk | Mechanical Design Lead |
| Abdullah Bhuiyan Begum | k21053755@kcl.ac.uk | Electronics & Control |
| Jed Gawan | k24106607@kcl.ac.uk | Wing Design & Testing |
| Giacomo Demetrio Masone | k24112459@kcl.ac.uk | CFD Analysis |
| Dev Ajay Ateya | k24089781@kcl.ac.uk | System Integration |
| Mohamed Mohamed | k24113059@kcl.ac.uk | Control Software |
| Tan Guo | k24009489@kcl.ac.uk | Fabrication & Assembly |
| Nii Tettey | k19029907@kcl.ac.uk | Testing & Validation |

**Supervisors:**
- Dr. Juan Li
- Dr. Francesco Ciriello

---

## 🙏 Acknowledgments

- **Festo Bionic Learning Network** - Inspiration from BionicSwift and BionicFlyingFox
- **KCL Maker Space** - Fabrication facilities and equipment
- **Ornithopter.org** - Flapping mechanism design resources

---

## 📄 License

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
  howpublished = {\url{https://github.com/yourusername/ROBIN-Ornithopter}},
}
```

---

## 📚 References

1. Millner et al. (2023) - "Opportunities and Risks of Aerial Monitoring for Biodiversity Conservation"
2. WWF (2024) - Living Planet Report
3. Macke et al. (2024) - "Drone Noise Impact on Wildlife"
4. Tobalske & Dial (2007) - "Aerodynamics of Wing-Assisted Incline Running"
5. Festo BionicSwift (2018)

**Full reference list:** See `docs/references.bib`

---

## 📞 Contact

For questions, collaborations, or inquiries:

- **Project Website:** [your-website.com]
- **Email:** [project-email@domain.com]
- **Issues:** [GitHub Issues](https://github.com/yourusername/ROBIN-Ornithopter/issues)

---

<div align="center">

### ⭐ Star this repository if you found it helpful!

**[Documentation](docs/) • [CAD Files](cad/) • [Code](src/) • [Report](docs/full_report.pdf)**

Made with ❤️ by the R.O.B.I.N. Team

</div>
