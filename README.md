# Autonomous Ground Vehicle (AGV)
### ETE-4851 | Four-Wheel-Drive Autonomous Robotics Project

## Project Overview

This repository contains the mechanical and electrical documentation, embedded firmware, ROS 2 software, simulation, and testing procedures for our autonomous ground vehicle (AGV).

The vehicle is designed for outdoor robotics competition on grass and pavement. It features a four-wheel-drive drivetrain, front and rear Ackermann-style steering, encoder-equipped DC gearmotors, and a distributed control architecture using a Raspberry Pi 5 and STM32 microcontroller.

### Project Objectives

- Develop reliable manual control using keyboard or gamepad teleoperation.
- Implement closed-loop drivetrain speed control using motor encoders.
- Support front and rear steering for improved maneuverability.
- Integrate LiDAR and RGB cameras for environmental perception.
- Detect obstacles, colored targets, and ground markings.
- Implement autonomous navigation and competition-specific behaviors.
- Establish safe, repeatable hardware and software testing procedures.

Development will proceed from basic actuator control to fully autonomous operation.

---

## 1. System Architecture

The AGV uses a hierarchical control architecture that separates high-level computation from real-time actuator control.

### High-Level Controller — Raspberry Pi 5

Responsible for:

- ROS 2 communication and coordination
- Keyboard and gamepad teleoperation
- Autonomous navigation and decision-making
- Camera and LiDAR processing
- Obstacle and target detection
- Communication with the STM32
- System monitoring and logging

### Low-Level Controller — STM32 NUCLEO-F767ZI

Responsible for:

- Independent front and rear motor control
- Quadrature encoder acquisition
- Closed-loop motor speed regulation
- Front and rear steering servo PWM generation
- Command validation and actuator limits
- Communication watchdog and safety monitoring
- Reporting drivetrain and controller status

### System Block Diagram

```text
             Raspberry Pi 5
            High-Level Control
                    |
             USB Serial / UART
                    |
           STM32 NUCLEO-F767ZI
                    |
       +------------+------------+
       |            |            |
  Motor Control  Servo Control  Feedback
       |            |            |
  +----+----+   +---+---+   +----+----+
  |         |   |       |   |         |
BTS7960   BTS7960 Front   Rear  Front/Rear
  #1        #2   Servo   Servo  Encoders
  |         |
Front      Rear
37D Motor  37D Motor
  |         |
Front      Rear
Differential Differential
  |         |
Front      Rear
Wheels     Wheels
```

The Raspberry Pi sends high-level motion commands while the STM32 handles real-time actuator control.

---

## 2. Hardware Configuration

| Component | Model | Quantity | Function |
|---|---|---|---|
| Main Computer | Raspberry Pi 5 | 1 | High-level processing |
| Microcontroller | STM32 NUCLEO-F767ZI | 1 | Real-time control |
| Drive Motors | Pololu 37D 24V 50:1 with Encoders | 2 | Front/rear axle propulsion |
| Motor Drivers | BTS7960 H-Bridge Modules | 2 | Independent DC motor control |
| Steering Servos | Powerhobby 190MG 35KG HV | 2 | Front/rear steering |
| Drive Axles | RC Crawler Differential Axles | 2 | Four-wheel drivetrain |
| LiDAR | SLAMTEC RPLIDAR C1M1 | 1 | Obstacle detection |
| Cameras | NexiGo N60 1080P | 2 | Vision and target detection. 1 Front 1 Rear |
| IMU | Adafruit BNO055 | 1 | Orientation estimation, Turning Rate Sense |

### Drivetrain Configuration

The AGV uses two independently powered differential axles:

- One Pololu 37D gearmotor drives the front axle.
- One Pololu 37D gearmotor drives the rear axle.
- Each differential distributes torque between its left and right wheels.
- Both motors are independently controlled through BTS7960 motor drivers.
- Four wheels are driven, but left and right wheel speeds are not independently commanded.

Unlike a differential-drive robot, vehicle steering is achieved through mechanical steering linkages rather than differential left/right motor speeds.

---

## 3. Drive Motor Specifications

### Pololu 37D 24V 50:1 Metal Gearmotor with Encoder

**Quantity:** 2

**Application:**
- Motor 1: Front drivetrain
- Motor 2: Rear drivetrain

### Manufacturer Specifications

| Parameter | Specification |
|---|---|
| Manufacturer | Pololu |
| Motor Type | Brushed DC |
| Rated Voltage | 24 V |
| Gear Ratio | 50:1 |
| No-Load Output Speed | 200 RPM |
| No-Load Current | 0.10 A |
| Speed at Maximum Efficiency | 170 RPM |
| Current at Maximum Efficiency | 0.41 A |
| Output Torque at Maximum Efficiency | 27 kg·mm |
| Stall Current (Extrapolated) | 3.0 A |
| Stall Torque (Extrapolated) | 230 kg·mm |
| Encoder Type | Hall-effect quadrature |
| Encoder Resolution | 64 counts/motor revolution (x4 decoding) |
| Nominal Gearbox Output Resolution | 3,200 counts/revolution (x4 decoding) |
| Encoder Supply | 3.5–20 V |

**Pololu part number:** #4693 (encoder-equipped 24V 50:1 version).

Manufacturer reference: Pololu 37D Metal Gearmotors, Rev. 1.2, pages 1, 3–4, and 31–32.

### Encoder Configuration

Each motor includes an integrated two-channel Hall-effect quadrature encoder.

Encoder wiring:

| Wire Color | Function |
|---|---|
| Red | Motor Power |
| Black | Motor Power |
| Green | Encoder Ground |
| Blue | Encoder VCC |
| Yellow | Encoder Channel A |
| White | Encoder Channel B |

The encoder produces 64 counts per motor-shaft revolution when counting both edges of both channels.

Nominal gearbox output resolution:

```text
Counts / Output Revolution = 64 × 50

Counts / Output Revolution = 3200
```

Encoder feedback will be used for:

1. Motor speed measurement
2. Direction verification
3. Independent front/rear PID speed control
4. Drivetrain monitoring
5. Approximate distance estimation and odometry

Encoder outputs operate at the supplied encoder voltage. STM32 input-voltage compatibility must be confirmed before connecting encoder outputs to GPIO pins.

Motor encoder measurements must also account for the additional differential-axle gear reduction when estimating wheel rotation.

---

## 4. Steering System

### Powerhobby 190MG 35KG HV Servo

**Quantity:** 2

- Servo 1: Front steering linkage
- Servo 2: Rear steering linkage

Each servo actuates a mechanical steering linkage connected to the left and right wheels on its axle.

### Manufacturer Specifications

| Parameter | Specification |
|---|---|
| Manufacturer | Powerhobby |
| Model | 190MG |
| Type | Digital Standard Servo |
| Rated Stall Torque at 6.0V | 22 kg·cm |
| Rated Stall Torque at 7.4V | 35 kg·cm |
| Rated Stall Torque at 8.4V | 40 kg·cm |
| Speed at 6.0V | 0.25 sec/60° |
| Speed at 7.4V | 0.22 sec/60° |
| Speed at 8.4V | 0.19 sec/60° |
| Dimensions | 40 × 20 × 40.5 mm |
| Gears | Steel |
| Housing | Aluminum Center Case |
| Bearings | Dual Ball Bearing |
| Horn Interface | 25T |
| Protection | Waterproof (manufacturer designation) |

### Four-Wheel Steering Modes

**Mode 1: Counter-Phase Steering**

The front and rear wheels turn in opposite directions.

Applications:
- Tight-radius turning
- Obstacle maneuvering
- General vehicle navigation

This is the primary planned steering mode.

**Mode 2: Crab Steering**

Front and rear wheels steer in the same direction.

Applications:
- Diagonal vehicle movement
- Lateral positioning during maneuvering

**Mode 3: Front-Only Steering (Optional)**

Rear wheels remain centered while front steering is active.

This mode can simplify initial testing and comparisons with conventional Ackermann steering.

### Steering Control

The STM32 generates separate PWM control signals for the two steering servos.

Servo center positions, control pulse limits, mechanical steering angles, and steering direction must be experimentally calibrated.

The commanded servo-shaft angle is not necessarily equal to the resulting wheel steering angle.

---

## 5. Motor Driver Configuration

### BTS7960 H-Bridge Motor Driver

**Quantity:** 2

Each motor uses one independent H-bridge module.

| Driver | Connected Motor |
|---|---|
| BTS7960 #1 | Front Pololu 37D |
| BTS7960 #2 | Rear Pololu 37D |

### Typical Control Signals

| Signal | Description |
|---|---|
| RPWM | Right-direction PWM |
| LPWM | Left-direction PWM |
| R_EN | Right-side enable |
| L_EN | Left-side enable |
| R_IS | Right-side sense/diagnostic output |
| L_IS | Left-side sense/diagnostic output |

The STM32 provides PWM and enable commands.

The BTS7960 modules supply motor current from the drivetrain power system.

### Required Electrical Verification

- Verify the motor-driver module's actual supply-voltage limits.
- Verify compatibility with the selected 24V motor supply.
- Confirm STM32 3.3V logic compatibility.
- Confirm driver logic-supply requirements.
- Verify thermal performance and protection.
- Implement appropriate fusing and physical emergency motor-power isolation.
- Keep all motors disabled during initialization and communication loss.

The advertised peak-current capability of a BTS7960 module must not be treated as its continuous operating-current rating.

---

## 6. Electrical Power System

The AGV uses separate propulsion and electronics power systems.

### Drivetrain Battery

Planned configuration:

- 6S LiPo battery
- 22.2V nominal voltage
- 25.2V maximum fully charged voltage
- Supplies front and rear drivetrain motor drivers

Battery wiring, fusing, motor-driver voltage margins, and motor operating conditions must be verified before powering the drivetrain.

### Electronics Power

A separate regulated electronics power system will supply:

- Raspberry Pi 5
- STM32
- Cameras
- LiDAR
- Supporting electronics

### Steering Servo Power

The two Powerhobby 190MG servos require an appropriately regulated high-current supply operating within their rated voltage range.

Servo power must not be drawn directly from Raspberry Pi or STM32 GPIO or logic-power outputs.

The electrical design must include appropriate common ground references for control signals, protection, and isolation of high-current motor wiring.

---

## 7. Software Architecture

### Raspberry Pi Software

Planned responsibilities:

```text
Raspberry Pi 5
├── ROS 2
├── Teleoperation
│   ├── Keyboard
│   └── Gamepad
├── Perception
│   ├── LiDAR
│   └── RGB Cameras
├── Navigation
├── High-Level Vehicle Control
├── STM32 Communication
└── Logging and Diagnostics
```

### STM32 Firmware

Planned responsibilities:

```text
STM32 NUCLEO-F767ZI
├── Serial Communication
├── Motor Control
│   ├── Front PWM
│   └── Rear PWM
├── Encoder Processing
│   ├── Front Encoder
│   └── Rear Encoder
├── Speed Control
│   ├── Front PID
│   └── Rear PID
├── Steering Control
│   ├── Front Servo
│   └── Rear Servo
└── Safety State Machine
```

### Proposed Communication Protocol

The Raspberry Pi communicates with the STM32 over USB serial or UART.

Proposed command fields:

```text
drive_speed
front_steering_angle
rear_steering_angle
steering_mode
enable_request
```

Proposed feedback fields:

```text
front_motor_rpm
rear_motor_rpm
front_encoder_count
rear_encoder_count
controller_state
fault_status
```

These are preliminary interface definitions, not a finalized serial protocol.

Message framing, units, command timeouts, and error detection must be defined before implementation.

---

## 8. Development Environment

The development workflow uses:

- Visual Studio Code
- Git and GitHub
- OpenAI Codex for AI-assisted development
- Raspberry Pi 5 running Linux
- Raspberry Pi SSH server
- Tailscale for secure remote access
- STM32 development/debugging tools

### Remote Development Workflow

```text
Developer Laptop
      |
VS Code + SSH
      |
Tailscale Network
      |
Raspberry Pi 5
      |
STM32
      |
Motor Drivers / Servos
```

Team members can remotely access the Raspberry Pi to develop, debug, and test software.

GitHub remains the central source repository. Team members should use separate branches and pull requests.

Only authorized operators should enable physical actuator testing.

---

## 9. Repository Structure

```text
ETE-4851-Project/
├── README.md
├── firmware/
│   └── STM32 firmware
├── ros2_ws/
│   └── ROS 2 workspace
├── hardware/
│   └── Electrical and mechanical documentation
├── simulation/
│   └── Vehicle simulation
├── docs/
│   └── Engineering documentation
├── scripts/
│   └── Setup and development utilities
└── tests/
    └── Hardware and software testing
```

The existing repository organization will be preserved where practical.

Previously developed differential-drive software and simulation components require review before reuse with four-wheel steering.

---

## 10. Testing and Development Milestones

### Phase 1 — STM32 and Communication

- [ ] Configure STM32 development environment.
- [ ] Flash and verify STM32 firmware.
- [ ] Establish USB serial/UART communication.
- [ ] Verify encoder input signals.
- [ ] Validate motor PWM generation without powered motors.
- [ ] Validate front/rear servo PWM generation.
- [ ] Implement command limits and watchdog.
- [ ] Validate safe startup, disarm, and fault behavior.

### Phase 2 — Actuator Testing

- [ ] Verify power-system electrical compatibility.
- [ ] Test BTS7960 #1 with front motor.
- [ ] Test BTS7960 #2 with rear motor.
- [ ] Verify motor direction and encoder polarity.
- [ ] Test front steering servo.
- [ ] Test rear steering servo.
- [ ] Calibrate servo center positions and steering limits.
- [ ] Integrate both motors and steering linkages.

### Phase 3 — Teleoperation

- [ ] Configure Raspberry Pi operating system.
- [ ] Configure Tailscale and SSH.
- [ ] Connect Raspberry Pi to STM32.
- [ ] Implement keyboard control.
- [ ] Implement gamepad control if required.
- [ ] Integrate speed and steering commands.
- [ ] Test communication-loss failsafe.
- [ ] Demonstrate safe forward, reverse, and turning operation.

### Phase 4 — Closed-Loop Control

- [ ] Measure front/rear motor RPM.
- [ ] Implement independent PID speed controllers.
- [ ] Calibrate drive control under load.
- [ ] Implement counter-phase steering control.
- [ ] Implement crab steering.
- [ ] Develop steering-aware odometry.
- [ ] Evaluate traction and wheel slip on grass.

### Phase 5 — Autonomous Navigation

- [ ] Integrate 2D LiDAR.
- [ ] Integrate RGB cameras.
- [ ] Implement obstacle detection.
- [ ] Implement color recognition.
- [ ] Implement ground-line detection.
- [ ] Integrate localization and navigation.
- [ ] Test autonomous driving outdoors.
- [ ] Validate competition-specific tasks.

---

## 11. Safety Requirements

The AGV must operate under a defined safety state machine.

Recommended operating states:

```text
DISARMED
   |
   v
ARMED
   |
   v
ACTIVE

FAULT / ESTOP
   |
   v
DISARMED
```

Safety requirements include:

1. Motor outputs disabled on startup.
2. Explicit authorization required before enabling drivetrain power.
3. Independent physical emergency motor-power cutoff.
4. STM32 communication watchdog.
5. Validated and bounded PWM and steering commands.
6. Safe response to stale or malformed commands.
7. Defined fault recovery and rearming procedures.
8. No automatic motor rearming after a reset.
9. Controlled, supervised hardware testing.
10. Separation of remote development permissions from physical vehicle operation.

SSH and Tailscale connectivity are development tools and must not serve as the sole emergency-stop mechanism.

---

## 12. Current Project Status

### Hardware Confirmed

- [x] Raspberry Pi 5 selected as high-level computer
- [x] STM32 NUCLEO-F767ZI selected for real-time control
- [x] Two Pololu 37D 24V 50:1 gearmotors with integrated encoders received
- [x] Two Powerhobby 190MG 35KG steering servos received
- [x] Two BTS7960 motor drivers selected
- [x] Front and rear differential-axle drivetrain architecture
- [x] Front and rear mechanical steering architecture

### Pending Integration

- [ ] STM32 firmware bring-up
- [ ] Motor-driver wiring and verification
- [ ] Encoder signal testing
- [ ] Steering calibration
- [ ] Raspberry Pi-to-STM32 communication
- [ ] Teleoperation software
- [ ] Closed-loop drivetrain control
- [ ] LiDAR and camera integration
- [ ] Autonomous navigation

Completion of a hardware selection does not imply that the corresponding electrical interface or software has been tested.

---

## 13. Technical References

### Drive Motors

Pololu 37D Metal Gearmotors Datasheet, Revision 1.2 (January 2020)

Relevant sections:

- Page 1: Performance summary
- Page 2: Mechanical dimensions
- Pages 3–4: Encoder operation and wiring
- Pages 31–32: 24V 50:1 gearmotor performance

### Steering Servos

Powerhobby 190MG 35KG High-Torque Metal Gear Waterproof HV Servo

Manufacturer product specifications covering operating voltage, torque, speed, dimensions, and mechanical construction.

### Software and Hardware Documentation

- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Tailscale Documentation](https://tailscale.com/docs)
- [Pololu Robotics](https://www.pololu.com/)

---

## Project Goal

Develop a reliable, maneuverable, four-wheel-drive autonomous ground vehicle that progresses from safe manual teleoperation to fully autonomous navigation and competition-specific task execution.

The immediate development priority is STM32 actuator control, encoder feedback, Raspberry Pi communication, and safe vehicle teleoperation.
