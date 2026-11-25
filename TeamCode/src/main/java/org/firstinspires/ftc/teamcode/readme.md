# BIOBUZZ Team 14363 — Robot Controller Codebase (2026–2027)

Official FTC Java repository for **BIOBUZZ Team 14363** competing in the **2026–2027 FIRST Tech Challenge season (BIOBUZZ)**. This software package is built on the FTC SDK 12.0.0 and utilizes the FTCLib Command-Based framework, Pedro Pathing 3.x motion algorithms, GoBilda Pinpoint dual-pod odometry, Axon MAX servo turret targeting, and Flower mechanism deployment.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Gamepad Controls](#2-gamepad-controls)
3. [OpModes](#3-opmodes)
4. [Pinpoint Odometry Goal Aiming & Pose Storage](#4-pinpoint-odometry-goal-aiming--pose-storage)
5. [Subsystems](#5-subsystems)
6. [Shooter / Outtake Velocity Config](#6-shooter--outtake-velocity-config)
7. [Drive Configuration](#7-drive-configuration)
8. [Library / Dependency Versions](#8-library--dependency-versions)
9. [Hardware Summary Table](#9-hardware-summary-table)
10. [Branch Structure](#10-branch-structure)

---

## 1. Project Overview

* **Team Name:** BIOBUZZ
* **Team Number:** 14363
* **Season:** 2026–2027 FIRST Tech Challenge
* **Game Name:** BIOBUZZ (Target Game Pieces: Pollen & Nectar)

Team 14363 BIOBUZZ operates a competition robot featuring a 4-wheel Mecanum drivetrain controlled via Pedro Pathing 3.x kinematics and GoBilda Pinpoint dual-pod odometry, a proportional analog intake subsystem (goBILDA 1100 RPM intake motor), a single-motor transfer mechanism powered by a goBILDA Yellow Jacket 1150 RPM motor, a single-motor closed-loop PID flywheel shooter (goBILDA Yellow Jacket 6000 RPM) paired with a spring-loaded backboard, a 355° Axon MAX Mark 2 servo turret, a flower mechanism servo (`flowerServo`), and Pinpoint odometry-based goal targeting and turret tracking.

---

## 2. Gamepad Controls

### Gamepad 1 (Single-Driver Controls)

| Button / Trigger | Component / Command | Action Description |
| --- | --- | --- |
| `Left Stick Y` | `DriveSubsystem` | Translation Forward / Backward (negated FTC convention) |
| `Left Stick X` | `DriveSubsystem` | Translation Strafe Left / Right |
| `Right Stick X` | `DriveSubsystem` | Drivetrain Rotation |
| `Left Trigger` (analog > 0.05) | `IntakeSubsystem` | Intake IN (proportional speed directly mapped to analog trigger depth) |
| `Right Trigger` (analog > 0.05) | `IntakeSubsystem` + `TransferSubsystem` | Intake IN + Transfer (held) & Toggles Flower Servo (0° <-> 90°) on press |
| `B Button` | `IntakeSubsystem` + `TransferSubsystem` | Full power Intake IN + Transfer (held) |
| `Left Bumper` | `IntakeOutCommand` | Eject / Reverse Intake + Transfer |
| `Right Bumper` | `VisionTeleOp` | Shooter Flywheel ON / OFF (Toggle) |
| `D-Pad UP` | `RelocalizeCommand` | Relocalize pose at Gardens (Red (0,0), Blue (144,144)) |
| `D-Pad DOWN` | `DriveSubsystem` | IMU / Heading Reset (resets field-centric heading to 90°) |

### Gamepad 2 (Co-Driver Controls)

| Button / Trigger | Component / Command | Action Description |
| --- | --- | --- |
| `A Button` | `VisionTeleOp` | Toggle Turret Mode (Odometry Auto-Tracking <-> Manual Co-Driver Stick Control) |
| `Left Stick X` | `TurretSubsystem` | Manual Turret Rotation input when in Manual Turret Mode |
| `Right Bumper` | `TurretShooterSubsystem` | Increase Shooter Flywheel target velocity (+100 ticks/sec ~ +215 RPM) |
| `Left Bumper` | `TurretShooterSubsystem` | Decrease Shooter Flywheel target velocity (-100 ticks/sec ~ -215 RPM) |

---

## 3. OpModes

| Class Name | Annotation & Group | Type | Description |
| --- | --- | --- | --- |
| `twoTiptwoFlowerRed` | `@Autonomous(name = "twoTiptwoFlowerRed", group = "Autonomous")` | Autonomous | Red Alliance 4-path trajectory. Executes initial load shooting, 2 Flower Servo deployments (0° <-> 90°), load shooting at waypoints, and shoot-on-the-move along Path 4. Saves final pose for TeleOp handoff. |
| `twoTiptwoFlowerBlue` | `@Autonomous(name = "twoTiptwoFlowerBlue", group = "Autonomous")` | Autonomous | Blue Alliance autonomous routine. Mirrored 180° across field midline ($X = 72\text{ in}$) using `PoseFactory.degrees().mirrorX(72.0)` and targeting Blue HIVE goals. Saves final pose for TeleOp. |
| `RedVisionTeleOp` | `@TeleOp(name = "Red Vision TeleOp", group = "Competition")` | TeleOp | Competition TeleOp for Red Alliance featuring automated Pinpoint odometry turret tracking to Red goals and dynamic flywheel velocity. |
| `BlueVisionTeleOp` | `@TeleOp(name = "Blue Vision TeleOp", group = "Competition")` | TeleOp | Competition TeleOp for Blue Alliance featuring automated Pinpoint odometry turret tracking to Blue goals and dynamic flywheel velocity. |
| `BasicFieldCentricTeleOp` | `@TeleOp(name = "Basic Field-Centric Drive", group = "TeleOp")` | TeleOp | Lightweight field-centric drive test OpMode without command scheduler overhead. |
| `VisionTeleOp` | Abstract Base Class | TeleOp Base | Base class extending `CommandOpMode` providing automated turret goal tracking and seamless Autonomous-to-TeleOp pose handoffs. |

---

## 4. Pinpoint Odometry Goal Aiming & Pose Storage

| Component / Class | Mechanism / Sensor | Capabilities | OpModes Used In | Field Goals & Parameters |
| --- | --- | --- | --- | --- |
| `VisionSubsystem` & `HiveGoalTracker` | GoBilda Pinpoint Odometry (`DriveSubsystem` / `Follower`) | Real-time robot pose estimation, active goal selection with 1.5" hysteresis, Euclidean distance calculation, and field-relative turret auto-aiming. | `RedVisionTeleOp`, `BlueVisionTeleOp`, `twoTiptwoFlowerRed`, `twoTiptwoFlowerBlue` | Red Goals: (60in, 60in) & (84in, 60in)<br>Blue Goals: (60in, 84in) & (84in, 84in) |
| `PoseStorage` (`util.PoseStorage`) | Static Pose Container | Stores `public static Pose currentPose` saved at the end of Autonomous for seamless handoff to TeleOp. | All Autonomous & TeleOp OpModes | Fallback to default starting pose if Autonomous is skipped |

---

## 5. Subsystems

| Subsystem Class | Hardware Controlled (Config Name) | Key Behaviors & Features |
| --- | --- | --- |
| `DriveSubsystem` | `leftFront`, `rightFront`, `leftRear`, `rightRear` (`DcMotorEx`), `"pinpoint"` (`GoBildaPinpointDriver`) | Field-centric and robot-centric Mecanum kinematics, heading normalization, pose estimation, and heading reset. |
| `FlowerSubsystem` | `flowerServo` (`Servo`, `"flowerServo"`) | Toggles position between 0° (0.0, CW retracted) and 90° (0.5, CCW deployed). |
| `IntakeSubsystem` | `intakeMotor` (`DcMotorEx`, goBILDA 1100 RPM, `"intakeMotor"`) | Inward/outward cargo intaking driven proportionally by analog trigger pressure (`0.0` to `1.0`). |
| `TransferSubsystem` | `transferMotor` (`DcMotorEx`, goBILDA 1150 RPM, `"transferMotor"`) | Proportional feeding and timed transfer sequence (`TRANSFERRING`, `COOLDOWN`) running motor at full power (`1.0`) for 500 ms. |
| `TurretShooterSubsystem` | `shooterMotor` (`DcMotorEx`, goBILDA 6000 RPM, `"shooterMotor"`) | Closed-loop PID + Feedforward flywheel velocity control (`IDLE`, `SPINNING_UP`, `AT_SPEED`), dynamic distance-scaled velocity, and encoder telemetry. |
| `TurretSubsystem` | `turretServo` (`Servo`, Axon MAX Mark 2, `"turretServo"`) | Position-controlled turret rotation mechanism (0° to 200° angle sweep, 100° center forward, expanded 500µs–2500µs PWM range). |
| `VisionSubsystem` | `HiveGoalTracker`, Pinpoint Odometry | Dynamic goal coordinate resolution, active goal hysteresis, distance-scaled launch velocity, and odometry turret angle calculation. |

---

## 6. Shooter / Outtake Velocity Config

| Parameter | Value | Specification / Description |
| --- | --- | --- |
| Motor Model | goBILDA Yellow Jacket 6000 RPM (SKU 5202-0002-0051) | Single flywheel motor with built-in quadrature hall-effect encoder |
| Encoder Resolution | 28.0 CPR | Ticks per revolution at the output shaft |
| Max Motor Speed | 6000.0 RPM / 2800.0 Ticks/sec | Rated maximum motor velocity |
| Short Range Velocity | 2025.0 Ticks/sec (~1982 RPM) | Standard target velocity for close outtake shots |
| Long Range Velocity | 2800.0 Ticks/sec (~2304 RPM) | Standard target velocity for long outtake shots |
| Target RPM Tolerance | ±50.0 RPM | Allowable error range for `AT_SPEED` state classification |
| Proportional Gain (`kP`) | `0.001` | Flywheel velocity PID proportional term |
| Integral Gain (`kI`) | `0.001` | Flywheel velocity PID integral term |
| Derivative Gain (`kD`) | `0.00001` | Flywheel velocity PID derivative term |
| Anti-Windup Limit (`maxI`) | `0.3` | Integral term clamping limit |
| Static Friction (`kS`) | `0.05` | Feedforward static friction offset |
| Velocity Constant (`kV`) | `0.00035` | Feedforward velocity multiplier |
| Acceleration Constant (`kA`) | `0.0` | Feedforward acceleration gain |

---

## 7. Drive Configuration

| Parameter | Configuration Value | Details |
| --- | --- | --- |
| Drivetrain Type | Mecanum Drivetrain | 4 independent Mecanum wheels for omnidirectional translation |
| Drivetrain Motors | goBILDA Yellow Jacket 425 RPM (`DcMotorEx`) | `leftFront` (REVERSE), `leftRear` (REVERSE), `rightFront` (FORWARD), `rightRear` (FORWARD) |
| Brake Mode | `DcMotor.ZeroPowerBehavior.BRAKE` | Motors set to `BRAKE` when zero power is applied |
| Localizer Hardware | GoBilda Pinpoint Driver (`"pinpoint"`) | Hardware odometry localizer with 2-wheel 4-bar odometry pods |
| Pinpoint Pod Offsets | X-Pod = +7.34 in, Y-Pod = +2.64 in | Physical pod offset coordinates relative to robot center |
| Pod Encoder Direction | `FORWARD` (X-Pod & Y-Pod) | Encoder rotation orientation |
| Motion Engine | Pedro Pathing 3.0.1 | Motion algorithm using `Follower`, `Mecanum`, and `Foresight` |

---

## 8. Library / Dependency Versions

| Library / Framework | Dependency Group & Artifact | Version | Notes |
| --- | --- | --- | --- |
| FTC SDK | `org.firstinspires.ftc:*` | `12.0.0` | Official FIRST Tech Challenge SDK |
| FTCLib Core | `org.ftclib.ftclib:core` | `2.1.1` | Command-based framework for FTC |
| Pedro Pathing Core | `com.pedropathing:core` | `3.0.1` | Advanced pathing & motion algorithm |
| Pedro Pathing REV Hub | `com.pedropathing:revhub` | `3.0.1` | Drivetrain & localizer integrations |
| AndroidX AppCompat | `androidx.appcompat:appcompat` | `1.2.0` | Android UI compatibility library |
| Android Gradle Plugin | `com.android.application` | `8.13.2` | AGP build system engine |

---

## 9. Hardware Summary Table

| Device Name (HardwareMap Config) | Device Type / Class | Subsystem / Function |
| --- | --- | --- |
| `"leftFront"` | `DcMotorEx` (goBILDA 425 RPM) | `DriveSubsystem` — Drivetrain Front Left |
| `"leftRear"` | `DcMotorEx` (goBILDA 425 RPM) | `DriveSubsystem` — Drivetrain Rear Left |
| `"rightFront"` | `DcMotorEx` (goBILDA 425 RPM) | `DriveSubsystem` — Drivetrain Front Right |
| `"rightRear"` | `DcMotorEx` (goBILDA 425 RPM) | `DriveSubsystem` — Drivetrain Rear Right |
| `"pinpoint"` | `GoBildaPinpointDriver` | `DriveSubsystem` — GoBilda Pinpoint Odometry Localizer |
| `"intakeMotor"` | `DcMotorEx` (goBILDA 1100 RPM) | `IntakeSubsystem` — Intake Motor |
| `"transferMotor"` | `DcMotorEx` (goBILDA 1150 RPM) | `TransferSubsystem` & `IntakeSubsystem` — Transfer Mechanism |
| `"shooterMotor1"` / `"shooterMotor"` | `DcMotorEx` (goBILDA 6000 RPM) | `TurretShooterSubsystem` — Dual Flywheel Primary Shooter Motor |
| `"shooterMotor2"` | `DcMotorEx` (goBILDA 6000 RPM) | `TurretShooterSubsystem` — Dual Flywheel Secondary Shooter Motor |
| `"turretServo"` | `Servo` (Axon MAX Mark 2) | `TurretSubsystem` — Rotating Turret Mechanism |
| `"flowerServo"` | `Servo` | `FlowerSubsystem` — Flower Servo Mechanism (0° <-> 90°) |

---

## 10. Branch Structure

- **`master`**: Primary production branch containing verified competition code.
- **`odod-turret-detached`** (Active Branch): Development branch combining Pinpoint odometry localizer, turret subsystem tracking, and detached architecture.
- **`moving-turret`**: Feature branch for dynamic on-the-fly turret goal tracking while driving.
- **`straight-transfer`**: Subsystem branch for the straight-line single-motor transfer mechanism.
- **`detached`**: Experimental branch isolating turret and vision subsystems from drivetrain dependencies.
- **`combine`**: Integration branch used to synthesize parallel subsystem feature branches.

---
