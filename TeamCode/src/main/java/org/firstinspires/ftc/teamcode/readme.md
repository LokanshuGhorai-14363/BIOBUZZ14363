# BIOBUZZ Team 14363 — Robot Controller Codebase (2026–2027)

Official FTC Java repository for **BIOBUZZ Team 14363** competing in the **2026–2027 FIRST Tech Challenge season (BIOBUZZ)**. This software package is built on the FTC SDK 12.0.0 and utilizes the FTCLib Command-Based framework, Pedro Pathing 3.x motion algorithms, GoBilda Pinpoint odometry, and Limelight 3A vision tracking.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Gamepad Controls](#2-gamepad-controls)
3. [OpModes](#3-opmodes)
4. [Vision Pipelines](#4-vision-pipelines)
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

Team 14363 BIOBUZZ operates a competition robot featuring a 4-wheel Mecanum drivetrain controlled via Pedro Pathing 3.x kinematics and GoBilda Pinpoint dual-pod odometry, a proportional analog intake subsystem (goBILDA 1100 RPM intake motor), a single-motor transfer mechanism powered by a goBILDA Yellow Jacket 1150 RPM motor, a single-motor closed-loop PID flywheel shooter (goBILDA Yellow Jacket 6000 RPM) paired with a spring-loaded backboard, a 355° Axon MAX Mark 2 servo turret, and a Limelight 3A vision unit delivering real-time goal tracking and PedroPathing relocalization.

---

## 2. Gamepad Controls

### Gamepad 1 (Driver Controls)

| Button / Trigger | Component / Command | Action Description |
| --- | --- | --- |
| `Left Stick Y` | `DriveSubsystem` | Translation Forward / Backward (negated FTC convention) |
| `Left Stick X` | `DriveSubsystem` | Translation Strafe Left / Right |
| `Right Stick X` | `DriveSubsystem` | Drivetrain Rotation |
| `Left Trigger` (analog > 0.05) | `IntakeInCommand` | Intake IN + Transfer (proportional speed directly mapped to analog trigger depth) |
| `Left Bumper` | `IntakeOutCommand` | Intake OUT + Transfer (manual reverse eject cargo) |
| `X Button` | `AutoEjectCommand` | Manual Purge sequence (1-second full reverse spin) |
| `A Button` | `TransferSubsystem` | Manual Transfer trigger (runs 1150 RPM transfer motor for 500 ms) |
| `Right Trigger` (analog > 0.05) | `ShootStartCommand` | Turret Shooter START (PID closed-loop flywheel spin-up) |
| `Right Bumper` | `ShootStopCommand` | Turret Shooter STOP (immediate flywheel stop) |
| `D-Pad UP` | `RelocalizeCommand` | PedroPathing pose relocalization using Limelight 3A corrected botpose |

### Gamepad 2 (Secondary Driver)

| Button / Trigger | Component / Command | Action Description |
| --- | --- | --- |
| Unassigned | — | All primary driver controls are consolidated on Gamepad 1 for unified single-driver execution |

---

## 3. OpModes

| Class Name | Annotation & Group | Type | Description |
| --- | --- | --- | --- |
| `MainTeleOp` | `@TeleOp(name = "Main TeleOp", group = "Competition")` | TeleOp | Primary competition TeleOp integrating command-based Mecanum drive, proportional trigger intake, transfer motor control, flywheel shooter PID, and bulk caching. |
| `BlueVisionTeleOp` | `@TeleOp(name = "Blue Vision TeleOp", group = "Competition")` | TeleOp | Competition TeleOp for Blue Alliance featuring automated Pinpoint odometry turret tracking to Blue goals and vision relocalization. |
| `RedVisionTeleOp` | `@TeleOp(name = "Red Vision TeleOp", group = "Competition")` | TeleOp | Competition TeleOp for Red Alliance featuring automated Pinpoint odometry turret tracking to Red goals and vision relocalization. |
| `VisionTeleOp` | Abstract Base Class | TeleOp Base | Base class extending `CommandOpMode` providing automated turret goal tracking and PedroPathing relocalization for alliance-specific TeleOps. |

---

## 4. Vision Pipelines

| Pipeline / Class | Hardware Name | Detections & Capabilities | OpModes Used In | Configurable Parameters & Thresholds |
| --- | --- | --- | --- | --- |
| `VisionSubsystem` | `"limelight"` (`Limelight3A`) | AprilTag tracking (Red Scoring 30..33, Red Audience 34..37, Blue Audience 38..41, Blue Scoring 42..45), 3D botpose, and field goal targeting. | `BlueVisionTeleOp`, `RedVisionTeleOp` | Y-Offset = 84.125 mm (3.31 in), Pitch = 21.0°, Red Goals (60in, 60in)/(60in, 84in), Blue Goals (84in, 60in)/(84in, 84in), Pipeline 0 |

---

## 5. Subsystems

| Subsystem Class | Hardware Controlled (Config Name) | Key Behaviors & Features |
| --- | --- | --- |
| `DriveSubsystem` | `leftFront`, `rightFront`, `leftRear`, `rightRear` (`DcMotorEx`), `"pinpoint"` (`GoBildaPinpointDriver`) | Field-centric and robot-centric Mecanum kinematics driven by Pedro Pathing `Follower`, heading normalization, pose estimation, and vision relocalization. |
| `IntakeSubsystem` | `intakeMotor` (`DcMotorEx`, goBILDA 1100 RPM), `transferMotor` (`DcMotorEx`, goBILDA 1150 RPM) | Inward/outward cargo intaking driven proportionally by analog trigger pressure (`0.0` to `1.0`). |
| `TransferSubsystem` | `transferMotor` (`DcMotorEx`, goBILDA 1150 RPM) | Manual trigger (`A` button). State machine (`IDLE`, `TRANSFERRING`, `COOLDOWN`) running motor at full power (`1.0`) for 500 ms with 1000 ms cooldown. |
| `TurretShooterSubsystem` | `shooterMotor` (`DcMotorEx`, goBILDA 6000 RPM, `"shooterMotor"`) | Closed-loop PID + Feedforward flywheel velocity control, state tracking (`IDLE`, `SPINNING_UP`, `AT_SPEED`), target RPM calculation, and detailed encoder telemetry. |
| `TurretSubsystem` | `turretServo` (`Servo`, Axon MAX Mark 2, `"turretServo"`) | Position-controlled rotation mechanism (0° to 220° physical range mapped from 0° to 355° servo PWM). |
| `VisionSubsystem` | `limelight` (`Limelight3A`) | Limelight 3A AprilTag targeting, dynamic goal coordinate resolution, Pinpoint odometry turret angle calculation, and corrected botpose calculation. |

---

## 6. Shooter / Outtake Velocity Config

| Parameter | Value | Specification / Description |
| --- | --- | --- |
| Motor Model | goBILDA Yellow Jacket 6000 RPM (SKU 5202-0002-0051) | Single flywheel motor with built-in quadrature hall-effect encoder |
| Encoder Resolution | 28.0 CPR | Ticks per revolution at the output shaft |
| Max Motor Speed | 6000.0 RPM / 2800.0 Ticks/sec | Rated maximum motor velocity |
| Short Range Velocity | 925.0 Ticks/sec (~1982.1 RPM) | Standard target velocity for close outtake shots |
| Long Range Velocity | 1075.0 Ticks/sec (~2303.6 RPM) | Standard target velocity for long outtake shots |
| Target RPM Tolerance | ±50.0 RPM | Allowable error range for `AT_SPEED` state classification |
| Proportional Gain (`kP`) | `0.005` | Flywheel velocity PID proportional term |
| Integral Gain (`kI`) | `0.01` | Flywheel velocity PID integral term |
| Derivative Gain (`kD`) | `0.00005` | Flywheel velocity PID derivative term |
| Anti-Windup Limit (`maxI`) | `0.3` | Integral term clamping limit |
| Static Friction (`kS`) | `8.07` | Feedforward static friction offset |
| Velocity Constant (`kV`) | `0.00067` | Feedforward velocity multiplier |
| Acceleration Constant (`kA`) | `0.0` | Feedforward acceleration gain |

---

## 7. Drive Configuration

| Parameter | Configuration Value | Details |
| --- | --- | --- |
| Drivetrain Type | Mecanum Drivetrain | 4 independent Mecanum wheels for omnidirectional translation |
| Drivetrain Motors | goBILDA Yellow Jacket 425 RPM (`DcMotorEx`) | `leftFront` (REVERSE), `leftRear` (REVERSE), `rightFront` (FORWARD), `rightRear` (FORWARD) |
| Brake Mode | `manualBrakeMode = true` | Motors set to `BRAKE` when zero power is applied |
| Localizer Hardware | GoBilda Pinpoint Driver (`"pinpoint"`) | Hardware odometry localizer with 2-wheel 4-bar odometry pods |
| Pinpoint Pod Offsets | Y-Pod = -168.0 mm, X-Pod = -84.0 mm | Physical pod offset coordinates relative to robot center |
| Pod Encoder Direction | `FORWARD` (X-Pod & Y-Pod) | Encoder rotation orientation |
| Motion Engine | Pedro Pathing 3.0.1 | Motion algorithm using `Follower`, `Mecanum`, and `Foresight` |
| Foresight Config | `brakeAggression = 1.0`, `maxBrakingPower = 0.2` | Translational tolerance 2.5 in, Heading tolerance 11.25° |

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
| `"shooterMotor"` | `DcMotorEx` (goBILDA 6000 RPM) | `TurretShooterSubsystem` — Flywheel Shooter Motor |
| `"turretServo"` | `Servo` (Axon MAX Mark 2) | `TurretSubsystem` — Rotating Turret Mechanism |
| `"limelight"` | `Limelight3A` | `VisionSubsystem` — Limelight Vision Camera |

---

## 10. Branch Structure

- **`master`**: Primary production branch containing verified competition code.
- **`odod-turret-detached`** (Active Branch): Development branch combining Pinpoint odometry localizer, turret subsystem tracking, and detached architecture.
- **`moving-turret`**: Feature branch for dynamic on-the-fly turret goal tracking while driving.
- **`straight-transfer`**: Subsystem branch for the straight-line single-motor transfer mechanism.
- **`detached`**: Experimental branch isolating turret and vision subsystems from drivetrain dependencies.
- **`combine`**: Integration branch used to synthesize parallel subsystem feature branches.

---

*Last Updated: March 30, 2026*
