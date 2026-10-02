# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
./gradlew build           # Compile + run tests
./gradlew compileJava     # Compile only (fastest check)
./gradlew deploy          # Deploy to RoboRIO
```

CI runs `./gradlew build` on push/PR to Main using `wpilib/roborio-cross-ubuntu:2025-22.04`.

## Architecture

FRC Team 7461 (Sushi Squad) command-based robot using **WPILib 2026**, **AdvantageKit**, and **Phoenix6** (CTRE TalonFX/Kraken motors).

### Hardware Access

Subsystems own their Phoenix6 hardware directly (no IO/sim layers). `Shooter` wraps `ShooterHW` (flywheel + feeder) and `HoodHW`. `drive/ModuleIO` and `drive/gyro/GyroIO` are unused leftovers.

### State Machine

`StateMachine.java` maps robot states (`IDLE`, `SHOOT_ONLY`, `INTAKE_DOWN`, `INTAKE_DOWN_AND_SHOOT`) to intake pivot/rollers, hopper, and shooter. Shooting states run `AutoShot`.

### Subsystems

- **Swerve** — SDS MK4i L3 with KrakenX60, Pigeon2 gyro, PathPlanner autos. Also fuses MegaTag2 poses from both Limelights ("limelight-left", "limelight-right")
- **Shooter** — Flywheel + hood (MotionMagic) + feeder. `AutoShot` picks RPM/hood from the `ShotTable` LUT by distance to the hub
- **Intake** — Pivot (two TalonFX, MotionMagic) + rollers
- **Hopper** — Simple transport motor
- **AutoAlign** — Turns to face the hub (`AllianceUtil.getHubCenter()`) while the driver translates

### Key Constants Location

`src/main/java/frc/robot/generated/Constants.java` holds hand-written constants (gversion generates `BuildConstants.java`, not this file). Shot data lives in `ShotTable.java`.

### Vendor Libraries

Phoenix6 26.1.1, AdvantageKit 26.0.0, PathplannerLib 2026.1.2, PhotonLib v2026.2.2, REVLib 2026.0.2. Vendordep JSONs are in `/vendordeps/`.

## Conventions

- Gear ratios are expressed as motor rotations per mechanism rotation
- Shooter RPM is sent straight to the motor; no flywheel gear ratio is applied
- Public subsystem actions return `Command`s; plain methods are for same-package use or read-only checks
- Autonomous paths are PathPlanner `.path` files registered as named commands in `AutoCommands`
