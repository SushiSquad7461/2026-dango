package frc.robot.subsystems.vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.ProjectileSimulator.GeneratedLUT;
import frc.robot.subsystems.vision.ProjectileSimulator.LUTEntry;

public class Vision extends SubsystemBase {
    // -------------------------------------------------------------------------
    // 2026 REBUILT field constants (WPILib blue-origin coordinate system)
    //
    //   Field: 16.54m long (X) x 8.07m wide (Y)
    //   Origin: bottom-right corner of the BLUE alliance wall, +X toward Red.
    //
    //   Hub positions are derived from the FIRST field drawings:
    //     - The Hub is a 47in x 47in (~1.194m x 1.194m) element.
    //     - It is centered on the Y axis (Y = 8.07 / 2 = 4.035m).
    //     - The front face of the Alliance Zone is 158.6in (~4.029m) from the
    //       alliance wall; the Hub sits against the back of that zone, placing
    //       its center roughly 120in (~3.048m) from its own alliance wall.
    //
    //   TODO: Verify HUB_CENTER_X_BLUE against the official Onshape model or
    //   the WPILib 2026-rebuilt-welded.json AprilTag layout before competition.
    //   The AprilTag layout file is the most reliable source once available on
    //   your robot's WPILib installation.
    // -------------------------------------------------------------------------
    private static final double FIELD_LENGTH_M      = 16.54;  // 651.2in, per 2026 game manual
    private static final double FIELD_WIDTH_M       = 8.07;   // 317.7in, per 2026 game manual
    private static final double FIELD_MID_Y         = FIELD_WIDTH_M / 2.0;  // 4.035m

    // Blue Hub: ~120in from blue alliance wall, centered in Y
    // TODO: confirm against official CAD / AprilTag JSON
    private static final double HUB_CENTER_X_BLUE   = 3.048;  // ~120in from blue wall
    private static final Translation2d BLUE_HUB_CENTER  = new Translation2d(HUB_CENTER_X_BLUE, FIELD_MID_Y);
    private static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1, 0);  // hub faces +X (toward field center)

    // Red Hub: mirrored across field centerline
    private static final double HUB_CENTER_X_RED    = FIELD_LENGTH_M - HUB_CENTER_X_BLUE;
    private static final Translation2d RED_HUB_CENTER   = new Translation2d(HUB_CENTER_X_RED, FIELD_MID_Y);
    private static final Translation2d RED_HUB_FORWARD  = new Translation2d(-1, 0);  // hub faces -X (toward field center)

    // ShotCalculator.LaunchParameters.confidence() returns a 0–100 score
    // (per frc-fire-control README and quick-start example).
    // visionConfidence passed into ShotInputs is 0.0–1.0 — these are different scales.
    private static final double CONFIDENCE_THRESHOLD = 50.0;

    // -------------------------------------------------------------------------

    private final ProjectileSimulator projectileSimulator = new ProjectileSimulator(Constants.Vision.SOTM_PARAMETERS);
    private final ShotCalculator shotCalc;
    private final Swerve swerve;
    private final ShooterSubsystem shooter;
    private final PIDController rotationPID = Constants.Vision.rotationPID;

    // Written in periodic(), read in shootOnTheMove() — both run on the main
    // robot thread, so no synchronization is needed. If periodic() is ever
    // moved off the main thread this will need a lock.
    private ShotCalculator.LaunchParameters currentShot = ShotCalculator.LaunchParameters.INVALID;

    public Vision(Swerve swerve, ShooterSubsystem shooter) {
        this.swerve = swerve;
        this.shooter = shooter;

        GeneratedLUT lut = projectileSimulator.generateLUT();
        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = -0.23;  // negative: launcher is behind robot center
        config.launcherOffsetY = 0.0;    // 0 if centered
        // phaseDelayMs: set to actual pipeline latency from Limelight's "tl" field.
        // Log limelight.getTl() during testing and replace this placeholder.
        config.phaseDelayMs = 30.0;
        config.mechLatencyMs = 20.0;
        // maxTiltDeg: 5.0 suppresses firing over bumps/ramps where the launcher
        // is knocked off-axis. 90.0 (the previous value) effectively disabled this gate.
        config.maxTiltDeg = 5.0;
        config.headingSpeedScalar = 1.0;
        config.headingReferenceDistance = 2.5;
        config.shooterAngleOffsetRad = Math.PI;  // 0.0 means the shooter faces the same direction as the robot front; π means it faces backward.

        shotCalc = new ShotCalculator(config);

        for (LUTEntry entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }

        rotationPID.enableContinuousInput(-180, 180);
        // 2° tolerance at a typical 5m shot distance = ~17cm miss at the hub.
        // Consider tightening this or making it distance-dependent during tuning.
        rotationPID.setTolerance(2.0);

        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, 4);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, 4);
    }

    @Override
    public void periodic() {
        // 1. Alliance-aware hub selection.
        //    Must live in periodic() so it picks up FMS alliance assignment after init.
        Translation2d hubCenter  = BLUE_HUB_CENTER;
        Translation2d hubForward = BLUE_HUB_FORWARD;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            hubCenter  = RED_HUB_CENTER;
            hubForward = RED_HUB_FORWARD;
        }

        // 2. Feed heading and yaw rate to both Limelights for MegaTag2.
        double headingDeg    = swerve.getPose().getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName,   headingDeg, yawRateDegPerSec, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);

        // 3. Fetch MegaTag2 pose estimates.
        LimelightHelpers.PoseEstimate leftPose  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.primaryLimelightName);
        LimelightHelpers.PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.secondaryLimelightName);

        // visionConfidence is normalized to [0.0, 1.0] and passed into ShotInputs.
        // Note: this is NOT the same scale as LaunchParameters.confidence(), which is 0–100.
        double visionConfidence = 0.0;

        // Limelight docs: reject vision updates when spinning too fast.
        // Fast rotation makes pose estimates unreliable; 720°/s is the recommended threshold.
        boolean spinningTooFast = Math.abs(yawRateDegPerSec) > 720.0;

        // 4. Process left Limelight.
        if (!spinningTooFast && leftPose != null && leftPose.tagCount > 0) {
            visionConfidence += 0.5;

            // Base std dev is tighter with multiple tags, and increases with distance.
            // 9999999 on heading tells the Kalman filter to ignore vision heading;
            // MegaTag2 heading comes from the IMU, not vision.
            double xyStdDev = leftPose.tagCount > 1 ? 0.1 : 0.5;
            xyStdDev += Math.pow(leftPose.avgTagDist, 2.0) * 0.1;
            swerve.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
        }

        // 5. Process right Limelight.
        if (!spinningTooFast && rightPose != null && rightPose.tagCount > 0) {
            visionConfidence += 0.5;

            double xyStdDev = rightPose.tagCount > 1 ? 0.1 : 0.5;
            xyStdDev += Math.pow(rightPose.avgTagDist, 2.0) * 0.1;
            swerve.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
        }

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                swerve.getPose(),
                swerve.getFieldVelocity(),
                swerve.getRobotRelativeSpeeds(),
                hubCenter,
                hubForward,
                visionConfidence   // [0.0, 1.0]; pitch/roll omitted, convenience constructor passes 0.0, 0.0
        );

        currentShot = shotCalc.calculate(inputs);
    }

    public void resetOffset() {
        shotCalc.resetOffset();
    }

    public void resetWarmStart() {
        shotCalc.resetWarmStart();
    }

    public void adjustOffset(double offset) {
        shotCalc.adjustOffset(offset);
    }

    public Command shootOnTheMove(DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier driverRotation) {
        return Commands.run(() -> {
            Translation2d driverInput = new Translation2d(xTranslation.getAsDouble(), yTranslation.getAsDouble());

            // confidence() returns 0–100 (frc-fire-control README: "0-100 confidence score")
            if (currentShot.isValid() && currentShot.confidence() > CONFIDENCE_THRESHOLD) {
                // Spin up to the calculated RPM.
                shooter.setTargetRPM(currentShot.rpm());

                // PID on heading error + SOTM angular feedforward.
                // driveAngle() points the front of the robot at the hub, so rotate by π
                // to aim the rear-facing shooter instead.
                double currentHeading = swerve.getPose().getRotation().getDegrees();
                double targetHeading  = currentShot.driveAngle().rotateBy(Rotation2d.kPi).getDegrees();
                double pidOutput      = rotationPID.calculate(currentHeading, targetHeading);
                double rotationSpeed  = pidOutput + currentShot.driveAngularVelocityRadPerSec();

                swerve.drive(driverInput, rotationSpeed, true, true);
            } else {
                // Target lost or out of range — hand full control back to the driver.
                // Default RPM keeps the shooter warm for when a target reappears.
                // If you want to save battery when far from the hub, gate this on distance.
                shooter.setTargetRPM(4500);
                swerve.drive(driverInput, driverRotation.getAsDouble(), true, true);
            }
        }, this, swerve, shooter);
    }
}