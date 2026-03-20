package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

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
    private static final Translation2d BLUE_HUB_CENTER  = new Translation2d(4.029, 4.034);
    private static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1, 0);  // hub faces +X (toward field center)

    private static final Translation2d RED_HUB_CENTER   = new Translation2d(12.513, 4.034);
    private static final Translation2d RED_HUB_FORWARD  = new Translation2d(-1, 0);  // hub faces -X (toward field center)

    // -------------------------------------------------------------------------

    private final ShotCalculator shotCalc;
    private final Swerve swerve;
    private final PIDController rotationPID = Constants.Vision.rotationPID;

    // Written in periodic(), read by AutoAlign via getCurrentShot() — both run on
    // the main robot thread, so no synchronization is needed.
    private ShotCalculator.LaunchParameters currentShot = ShotCalculator.LaunchParameters.INVALID;

    // Hard-reset the pose estimator once on the first confident multi-tag fix so
    // the robot doesn't start at field origin (0, 0) and slowly converge.
    private boolean poseInitialized = false;

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        ShotLUT lut = ShotTable.buildLUT();
        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = -0.1905;  // negative: launcher is behind robot center
        config.launcherOffsetY = 0.0;    // 0 if centered
        // phaseDelayMs: set to actual pipeline latency from Limelight's "tl" field.
        // Log limelight.getTl() during testing and replace this placeholder.
        config.phaseDelayMs = 30.0;
        config.mechLatencyMs = 20.0;
        // maxTiltDeg: 5.0 suppresses firing over bumps/ramps where the launcher
        // is knocked off-axis. 90.0 (the previous value) effectively disabled this gate.
        config.maxTiltDeg = 5.0;
        config.maxScoringDistance = 15.0; // TODO: tighten once hub coordinates are verified
        config.headingSpeedScalar = 1.0;
        config.headingReferenceDistance = 2.5;
        config.shooterAngleOffsetRad = Math.PI;  // 0.0 means the shooter faces the same direction as the robot front; π means it faces backward.

        shotCalc = new ShotCalculator(config, lut);

        rotationPID.enableContinuousInput(-180, 180);
        // 2° tolerance at a typical 5m shot distance = ~17cm miss at the hub.
        // Consider tightening this or making it distance-dependent during tuning.
        rotationPID.setTolerance(2.0);

    }

    @Override
    
    public void periodic() {
        // 1. IMU mode: seed internal IMU from external gyro while disabled (mode 1),
        //    switch to fused internal+external mode while enabled (mode 4).
        //    Docs: mode 1 seeds each frame; mode 4 uses LL4's 1kHz IMU + gentle external correction.
        int imuMode = DriverStation.isEnabled() ? 4 : 1;
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, imuMode);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, imuMode);

        // 3. Alliance-aware hub selection.
        //    Must live in periodic() so it picks up FMS alliance assignment after init.
        Translation2d hubCenter  = BLUE_HUB_CENTER;
        Translation2d hubForward = BLUE_HUB_FORWARD;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            hubCenter  = RED_HUB_CENTER;
            hubForward = RED_HUB_FORWARD;
        }

        // 4. Feed heading and yaw rate to both Limelights for MegaTag2.
        double headingDeg    = swerve.getPose().getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName,   headingDeg, yawRateDegPerSec, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);

        // 5. Fetch MegaTag2 pose estimates.
        LimelightHelpers.PoseEstimate leftPose  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.primaryLimelightName);
        LimelightHelpers.PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.secondaryLimelightName);

        // visionConfidence is normalized to [0.0, 1.0] and passed into ShotInputs.
        // Note: this is NOT the same scale as LaunchParameters.confidence(), which is 0–100.
        double visionConfidence = 0.0;

        // Limelight docs: reject vision updates when spinning too fast.
        // Fast rotation makes pose estimates unreliable; 360°/s is the recommended threshold.
        boolean spinningTooFast = Math.abs(yawRateDegPerSec) > 360.0;

        // 6. Process left Limelight.
        if (!spinningTooFast && leftPose != null && leftPose.tagCount > 0) {
            visionConfidence += 0.5;

            // On the first confident multi-tag reading, hard-reset the pose estimator
            // so the robot doesn't spend several seconds converging from (0, 0, 0°).
            if (!poseInitialized && leftPose.tagCount >= 2 && leftPose.avgTagDist < Constants.Vision.POSE_INIT_MAX_TAG_DIST_M) {
                swerve.setPose(leftPose.pose);
                seedIMU();
                poseInitialized = true;
            }

            // Base std dev is tighter with multiple tags, and increases with distance.
            // 9999999 on heading tells the Kalman filter to ignore vision heading;
            // MegaTag2 heading comes from the IMU, not vision.
            double xyStdDev = leftPose.tagCount > 1 ? 0.1 : 0.5;
            xyStdDev += Math.pow(leftPose.avgTagDist, 2.0) * 0.1;
            swerve.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
        }

        // 7. Process right Limelight.
        if (!spinningTooFast && rightPose != null && rightPose.tagCount > 0) {
            visionConfidence += 0.5;

            if (!poseInitialized && rightPose.tagCount >= 2 && rightPose.avgTagDist < Constants.Vision.POSE_INIT_MAX_TAG_DIST_M) {
                swerve.setPose(rightPose.pose);
                seedIMU();
                poseInitialized = true;
            }

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

        SmartDashboard.putNumber("Vision/Confidence", currentShot.confidence());
        SmartDashboard.putNumber("Vision/TargetRPM", currentShot.rpm());
        SmartDashboard.putNumber("Vision/SolvedDistanceM", currentShot.solvedDistanceM());
        SmartDashboard.putNumber("Vision/HoodAngleDeg", currentShot.hoodAngleDeg());
        SmartDashboard.putNumber("Vision/DriveAngleDeg", currentShot.driveAngle().getDegrees());
        SmartDashboard.putBoolean("Vision/ShotValid", currentShot.isValid());
        SmartDashboard.putBoolean("Vision/SpinningTooFast", spinningTooFast);
        SmartDashboard.putNumber("Vision/SolverIterations", currentShot.iterationsUsed());
        SmartDashboard.putData("Vision/RotationPID", rotationPID);
        SmartDashboard.putNumber("Vision/LimelightTLLeft",  LimelightHelpers.getLatency_Pipeline(Constants.Vision.primaryLimelightName));
        SmartDashboard.putNumber("Vision/LimelightTLRight", LimelightHelpers.getLatency_Pipeline(Constants.Vision.secondaryLimelightName));
        // Debug: raw pose and vision tag counts to distinguish pose vs. solver failures
        SmartDashboard.putNumber("Vision/RobotX", swerve.getPose().getX());
        SmartDashboard.putNumber("Vision/RobotY", swerve.getPose().getY());
        SmartDashboard.putNumber("Vision/RobotHeadingDeg", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/VisionConfidence", visionConfidence);
        SmartDashboard.putNumber("Vision/HubX", hubCenter.getX());
        SmartDashboard.putNumber("Vision/HubY", hubCenter.getY());
        double distToHub = swerve.getPose().getTranslation().getDistance(hubCenter);
        SmartDashboard.putNumber("Vision/DistToHubRaw", distToHub);
        int leftTags  = (leftPose  != null) ? leftPose.tagCount  : 0;
        int rightTags = (rightPose != null) ? rightPose.tagCount : 0;
        SmartDashboard.putNumber("Vision/TagCountLeft",  leftTags);
        SmartDashboard.putNumber("Vision/TagCountRight", rightTags);
    }

    public ShotCalculator.LaunchParameters getCurrentShot() {
        return currentShot;
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

    /**
     * Hard-seeds both Limelight IMUs with the current robot heading.
     *
     * In mode 4 (enabled), the Limelight uses its internal 1kHz IMU with only gentle
     * external correction. After a gyro reset the internal IMU won't snap to the new
     * heading for several seconds. Calling this immediately after resetGyro() forces a
     * one-shot mode-1 seed so MegaTag2 estimates are correct right away.
     * The next periodic() call will restore the correct mode (1 or 4).
     *
     * Call order matters: invoke this AFTER swerve.resetGyro() so getPose() already
     * returns the new heading (0°).
     */
    public void seedIMU() {
        double headingDeg = swerve.getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName,   headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName,   1);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, 1);
    }
}
