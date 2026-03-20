package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // Field boundary limits for rejecting wild MT2 poses (meters).
    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH  = 8.07;

    // Jump-distance thresholds for pose rejection.
    // Multi-tag poses are far more reliable, so we allow much larger jumps
    // (including initial localization from any starting position on the field).
    private static final double JUMP_THRESHOLD_SINGLE_TAG = 1.0;   // meters
    private static final double JUMP_THRESHOLD_MULTI_TAG  = 20.0;  // meters (> field diagonal, allows init from anywhere)

    // Maximum single-tag ambiguity to accept (MT2 resolves most ambiguity via
    // gyro heading, but very high values indicate poor corner detection).
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.7;

    // Minimum tag count required for the MT1 bootstrap to accept a heading.
    // 2+ tags give MT1 a reliable heading; single-tag MT1 has severe ambiguity.
    private static final int MT1_BOOTSTRAP_MIN_TAGS = 2;

    // -------------------------------------------------------------------------

    private final ShotCalculator shotCalc;
    private final Swerve swerve;

    // True until the first multi-tag MT1 observation seeds the gyro offset.
    // While bootstrapping, we use MT1 (which solves for heading from geometry)
    // instead of MT2 (which requires a correct heading input).
    private boolean headingBootstrapped = false;

    // Rejection counters for field debugging (reset each cycle).
    private int rejectSpin = 0;
    private int rejectNull = 0;
    private int rejectBounds = 0;
    private int rejectJump = 0;
    private int rejectAmbiguity = 0;
    private int acceptCount = 0;

    // Written in periodic(), read by AutoAlign via getCurrentShot() — both run on
    // the main robot thread, so no synchronization is needed.
    private ShotCalculator.LaunchParameters currentShot = ShotCalculator.LaunchParameters.INVALID;


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
        config.maxScoringDistance = 5.5; // slightly beyond LUT max (5.0m) for interpolation margin
        config.headingSpeedScalar = 1.0;
        config.headingReferenceDistance = 2.5;
        config.shooterAngleOffsetRad = Math.PI;  // 0.0 means the shooter faces the same direction as the robot front; π means it faces backward.

        shotCalc = new ShotCalculator(config, lut);
    }

    @Override
    
    public void periodic() {
        // 1. IMU mode: seed internal IMU from external gyro while disabled (mode 1),
        //    switch to fused internal+external mode while enabled (mode 4).
        //    Docs: mode 1 seeds each frame; mode 4 uses LL4's 1kHz IMU + gentle external correction.
        int imuMode = DriverStation.isEnabled() ? 4 : 1;
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, imuMode);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, imuMode);

        // 2. Alliance-aware hub selection.
        //    Must live in periodic() so it picks up FMS alliance assignment after init.
        Translation2d hubCenter  = BLUE_HUB_CENTER;
        Translation2d hubForward = BLUE_HUB_FORWARD;

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            hubCenter  = RED_HUB_CENTER;
            hubForward = RED_HUB_FORWARD;
        }

        // 3. Heading bootstrap: on first boot, the gyro offset is 0° which may be
        //    wrong if the robot starts at an arbitrary orientation. Use MegaTag1
        //    (which solves for heading from tag geometry alone) to get a reliable
        //    multi-tag heading, seed the gyro offset, then switch to MT2 for all
        //    subsequent cycles.
        double yawRateDegPerSec = Math.toDegrees(swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond);

        if (!headingBootstrapped) {
            tryBootstrapHeading(yawRateDegPerSec);
        }

        // 4. Feed heading and yaw rate to both Limelights for MegaTag2.
        //    Uses the gyro-derived field heading (rawGyro + offset from last reset),
        //    NOT the pose estimator heading. The estimator heading could include
        //    tiny vision corrections, creating a feedback loop.
        double headingDeg    = swerve.getFieldHeadingFromGyro().getDegrees();
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

        // Reset per-cycle rejection counters.
        rejectSpin = 0; rejectNull = 0; rejectBounds = 0; rejectJump = 0; rejectAmbiguity = 0; acceptCount = 0;

        // 6. Process each Limelight independently (let the Kalman filter fuse them).
        //    Sanity checks per MT2 best practices:
        //      - Reject when spinning too fast (above)
        //      - Reject null / zero-tag results
        //      - Reject poses outside the field boundary
        //      - Reject poses that jump too far from current estimate (dynamic threshold)
        //      - Reject single-tag poses with high ambiguity
        //    Std dev formula (Gray Matter / community consensus):
        //      xyStdDev = 0.5 * avgTagDist² / tagCount
        //    Heading std = 9999999 — always trust gyro, never vision heading.
        visionConfidence += processCamera(leftPose, spinningTooFast);
        visionConfidence += processCamera(rightPose, spinningTooFast);

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

        // Rejection breakdown for field debugging.
        SmartDashboard.putNumber("Vision/RejectSpin", rejectSpin);
        SmartDashboard.putNumber("Vision/RejectNull", rejectNull);
        SmartDashboard.putNumber("Vision/RejectBounds", rejectBounds);
        SmartDashboard.putNumber("Vision/RejectJump", rejectJump);
        SmartDashboard.putNumber("Vision/RejectAmbiguity", rejectAmbiguity);
        SmartDashboard.putNumber("Vision/AcceptCount", acceptCount);
        SmartDashboard.putBoolean("Vision/HeadingBootstrapped", headingBootstrapped);
        SmartDashboard.putNumber("Vision/GyroFieldHeadingDeg", headingDeg);
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
     * Attempts to bootstrap heading from MegaTag1 multi-tag observations.
     * MT1 solves for both position AND heading from tag geometry, so it doesn't
     * need a correct heading input. Once a reliable multi-tag MT1 pose is found,
     * we use its heading to seed the gyro offset, then switch to MT2 permanently.
     */
    private void tryBootstrapHeading(double yawRateDegPerSec) {
        // Don't bootstrap while spinning — MT1 heading is unreliable during fast rotation.
        if (Math.abs(yawRateDegPerSec) > 120.0) {
            return;
        }

        // Try both cameras for a multi-tag MT1 result.
        LimelightHelpers.PoseEstimate mt1 = pickBestMT1(
                LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.primaryLimelightName),
                LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.secondaryLimelightName));

        if (mt1 == null) {
            return;
        }

        // Validate the MT1 pose is on the field.
        double x = mt1.pose.getX();
        double y = mt1.pose.getY();
        if (x < 0 || x > FIELD_LENGTH || y < 0 || y > FIELD_WIDTH) {
            return;
        }

        // MT1 heading is reliable with 2+ tags. Use it to seed the full pose
        // (position + heading) and compute the gyro offset.
        Rotation2d mt1Heading = mt1.pose.getRotation();
        swerve.setPose(mt1.pose);
        headingBootstrapped = true;

        // Also re-seed the Limelight IMUs with the corrected heading.
        seedIMU(mt1Heading.getDegrees());
    }

    /**
     * Returns the best multi-tag MT1 pose from two cameras, or null if neither qualifies.
     * Prefers the camera with more tags; breaks ties by closer average tag distance.
     */
    private LimelightHelpers.PoseEstimate pickBestMT1(
            LimelightHelpers.PoseEstimate a, LimelightHelpers.PoseEstimate b) {
        boolean aValid = a != null && a.tagCount >= MT1_BOOTSTRAP_MIN_TAGS;
        boolean bValid = b != null && b.tagCount >= MT1_BOOTSTRAP_MIN_TAGS;
        if (!aValid && !bValid) return null;
        if (!bValid) return a;
        if (!aValid) return b;
        // Both valid: prefer more tags, then closer distance.
        if (a.tagCount != b.tagCount) return a.tagCount > b.tagCount ? a : b;
        return a.avgTagDist <= b.avgTagDist ? a : b;
    }

    /**
     * Validates and processes a single camera's MT2 pose estimate.
     * Returns 0.5 if the measurement was accepted, 0.0 if rejected.
     */
    private double processCamera(LimelightHelpers.PoseEstimate pose, boolean spinningTooFast) {
        if (spinningTooFast) {
            rejectSpin++;
            return 0.0;
        }
        if (pose == null || pose.tagCount == 0) {
            rejectNull++;
            return 0.0;
        }

        // Reject poses outside the field boundary.
        double x = pose.pose.getX();
        double y = pose.pose.getY();
        if (x < 0 || x > FIELD_LENGTH || y < 0 || y > FIELD_WIDTH) {
            rejectBounds++;
            return 0.0;
        }

        // Reject single-tag poses with high ambiguity. MT2 uses the gyro to
        // resolve most ambiguity, but very high values indicate the tag corners
        // were poorly detected (glare, motion blur, extreme viewing angle).
        if (pose.tagCount == 1
                && pose.rawFiducials != null
                && pose.rawFiducials.length > 0
                && pose.rawFiducials[0].ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
            rejectAmbiguity++;
            return 0.0;
        }

        // Dynamic jump threshold: multi-tag poses are far more reliable, so
        // allow larger jumps. This also solves the startup problem where the
        // estimator begins at (0,0) and rejects the first valid vision pose.
        double jumpThreshold = (pose.tagCount >= 2) ? JUMP_THRESHOLD_MULTI_TAG : JUMP_THRESHOLD_SINGLE_TAG;
        double jumpM = swerve.getPose().getTranslation().getDistance(pose.pose.getTranslation());
        if (jumpM > jumpThreshold) {
            rejectJump++;
            return 0.0;
        }

        // Community std dev formula: 0.5 * avgTagDist² / tagCount.
        // Heading std = 9999999 — always trust gyro.
        double xyStdDev = 0.5 * Math.pow(pose.avgTagDist, 2.0) / pose.tagCount;
        swerve.addVisionMeasurement(pose.pose, pose.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
        acceptCount++;
        return 0.5;
    }

    /**
     * Hard-seeds both Limelight IMUs with the given heading.
     *
     * Since resetGyro() no longer calls gyro.setYaw(), the raw gyro value hasn't
     * changed after a reset. Callers must pass the intended heading explicitly.
     *
     * @param headingDeg the target heading in degrees (pass the same value used in
     *                   resetGyro, NOT a live gyro read).
     */
    public void seedIMU(double headingDeg) {
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName,   headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName,   1);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, 1);
        // Manual seed means heading is known — skip MT1 bootstrap.
        headingBootstrapped = true;
    }
}
