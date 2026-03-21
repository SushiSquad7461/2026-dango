package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

public class Vision extends SubsystemBase {
    private record TxTyMeasurement(
            boolean hasTarget,
            double distanceMeters,
            Rotation2d driveAngle,
            String cameraName,
            double txDegrees,
            double tyDegrees) {}

    private final ShotCalculator shotCalc;
    private final ShotLUT shotLut;
    private final ProjectileSimulator projectileModel;
    private final Swerve swerve;
    private final PIDController rotationPID = Constants.Vision.rotationPID;
    private final SOTMShotPhysicsSim runtimeProjectileSim;

    // Written in periodic(), read by AutoAlign via getCurrentShot(); both run on
    // the main robot thread, so no synchronization is needed.
    private ShotCalculator.LaunchParameters currentShot = ShotCalculator.LaunchParameters.INVALID;

    private Translation2d lastHubCenter = Constants.Vision.BLUE_HUB_CENTER;
    private Translation2d lastHubForward = Constants.Vision.BLUE_HUB_FORWARD;

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        projectileModel = new ProjectileSimulator(Constants.Vision.SOTM_PARAMETERS);
        shotLut = buildDefaultShotLut(projectileModel);

        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = Constants.Vision.LAUNCHER_OFFSET_METERS.getX();
        config.launcherOffsetY = Constants.Vision.LAUNCHER_OFFSET_METERS.getY();
        // phaseDelayMs: set to actual pipeline latency from Limelight's "tl" field.
        // Log limelight.getTl() during testing and replace this placeholder.
        config.phaseDelayMs = 30.0;
        config.mechLatencyMs = 20.0;
        // maxTiltDeg: 5.0 suppresses firing over bumps/ramps where the launcher
        // is knocked off-axis. 90.0 effectively disables this gate.
        config.maxTiltDeg = 5.0;
        config.maxScoringDistance = Constants.Vision.TXTY_MAX_DISTANCE_METERS;
        config.headingSpeedScalar = 1.0;
        config.headingReferenceDistance = 2.5;
        // Shooter is rear-facing in this robot configuration.
        config.shooterAngleOffsetRad = Math.PI;

        shotCalc = new ShotCalculator(config, shotLut);

        runtimeProjectileSim = RobotBase.isSimulation()
                ? new SOTMShotPhysicsSim(Constants.Vision.SOTM_PARAMETERS)
                : null;

        rotationPID.enableContinuousInput(-180, 180);
        // 2 deg tolerance at a typical 5m shot distance = ~17cm miss at the hub.
        rotationPID.setTolerance(2.0);
    }

    @Override
    public void periodic() {
        // 1) IMU mode: seed internal IMU from external gyro while disabled (mode 1),
        //    switch to fused internal+external mode while enabled (mode 4).
        int imuMode = DriverStation.isEnabled() ? 4 : 1;
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, imuMode);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, imuMode);

        // 2) Alliance-aware hub selection. Must run periodically to pick up FMS alliance.
        Translation2d hubCenter = Constants.Vision.BLUE_HUB_CENTER;
        Translation2d hubForward = Constants.Vision.BLUE_HUB_FORWARD;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            hubCenter = Constants.Vision.RED_HUB_CENTER;
            hubForward = Constants.Vision.RED_HUB_FORWARD;
        }
        lastHubCenter = hubCenter;
        lastHubForward = hubForward;

        // 3) Feed heading + yaw rate to Limelight for MegaTag2.
        double headingDeg = swerve.getPose().getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(swerve.getRobotRelativeSpeeds().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);

        // 4) Fetch MegaTag2 pose estimates.
        LimelightHelpers.PoseEstimate leftPose =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.primaryLimelightName);
        LimelightHelpers.PoseEstimate rightPose =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.secondaryLimelightName);

        // visionConfidence is normalized to [0.0, 1.0] and passed into ShotInputs.
        double visionConfidence = 0.0;

        // Limelight docs: reject vision updates when spinning too fast.
        boolean spinningTooFast = Math.abs(yawRateDegPerSec) > 360.0;

        // 5) Process left Limelight.
        if (!spinningTooFast && leftPose != null && leftPose.tagCount > 0) {
            visionConfidence += 0.5;

            double xyStdDev = leftPose.tagCount > 1 ? 0.1 : 0.5;
            xyStdDev += Math.pow(leftPose.avgTagDist, 2.0) * 0.1;
            swerve.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
        }

        // 6) Process right Limelight.
        if (!spinningTooFast && rightPose != null && rightPose.tagCount > 0) {
            visionConfidence += 0.5;

            double xyStdDev = rightPose.tagCount > 1 ? 0.1 : 0.5;
            xyStdDev += Math.pow(rightPose.avgTagDist, 2.0) * 0.1;
            swerve.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
        }

        // tx/ty stays as a live measurement source and fallback path.
        TxTyMeasurement txTy = selectBestTxTyMeasurement(swerve.getPose(), hubCenter);
        if (txTy.hasTarget()) {
            visionConfidence = Math.max(visionConfidence, 0.25);
        }

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
                swerve.getPose(),
                swerve.getFieldVelocity(),
                swerve.getRobotRelativeSpeeds(),
                hubCenter,
                hubForward,
                visionConfidence);

        ShotCalculator.LaunchParameters solvedShot = shotCalc.calculate(inputs);
        boolean usingFallback = false;

        if (solvedShot.isValid()) {
            currentShot = solvedShot;
        } else if (txTy.hasTarget()) {
            currentShot = buildTxTyFallbackShot(txTy, hubCenter);
            usingFallback = true;
        } else {
            currentShot = ShotCalculator.LaunchParameters.INVALID;
        }

        if (runtimeProjectileSim != null) {
            runtimeProjectileSim.update(0.02);
            runtimeProjectileSim.logOutputs();
        }

        SmartDashboard.putNumber("Vision/Confidence", currentShot.confidence());
        SmartDashboard.putNumber("Vision/TargetRPM", currentShot.rpm());
        SmartDashboard.putNumber("Vision/SolvedDistanceM", currentShot.solvedDistanceM());
        SmartDashboard.putNumber("Vision/HoodAngleDeg", currentShot.hoodAngleDeg());
        SmartDashboard.putNumber("Vision/DriveAngleDeg", currentShot.driveAngle().getDegrees());
        SmartDashboard.putBoolean("Vision/ShotValid", currentShot.isValid());
        SmartDashboard.putBoolean("Vision/SpinningTooFast", spinningTooFast);
        SmartDashboard.putNumber("Vision/SolverIterations", currentShot.iterationsUsed());
        SmartDashboard.putData("Vision/RotationPID", rotationPID);
        SmartDashboard.putNumber("Vision/LimelightTLLeft", LimelightHelpers.getLatency_Pipeline(Constants.Vision.primaryLimelightName));
        SmartDashboard.putNumber("Vision/LimelightTLRight", LimelightHelpers.getLatency_Pipeline(Constants.Vision.secondaryLimelightName));

        SmartDashboard.putNumber("Vision/RobotX", swerve.getPose().getX());
        SmartDashboard.putNumber("Vision/RobotY", swerve.getPose().getY());
        SmartDashboard.putNumber("Vision/RobotHeadingDeg", swerve.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Vision/VisionConfidence", visionConfidence);
        SmartDashboard.putNumber("Vision/HubX", hubCenter.getX());
        SmartDashboard.putNumber("Vision/HubY", hubCenter.getY());
        SmartDashboard.putNumber("Vision/DistToHubRaw", swerve.getPose().getTranslation().getDistance(hubCenter));
        SmartDashboard.putNumber("Vision/TagCountLeft", leftPose != null ? leftPose.tagCount : 0);
        SmartDashboard.putNumber("Vision/TagCountRight", rightPose != null ? rightPose.tagCount : 0);

        SmartDashboard.putBoolean("Vision/UsingTxTyFallback", usingFallback);
        SmartDashboard.putBoolean("Vision/TxTyVisible", txTy.hasTarget());
        SmartDashboard.putNumber("Vision/TxTyDistanceM", txTy.distanceMeters());
        SmartDashboard.putNumber("Vision/TxTyDeg", txTy.txDegrees());
        SmartDashboard.putNumber("Vision/TyDeg", txTy.tyDegrees());
        SmartDashboard.putString("Vision/TxTyCamera", txTy.cameraName());
        SmartDashboard.putNumber("Vision/ActiveFuelProjectiles", runtimeProjectileSim != null ? runtimeProjectileSim.getActiveCount() : 0);
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

    public void seedIMU() {
        double headingDeg = swerve.getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, 1);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, 1);
    }

    public void onSimShotLaunch(SOTMShotPhysicsSim.LaunchState launchState) {
        if (runtimeProjectileSim == null) {
            return;
        }
        runtimeProjectileSim.spawnProjectile(launchState);
    }

    private static ShotLUT buildDefaultShotLut(ProjectileSimulator simulator) {
        try {
            ShotLUT physicsLut = simulator.generateLUT();
            // Keep static table as optional fallback if generation fails or yields no points.
            if (physicsLut != null && (physicsLut.get(2.0) != null || physicsLut.get(3.0) != null)) {
                return physicsLut;
            }
        } catch (Exception ignored) {
            // Fall through to static fallback if enabled.
        }

        if (Constants.Vision.USE_STATIC_SHOT_TABLE_FALLBACK) {
            return ShotTable.buildLUT();
        }

        // Last resort: try physics LUT again to keep behavior deterministic.
        return simulator.generateLUT();
    }

    private TxTyMeasurement selectBestTxTyMeasurement(Pose2d robotPose, Translation2d hubCenter) {
        TxTyMeasurement left = sampleTxTyMeasurement(
                robotPose,
                hubCenter,
                Constants.Vision.primaryLimelightName,
                Constants.Vision.cameraPosePrimary);
        TxTyMeasurement right = sampleTxTyMeasurement(
                robotPose,
                hubCenter,
                Constants.Vision.secondaryLimelightName,
                Constants.Vision.cameraPoseSecondary);

        if (left.hasTarget() && right.hasTarget()) {
            return LimelightHelpers.getTA(Constants.Vision.primaryLimelightName)
                    >= LimelightHelpers.getTA(Constants.Vision.secondaryLimelightName)
                            ? left
                            : right;
        }
        if (left.hasTarget()) {
            return left;
        }
        if (right.hasTarget()) {
            return right;
        }

        Rotation2d hubAngle =
                new Rotation2d(hubCenter.getX() - robotPose.getX(), hubCenter.getY() - robotPose.getY());
        return new TxTyMeasurement(false,
                robotPose.getTranslation().getDistance(hubCenter),
                hubAngle,
                "none",
                0.0,
                0.0);
    }

    private TxTyMeasurement sampleTxTyMeasurement(
            Pose2d robotPose, Translation2d hubCenter, String cameraName, Pose3d cameraPose) {
        if (!LimelightHelpers.getTV(cameraName)) {
            Rotation2d hubAngle =
                    new Rotation2d(hubCenter.getX() - robotPose.getX(), hubCenter.getY() - robotPose.getY());
            return new TxTyMeasurement(false,
                    robotPose.getTranslation().getDistance(hubCenter),
                    hubAngle,
                    cameraName,
                    0.0,
                    0.0);
        }

        double txDeg = LimelightHelpers.getTX(cameraName);
        double tyDeg = LimelightHelpers.getTY(cameraName);

        // Distance estimate from ty uses standard camera-height relation.
        double cameraPitchDeg = Math.toDegrees(cameraPose.getRotation().getY());
        double totalPitchRad = Math.toRadians(cameraPitchDeg + tyDeg);
        double denominator = Math.tan(totalPitchRad);

        double fallbackDistance = robotPose.getTranslation().getDistance(hubCenter);
        double distanceMeters = fallbackDistance;
        if (Math.abs(denominator) > 1e-4) {
            double solved = (Constants.Vision.TARGET_CENTER_HEIGHT_METERS - cameraPose.getZ()) / denominator;
            if (Double.isFinite(solved) && solved > 0.0) {
                distanceMeters = solved;
            }
        }

        distanceMeters = MathUtil.clamp(
                distanceMeters,
                Constants.Vision.TXTY_MIN_DISTANCE_METERS,
                Constants.Vision.TXTY_MAX_DISTANCE_METERS);

        double robotHeading = robotPose.getRotation().getRadians();
        double driveHeading = robotHeading + cameraPose.getRotation().getZ() + Math.toRadians(txDeg);
        Rotation2d driveAngle = Rotation2d.fromRadians(driveHeading);

        return new TxTyMeasurement(true, distanceMeters, driveAngle, cameraName, txDeg, tyDeg);
    }

    private ShotCalculator.LaunchParameters buildTxTyFallbackShot(
            TxTyMeasurement txTyMeasurement, Translation2d hubCenter) {
        double distanceM = MathUtil.clamp(
                txTyMeasurement.distanceMeters(),
                Constants.Vision.TXTY_MIN_DISTANCE_METERS,
                Constants.Vision.TXTY_MAX_DISTANCE_METERS);

        ShotLUT.ShotParameters params = shotLut.get(distanceM);

        double rpm = params != null ? params.rpm() : Constants.Shooter.TARGET_RPM_DEFAULT;
        double hoodAngle = params != null ? params.angle() : Constants.Vision.TXTY_DEFAULT_HOOD_ANGLE_DEG;
        double tof = params != null ? params.tof() : 0.0;

        Rotation2d driveAngle = txTyMeasurement.hasTarget()
                ? txTyMeasurement.driveAngle()
                : new Rotation2d(hubCenter.getX() - swerve.getPose().getX(), hubCenter.getY() - swerve.getPose().getY());

        return new ShotCalculator.LaunchParameters(
                rpm,
                tof,
                driveAngle,
                0.0,
                true,
                Constants.Vision.TXTY_FALLBACK_CONFIDENCE,
                distanceM,
                hoodAngle,
                0,
                false);
    }

    public Translation2d getCurrentHubCenter() {
        return lastHubCenter;
    }

    public Translation2d getCurrentHubForward() {
        return lastHubForward;
    }

    public ProjectileSimulator getProjectileModel() {
        return projectileModel;
    }
}

