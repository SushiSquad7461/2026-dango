package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.ProjectileSimulator.GeneratedLUT;
import frc.robot.subsystems.vision.ProjectileSimulator.LUTEntry;

public class Vision extends SubsystemBase {
    private ProjectileSimulator projectileSimulator;
    private ShotCalculator shotCalc;
    private NetworkTable limelight_left;
    private NetworkTable limelight_right;
    private Swerve swerve;
    private ShooterSubsystem shooter;

    public Vision(Swerve swerve, ShooterSubsystem shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
        limelight_left = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        limelight_right = NetworkTableInstance.getDefault().getTable(Constants.Vision.secondaryLimelightName);
        projectileSimulator = new ProjectileSimulator(Constants.Vision.SOTM_PARAMETERS);
        GeneratedLUT lut = projectileSimulator.generateLUT();
        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = 0.23;  // how far forward the launcher is from robot center (m)
        config.launcherOffsetY = 0.0;   // how far left, 0 if centered
        config.phaseDelayMs = 30.0;     // your vision pipeline latency
        config.mechLatencyMs = 20.0;    // how long the mechanism takes to respond
        config.maxTiltDeg = 90.0;        // suppress firing when chassis tilts past this (bumps/ramps)
        config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
        config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub
        ShotCalculator shotCalc = new ShotCalculator(config);
        // load the LUT you generated
        for (LUTEntry entry : lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
            }
        }
        this.shotCalc = shotCalc;

        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, 4);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, 4);
    }

    public void periodic() {
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // 1. Fetch the poses
LimelightHelpers.PoseEstimate leftPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.primaryLimelightName);
LimelightHelpers.PoseEstimate rightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.secondaryLimelightName);

// 2. Process Left Limelight
if (leftPose != null && leftPose.tagCount > 0) {
    // Start with a baseline trust (e.g., 0.5 meters)
    double xyStdDev = 0.5;
    
    // If it sees multiple tags, we trust it WAY more
    if (leftPose.tagCount > 1) xyStdDev = 0.1;
    
    // Add penalty based on distance (farther = less trust)
    xyStdDev += Math.pow(leftPose.avgTagDist, 2.0) * 0.1;

    // 9999999.0 is still strictly required to prevent MegaTag2 gyro feedback loops
    swerve.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
}

// 3. Process Right Limelight
if (rightPose != null && rightPose.tagCount > 0) {
    double xyStdDev = 0.5;
    if (rightPose.tagCount > 1) xyStdDev = 0.1;
    xyStdDev += Math.pow(rightPose.avgTagDist, 2.0) * 0.1;

    swerve.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0));
}

        Translation2d hubCenter = new Translation2d(4.6, 4.0);  // your target
        Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            swerve.getPose(),
            swerve.getFieldVelocity(),
            swerve.getRobotRelativeSpeeds(),
            hubCenter,
            hubForward,
            0.9,  // vision confidence, 0 to 1
            0.0,  // pitch for tilt gate (0.0 if no gyro)
            0.0    // roll for tilt gate (0.0 if no gyro)
        );

        ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
        if (shot.isValid() && shot.confidence() > 50) {
            shooter.setTargetRPM(shot.rpm());
            swerve.drive(new Translation2d(0, 0), shot.driveAngle().getDegrees(), true, true);
            // shot.driveAngularVelocityRadPerSec() gives you a heading feedforward if you want it
        }
    }
}
