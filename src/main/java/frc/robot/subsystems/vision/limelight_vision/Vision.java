package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightLeft;
    private final NetworkTable limelightRight;
    private final Swerve swerve;

    // Track heartbeats so we don't re-submit the same stale frame every loop
    private double lastHbLeft = -1;
    private double lastHbRight = -1;

    private boolean hasPoseLeft = false;
    private boolean hasPoseRight = false;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        limelightLeft = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        limelightRight = NetworkTableInstance.getDefault().getTable(Constants.Vision.secondaryLimelightName);

        // Tell each limelight where it sits on the robot so its pose math is accurate.
        // Format: [forward_m, side_m (left+), up_m, roll_deg, pitch_deg, yaw_deg]
        limelightLeft.getEntry("camerapose_robotspace_set").setDoubleArray(
            new double[]{ -0.263525, 0.263525, 0.2439162, 0, 20, 150 });
        limelightRight.getEntry("camerapose_robotspace_set").setDoubleArray(
            new double[]{ -0.263525, -0.263525, 0.2439162, 0, 20, -150 });
    }

    /**
     * Reads a MegaTag1 (botpose_wpiblue) frame from one limelight and feeds it into
     * the swerve pose estimator. MegaTag1 does NOT require gyro input — it computes
     * the robot's field-relative position purely from visible AprilTags. This means
     * the robot knows where it is from the moment it boots, regardless of orientation.
     *
     * Returns the new heartbeat value (store it for next loop to avoid re-submitting stale frames).
     */
    private double processMegaTag1(NetworkTable limelight, double lastHb) {
        double hb = limelight.getEntry("hb").getDouble(-1);
        if (hb == lastHb) return lastHb; // no new frame from limelight

        // botpose_wpiblue: [x, y, z, roll, pitch, yaw, latency_ms, tagCount, tagSpan, avgDist, avgArea]
        double[] botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (botpose.length < 11) return hb;

        int tagCount = (int) botpose[7];
        if (tagCount < 1) return hb;

        double x = botpose[0];
        double y = botpose[1];
        double yawDeg = botpose[5];
        double latencyMs = botpose[6];
        double avgDist = botpose[9];

        // Reject impossible poses
        if (x < 0 || x > 17.6 || y < 0 || y > 8.2) return hb;

        // Single-tag estimates get noisy fast with distance — reject far single-tag readings
        if (tagCount == 1 && avgDist > 3.0) return hb;

        double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        // Trust position more with multiple nearby tags; yaw from gyro is always more accurate than vision
        double xyStdDev = (tagCount >= 2) ? 0.7 : 1.5;
        swerve.addVisionMeasurement(
            new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg)),
            timestamp,
            VecBuilder.fill(xyStdDev, xyStdDev, 999)
        );
        return hb;
    }

    /** Returns true if at least one limelight produced a valid pose update this loop. */
    public boolean hasPoseEstimate() {
        return hasPoseLeft || hasPoseRight;
    }

    /**
     * Returns the field-relative heading the robot must face so its launcher (back of robot)
     * points at the hub. Computed from swerve pose — tags don't need to be visible.
     */
    public Rotation2d getHeadingToHub(boolean isRed) {
        Pose2d robotPose = swerve.getPose();
        Translation2d hub = isRed ? Constants.Vision.RED_HUB_POSITION : Constants.Vision.BLUE_HUB_POSITION;
        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();
        // +PI because the launcher is at the back of the robot
        return new Rotation2d(Math.atan2(dy, dx) + Math.PI);
    }

    /**
     * Returns the straight-line distance (meters) from the robot to the hub.
     * Computed from swerve pose — tags don't need to be visible.
     */
    public double getDistanceToHub(boolean isRed) {
        Pose2d robotPose = swerve.getPose();
        Translation2d hub = isRed ? Constants.Vision.RED_HUB_POSITION : Constants.Vision.BLUE_HUB_POSITION;
        return robotPose.getTranslation().getDistance(hub);
    }

    @Override
    public void periodic() {
        double newHbLeft = processMegaTag1(limelightLeft, lastHbLeft);
        hasPoseLeft = (newHbLeft != lastHbLeft);
        lastHbLeft = newHbLeft;

        double newHbRight = processMegaTag1(limelightRight, lastHbRight);
        hasPoseRight = (newHbRight != lastHbRight);
        lastHbRight = newHbRight;

        Pose2d pose = swerve.getPose();
        SmartDashboard.putBoolean("Vision/HasPoseLeft", hasPoseLeft);
        SmartDashboard.putBoolean("Vision/HasPoseRight", hasPoseRight);
        SmartDashboard.putNumber("Vision/RobotX", pose.getX());
        SmartDashboard.putNumber("Vision/RobotY", pose.getY());
        SmartDashboard.putNumber("Vision/RobotHeadingDeg", pose.getRotation().getDegrees());
        SmartDashboard.putData("Vision/AutoAlignPID", Constants.Vision.rotationPID);
    }
}
