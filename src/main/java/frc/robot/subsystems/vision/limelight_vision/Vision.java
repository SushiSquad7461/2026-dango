package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    private boolean hasPoseLeft = false;
    private boolean hasPoseRight = false;
    // Whether we've seeded the pose estimator from MegaTag1 yet.
    // MegaTag2 requires the gyro to already match the field heading — seeding fixes that.
    private boolean positionSeeded = false;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        limelightLeft = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        limelightRight = NetworkTableInstance.getDefault().getTable(Constants.Vision.secondaryLimelightName);

        // Set camera poses in robot space so MegaTag2 can do accurate pose estimation.
        // Format: [forward_m, side_m (left+), up_m, roll_deg, pitch_deg, yaw_deg]
        limelightLeft.getEntry("camerapose_robotspace_set").setDoubleArray(new double[]{
            -0.263525, 0.263525, 0.2439162, 0, 20, 150
        });
        limelightRight.getEntry("camerapose_robotspace_set").setDoubleArray(new double[]{
            -0.263525, -0.263525, 0.2439162, 0, 20, -150
        });
    }

    /**
     * Reads a MegaTag1 pose (botpose_wpiblue — no gyro needed) from one limelight
     * and calls swerve.setPose() to seed the pose estimator.
     * After this, MegaTag2 works correctly because the gyro offset is calibrated.
     * Returns true if a valid seed pose was found.
     */
    private boolean trySeedPose(NetworkTable limelight) {
        // MegaTag1: pure visual pose, does not require robot_orientation_set
        double[] botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        if (botpose.length < 11) return false;

        int tagCount = (int) botpose[7];
        if (tagCount < 2) return false; // require 2+ tags for a reliable seed

        double x = botpose[0];
        double y = botpose[1];
        double yawDeg = botpose[5];

        if (x < 0 || x > 17.6 || y < 0 || y > 8.2) return false;

        swerve.setPose(new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg)));
        return true;
    }

    /** Call this to force a re-seed (e.g. bound to a button if the robot is repositioned). */
    public void resetPoseSeed() {
        positionSeeded = false;
    }

    /**
     * Push the robot's current gyro yaw to both limelights every loop.
     * MegaTag2 requires this to constrain its pose estimate.
     */
    private void sendRobotOrientation() {
        double yawDeg = swerve.getHeading().getDegrees();
        // Format: [yaw, yawRate, pitch, pitchRate, roll, rollRate]
        double[] orientation = new double[]{yawDeg, 0, 0, 0, 0, 0};
        limelightLeft.getEntry("robot_orientation_set").setDoubleArray(orientation);
        limelightRight.getEntry("robot_orientation_set").setDoubleArray(orientation);
    }

    /**
     * Read MegaTag2 pose from one limelight and feed it into the swerve pose estimator.
     * Returns true if a valid pose was received.
     */
    private boolean processMegaTag2(NetworkTable limelight) {
        // botpose_orb_wpiblue: [x, y, z, roll, pitch, yaw, latency_ms, tagCount, tagSpan, avgDist, avgArea]
        double[] botpose = limelight.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
        if (botpose.length < 11) return false; // need indices 0-10

        int tagCount = (int) botpose[7];
        if (tagCount < 1) return false;

        double x = botpose[0];
        double y = botpose[1];
        double yawDeg = botpose[5];
        double latencyMs = botpose[6];
        double timestamp = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

        // Reject poses outside field bounds
        if (x < 0 || x > 17.6 || y < 0 || y > 8.2) return false;

        // Standard deviations: trust x/y position, ignore vision yaw (gyro is more accurate).
        Matrix<N3, N1> stdDevs;
        if (tagCount >= 2) {
            stdDevs = VecBuilder.fill(0.3, 0.3, 999);
        } else {
            double avgDist = botpose[9];
            if (avgDist > 4.0) return false; // single far tag is too noisy
            stdDevs = VecBuilder.fill(1.0, 1.0, 999);
        }

        swerve.addVisionMeasurement(new Pose2d(x, y, Rotation2d.fromDegrees(yawDeg)), timestamp, stdDevs);
        return true;
    }

    /** Returns true if at least one limelight has a valid MegaTag2 pose this loop. */
    public boolean hasPoseEstimate() {
        return hasPoseLeft || hasPoseRight;
    }

    /**
     * Returns the field-relative heading the robot should face to point its launcher at the hub.
     * Computed from the robot's current pose — does not require hub tags to be visible.
     */
    public Rotation2d getHeadingToHub(boolean isRed) {
        Pose2d robotPose = swerve.getPose();
        Translation2d hubPos = isRed ? Constants.Vision.RED_HUB_POSITION : Constants.Vision.BLUE_HUB_POSITION;
        double dx = hubPos.getX() - robotPose.getX();
        double dy = hubPos.getY() - robotPose.getY();
        // +PI because the launcher faces the back of the robot
        return new Rotation2d(Math.atan2(dy, dx) + Math.PI);
    }

    /**
     * Returns the 2D distance (meters) from the robot to the hub.
     * Computed from the robot's current pose — does not require hub tags to be visible.
     */
    public double getDistanceToHub(boolean isRed) {
        Pose2d robotPose = swerve.getPose();
        Translation2d hubPos = isRed ? Constants.Vision.RED_HUB_POSITION : Constants.Vision.BLUE_HUB_POSITION;
        return robotPose.getTranslation().getDistance(hubPos);
    }

    @Override
    public void periodic() {
        // Step 1: seed the pose from MegaTag1 once before relying on MegaTag2.
        // MegaTag1 uses pure visual detection (no gyro), so it works regardless of boot orientation.
        if (!positionSeeded) {
            if (trySeedPose(limelightLeft) || trySeedPose(limelightRight)) {
                positionSeeded = true;
            }
        }

        // Step 2: push gyro yaw so MegaTag2 can use it.
        sendRobotOrientation();

        // Step 3: feed MegaTag2 corrections into the pose estimator every loop.
        hasPoseLeft = processMegaTag2(limelightLeft);
        hasPoseRight = processMegaTag2(limelightRight);

        Pose2d pose = swerve.getPose();
        SmartDashboard.putBoolean("Vision/PoseSeeded", positionSeeded);
        SmartDashboard.putBoolean("Vision/HasPoseLeft", hasPoseLeft);
        SmartDashboard.putBoolean("Vision/HasPoseRight", hasPoseRight);
        SmartDashboard.putNumber("Vision/RobotX", pose.getX());
        SmartDashboard.putNumber("Vision/RobotY", pose.getY());
        SmartDashboard.putNumber("Vision/RobotHeadingDeg", pose.getRotation().getDegrees());
        SmartDashboard.putData("Vision/AutoAlignPID", Constants.Vision.rotationPID);
    }
}
