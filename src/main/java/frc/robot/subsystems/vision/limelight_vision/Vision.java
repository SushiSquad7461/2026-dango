package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Swerve;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightLeft;
    private final NetworkTable limelightRight;
    private final Swerve drive;

    // TODO: Update these with the actual 2026 constants for the Score Pillars
    private static final Translation2d BLUE_SCORE_PILLAR = new Translation2d(0.0, 0.0);
    private static final Translation2d RED_SCORE_PILLAR = new Translation2d(16.54, 0.0);

    public enum StartingPosition {
        POS_1, POS_2, POS_3, POS_4
    }

    public Vision(Swerve drive) {
        this.drive = drive;
        // TODO: Ensure these networktable names match the names of your Limelights
        limelightLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
        limelightRight = NetworkTableInstance.getDefault().getTable("limelight-right");
    }

    /**
     * Updates Limelights with current robot orientation, reads MegaTag2 poses,
     * and feeds them into the Swerve pose estimator if valid.
     */
    private void updateOdometryWithVision() {
        // 1. Send current robot yaw to Limelights (CCW positive, 0 = facing red
        // alliance wall)
        // Ensure Odometry rotation is passed in degrees
        double yaw = drive.getRotation().getDegrees();
        double[] orientationStr = new double[] { yaw, 0, 0, 0, 0, 0 };

        limelightLeft.getEntry("robot_orientation_set").setDoubleArray(orientationStr);
        limelightRight.getEntry("robot_orientation_set").setDoubleArray(orientationStr);

        // 2. Read MT2 botpose from cameras (botpose_orb_wpiblue)
        PoseEstimate leftEstimate = getPoseEstimate(limelightLeft);
        PoseEstimate rightEstimate = getPoseEstimate(limelightRight);

        // 3. Choose the best pose
        PoseEstimate bestEstimate = null;

        if (leftEstimate != null && rightEstimate != null) {
            // Since cameras are at angle, pick the one with the larger target area (better
            // view)
            if (leftEstimate.targetArea > rightEstimate.targetArea) {
                bestEstimate = leftEstimate;
            } else {
                bestEstimate = rightEstimate;
            }
        } else if (leftEstimate != null) {
            bestEstimate = leftEstimate;
        } else if (rightEstimate != null) {
            bestEstimate = rightEstimate;
        }

        // 4. Feed into Swerve odometry if valid
        if (bestEstimate != null) {
            // Trust MegaTag2 x/y, but rely completely on Gyro for rotation (degStds =
            // 9999999)
            double xyStds = 0.7; // Can be scaled dynamically if needed based on distance or targetArea
            double degStds = 9999999;

            drive.addVisionMeasurement(bestEstimate.pose, bestEstimate.timestampSeconds,
                    VecBuilder.fill(xyStds, xyStds, degStds));
        }
    }

    private PoseEstimate getPoseEstimate(NetworkTable table) {
        if (table.getEntry("tv").getDouble(0) != 1.0) {
            return null; // No target seen
        }

        // MegaTag 2 botpose array: [X, Y, Z, Roll, Pitch, Yaw(Angle)]
        double[] botpose = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length < 6)
            return null; // Malformed data

        // Sometimes tv is 1 but botpose is all 0s
        if (botpose[0] == 0 && botpose[1] == 0)
            return null;

        double latency = table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0);
        double timestampSeconds = Timer.getFPGATimestamp() - (latency / 1000.0);

        // Use Target Area to approximate the quality of the view
        double targetArea = table.getEntry("ta").getDouble(0);

        Pose2d pose = new Pose2d(
                botpose[0], botpose[1],
                Rotation2d.fromDegrees(botpose[5]));

        return new PoseEstimate(pose, timestampSeconds, targetArea);
    }

    private static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double targetArea;

        public PoseEstimate(Pose2d pose, double timestampSeconds, double targetArea) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.targetArea = targetArea;
        }
    }

    /**
     * Returns the heading (Rotation2d) required to face the chosen score pillar.
     */
    public Rotation2d getHeadingToScorePillar(boolean isRed) {
        Pose2d currentPose = drive.getPose();
        Translation2d target = isRed ? RED_SCORE_PILLAR : BLUE_SCORE_PILLAR;

        Translation2d difference = target.minus(currentPose.getTranslation());
        return difference.getAngle();
    }

    /**
     * Returns the distance (meters) to the chosen score pillar.
     */
    public double getDistanceToScorePillar(boolean isRed) {
        Pose2d currentPose = drive.getPose();
        Translation2d target = isRed ? RED_SCORE_PILLAR : BLUE_SCORE_PILLAR;

        return currentPose.getTranslation().getDistance(target);
    }

    /**
     * Initializes the robot's pose based on 4 possible start positions.
     * Note: Cameras are on the back facing outward. So front of the robot faces the
     * opposing alliance wall.
     */
    public void initializeStartPose(StartingPosition pos, boolean isRed) {
        // TODO: Update these with the actual 2026 starting coordinates
        double x = isRed ? 15.0 : 1.5;
        double y = 0.0;

        switch (pos) {
            case POS_1:
                y = 2.0;
                break;
            case POS_2:
                y = 4.0;
                break;
            case POS_3:
                y = 6.0;
                break;
            case POS_4:
                y = 8.0;
                break;
        }

        // If Red, pointing towards blue (-180 / 180 degrees)
        // If Blue, pointing towards red (0 degrees)
        // Since cameras are on the back, the back faces the alliance wall, so front
        // faces the field
        Rotation2d initialHeading = isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

        Pose2d startPose = new Pose2d(x, y, initialHeading);
        drive.setPose(startPose);
    }

    @Override
    public void periodic() {
        // Runs MegaTag 2 Updates
        updateOdometryWithVision();
    }
}