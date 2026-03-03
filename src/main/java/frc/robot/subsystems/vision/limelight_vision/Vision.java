package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

public class Vision extends SubsystemBase {
    private final String limelightLeftName = "limelight-left";
    private final String limelightRightName = "limelight-right";
    private final Swerve drive;

    // TODO: Update these with the actual 2026 constants for the Score Pillars
    private static final Translation2d BLUE_SCORE_PILLAR = new Translation2d(0.0, 0.0);
    private static final Translation2d RED_SCORE_PILLAR = new Translation2d(16.54, 0.0);

    public enum StartingPosition {
        POS_1, POS_2, POS_3, POS_4
    }

    public Vision(Swerve drive) {
        this.drive = drive;
        // LimelightHelpers handles the NetworkTables internally
    }

    /**
     * Updates Limelights with current robot orientation, reads MegaTag2 poses,
     * and feeds them into the Swerve pose estimator if valid.
     */
    private void updateOdometryWithVision() {
        // 1. Send current robot yaw to Limelights (CCW positive, 0 = facing red
        // alliance wall)
        // Ensure Odometry rotation is passed in degrees
        double yaw = drive.getHeading().getDegrees();

        LimelightHelpers.SetRobotOrientation(limelightLeftName, yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(limelightRightName, yaw, 0, 0, 0, 0, 0);

        // 2. Read MT2 botpose from cameras (botpose_orb_wpiblue)
        PoseEstimate leftEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightLeftName);
        PoseEstimate rightEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightRightName);

        // 3. Choose the best pose
        PoseEstimate bestEstimate = null;

        boolean leftValid = leftEstimate != null && leftEstimate.tagCount > 0;
        boolean rightValid = rightEstimate != null && rightEstimate.tagCount > 0;

        if (leftValid && rightValid) {
            // Since cameras are at angle, pick the one with the larger target area (better
            // view)
            if (leftEstimate.avgTagArea > rightEstimate.avgTagArea) {
                bestEstimate = leftEstimate;
            } else {
                bestEstimate = rightEstimate;
            }
        } else if (leftValid) {
            bestEstimate = leftEstimate;
        } else if (rightValid) {
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