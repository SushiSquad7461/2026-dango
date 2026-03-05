package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

public class Vision extends SubsystemBase {
    // Reads from the primary Limelight which has been configured in the UI as a
    // "Limelight Vision Array". This camera will pull tag corners from the
    // secondary
    // camera and compute a single, unified MegaTag pose.
    private final NetworkTable limelightTable;
    private final Swerve swerve;
    public Vision(Swerve swerve) {
        this.swerve = swerve;
        limelightTable = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        LimelightHelpers.SetIMUMode(Constants.Vision.primaryLimelightName, 4);
        LimelightHelpers.SetIMUMode(Constants.Vision.secondaryLimelightName, 4);
        LimelightHelpers.SetIMUAssistAlpha(Constants.Vision.primaryLimelightName, 0.01);
        LimelightHelpers.SetIMUAssistAlpha(Constants.Vision.secondaryLimelightName, 0.01);
    }
    public Rotation2d getHeadingToScorePillar(boolean isRed) {
        // make sure a valid target exists
        double tv = limelightTable.getEntry("tv").getDouble(0.0); // 1.0 when a target is valid
        if (tv < 0.5) {
            return new Rotation2d(); // no target
        }
        double[] tagPose = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        if (tagPose == null || tagPose.length < 6) {
            return new Rotation2d();
        }

        double x = tagPose[0];
        double y = tagPose[1]; // Limelight's 2D pose has Y as the forward direction
        if (isRed) {
            return new Rotation2d(Math.atan2(y, x));
        } else {
            return new Rotation2d(Math.atan2(-y, -x));
        }
    }
    public double getDistanceToScorePillar() {
        double tv = limelightTable.getEntry("tv").getDouble(0.0);
        if (tv < 0.5) {
            return Double.NaN;
        }
        double[] tagPose = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        if (tagPose == null || tagPose.length < 6) {
            return Double.NaN;
        }
        double x = tagPose[0];
        double y = tagPose[1]; // Limelight's 2D pose has Y as the forward direction
        return Math.hypot(x, y);
    }
    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName, swerve.getGyroYaw().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, swerve.getGyroYaw().getDegrees(), 0, 0, 0, 0, 0);
    }
}