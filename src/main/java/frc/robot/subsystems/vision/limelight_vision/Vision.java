package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightLeft;
    private final NetworkTable limelightRight;
    private final Swerve swerve;
    public Vision(Swerve swerve) {
        this.swerve = swerve;
        limelightLeft = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        limelightRight = NetworkTableInstance.getDefault().getTable(Constants.Vision.secondaryLimelightName);
    }
    public Rotation2d getHeadingToScorePillar(boolean isRed) {
        // make sure a valid target exists
        double tvLeft = limelightLeft.getEntry("tv").getDouble(0.0); // 1.0 when a target is valid
        double tvRight = limelightRight.getEntry("tv").getDouble(0.0); // 1.0 when a target is valid
        if (tvLeft < 0.5 && tvRight < 0.5) {
            return new Rotation2d(); // no target
        }
        double[] tagPoseLeft = limelightLeft.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        double[] tagPoseRight = limelightRight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        if ((tagPoseLeft == null || tagPoseLeft.length < 6) && (tagPoseRight == null || tagPoseRight.length < 6)) {
            return new Rotation2d();
        }
        double x;
        double y;
        if(tvLeft == 1.0 && tvRight == 1.0) {
            x = (tagPoseLeft[0] + tagPoseRight[0]) / 2;
            y = (tagPoseLeft[1] + tagPoseRight[1]) / 2;
        } else if (tvLeft == 1.0) {
            x = tagPoseLeft[0];
            y = tagPoseLeft[1];
        } else if (tvRight == 1.0) {
            x = tagPoseRight[0];
            y = tagPoseRight[1];
        } else {
            return new Rotation2d();
        }
        if (isRed) {
            return new Rotation2d(Math.atan2(y, x) + Math.toRadians(30));
        } else {
            return new Rotation2d(Math.atan2(-y, -x) - Math.toRadians(30));
        }
    }
    public double getDistanceToScorePillar() {
        // make sure a valid target exists
        double tvLeft = limelightLeft.getEntry("tv").getDouble(0.0); // 1.0 when a target is valid
        double tvRight = limelightRight.getEntry("tv").getDouble(0.0); // 1.0 when a target is valid
        if (tvLeft < 0.5 && tvRight < 0.5) {
            return Double.NaN; // no target
        }
        double[] tagPoseLeft = limelightLeft.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        double[] tagPoseRight = limelightRight.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        if ((tagPoseLeft == null || tagPoseLeft.length < 6) && (tagPoseRight == null || tagPoseRight.length < 6)) {
            return Double.NaN;
        }
        double x;
        double y;
        if(tvLeft == 1.0 && tvRight == 1.0) {
            x = (tagPoseLeft[0] + tagPoseRight[0]) / 2;
            y = (tagPoseLeft[1] + tagPoseRight[1]) / 2;
            return Math.hypot(x, y);
        } else if (tvLeft == 1.0) {
            x = tagPoseLeft[0];
            y = tagPoseLeft[1];
            return Math.hypot(x, y);
        } else {
            x = tagPoseRight[0];
            y = tagPoseRight[1];
            return Math.hypot(x, y);
        }// Limelight's 2D pose has Y as the forward direction
    }

    public void periodic() {
        SmartDashboard.putData("Vision/AutoAlignPID", Constants.Vision.rotationPID);
    }
}