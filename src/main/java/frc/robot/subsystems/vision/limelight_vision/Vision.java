package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose3d;
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
    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(Constants.Vision.primaryLimelightName, swerve.getGyroYaw().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.Vision.secondaryLimelightName, swerve.getGyroYaw().getDegrees(), 0, 0, 0, 0, 0);
    }
}