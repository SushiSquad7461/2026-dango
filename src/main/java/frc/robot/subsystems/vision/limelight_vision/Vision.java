package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

        // write camera mount poses (camera in robot coordinates) into each Limelight if provided
        if (Constants.Vision.cameraPosePrimary != null) {
            double[] primaryPoseArr = LimelightHelpers.pose3dToArray(Constants.Vision.cameraPosePrimary);
            LimelightHelpers.setLimelightNTDoubleArray(Constants.Vision.primaryLimelightName, "camerapose_robotspace_set", primaryPoseArr);
        }

        if (Constants.Vision.cameraPoseSecondary != null) {
            double[] secondaryPoseArr = LimelightHelpers.pose3dToArray(Constants.Vision.cameraPoseSecondary);
            LimelightHelpers.setLimelightNTDoubleArray(Constants.Vision.secondaryLimelightName, "camerapose_robotspace_set", secondaryPoseArr);
        }

        LimelightHelpers.Flush();
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

        Pose3d leftRobot = null;
        Pose3d rightRobot = null;

        if (tvLeft == 1.0) {
            Pose3d leftCam = new Pose3d(
                new Translation3d(tagPoseLeft[0], tagPoseLeft[1], tagPoseLeft[2]),
                new Rotation3d(Math.toRadians(tagPoseLeft[3]), Math.toRadians(tagPoseLeft[4]), Math.toRadians(tagPoseLeft[5]))
            );
            leftRobot = Constants.Vision.cameraPosePrimary.transformBy(new Transform3d(leftCam.getTranslation(), leftCam.getRotation()));
        }

        if (tvRight == 1.0) {
            Pose3d rightCam = new Pose3d(
                new Translation3d(tagPoseRight[0], tagPoseRight[1], tagPoseRight[2]),
                new Rotation3d(Math.toRadians(tagPoseRight[3]), Math.toRadians(tagPoseRight[4]), Math.toRadians(tagPoseRight[5]))
            );
            rightRobot = Constants.Vision.cameraPoseSecondary.transformBy(new Transform3d(rightCam.getTranslation(), rightCam.getRotation()));
        }

        double tx;
        double ty;

        if (leftRobot != null && rightRobot != null) {
            tx = (leftRobot.getX() + rightRobot.getX()) / 2.0;
            ty = (leftRobot.getY() + rightRobot.getY()) / 2.0;
        } else if (leftRobot != null) {
            tx = leftRobot.getX();
            ty = leftRobot.getY();
        } else if (rightRobot != null) {
            tx = rightRobot.getX();
            ty = rightRobot.getY();
        } else {
            return new Rotation2d();
        }

        double angleRad = Math.atan2(ty, tx);
        return swerve.getHeading().plus(new Rotation2d(angleRad));
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

        Pose3d leftRobot = null;
        Pose3d rightRobot = null;

        if (tvLeft == 1.0) {
            Pose3d leftCam = new Pose3d(
                new Translation3d(tagPoseLeft[0], tagPoseLeft[1], tagPoseLeft[2]),
                new Rotation3d(Math.toRadians(tagPoseLeft[3]), Math.toRadians(tagPoseLeft[4]), Math.toRadians(tagPoseLeft[5]))
            );
            leftRobot = Constants.Vision.cameraPosePrimary.transformBy(new Transform3d(leftCam.getTranslation(), leftCam.getRotation()));
        }

        if (tvRight == 1.0) {
            Pose3d rightCam = new Pose3d(
                new Translation3d(tagPoseRight[0], tagPoseRight[1], tagPoseRight[2]),
                new Rotation3d(Math.toRadians(tagPoseRight[3]), Math.toRadians(tagPoseRight[4]), Math.toRadians(tagPoseRight[5]))
            );
            rightRobot = Constants.Vision.cameraPoseSecondary.transformBy(new Transform3d(rightCam.getTranslation(), rightCam.getRotation()));
        }

        double dx;
        double dy;

        if (leftRobot != null && rightRobot != null) {
            dx = (leftRobot.getX() + rightRobot.getX()) / 2.0;
            dy = (leftRobot.getY() + rightRobot.getY()) / 2.0;
        } else if (leftRobot != null) {
            dx = leftRobot.getX();
            dy = leftRobot.getY();
        } else if (rightRobot != null) {
            dx = rightRobot.getX();
            dy = rightRobot.getY();
        } else {
            return Double.NaN;
        }

        return Math.hypot(dx, dy); // Limelight's 2D pose has Y as the forward direction
    }

    public void periodic() {
        SmartDashboard.putData("Vision/AutoAlignPID", Constants.Vision.rotationPID);
    }
}