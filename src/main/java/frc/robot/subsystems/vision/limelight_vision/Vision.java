package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

public class Vision extends SubsystemBase {
    private final NetworkTable limelightLeft;
    private final NetworkTable limelightRight;
    private final Swerve swerve;
    private final Pose3d camPosePrimary;
    private final Pose3d camPoseSecondary;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        limelightLeft = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        limelightRight = NetworkTableInstance.getDefault().getTable(Constants.Vision.secondaryLimelightName);
        // Fall back to identity pose (camera at robot center) if constants aren't filled in
        camPosePrimary = Constants.Vision.cameraPosePrimary != null ? Constants.Vision.cameraPosePrimary : new Pose3d();
        camPoseSecondary = Constants.Vision.cameraPoseSecondary != null ? Constants.Vision.cameraPoseSecondary : new Pose3d();
    }

    /**
     * Converts a tag pose from camera space to robot space using the camera's mount pose.
     * poseArr is [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg] in camera frame.
     */
    private Pose3d tagCamToRobotSpace(double[] poseArr, Pose3d camPose) {
        Pose3d tagInCam = new Pose3d(
            new Translation3d(poseArr[0], poseArr[1], poseArr[2]),
            new Rotation3d(Math.toRadians(poseArr[3]), Math.toRadians(poseArr[4]), Math.toRadians(poseArr[5]))
        );
        return camPose.transformBy(new Transform3d(tagInCam.getTranslation(), tagInCam.getRotation()));
    }

    /**
     * Returns the absolute field heading the robot should face to point toward the score pillar.
     * Reads raw camera-space data and manually converts to robot frame.
     */
    public Rotation2d getHeadingToScorePillar(boolean isRed) {
        double tvLeft = limelightLeft.getEntry("tv").getDouble(0.0);
        double tvRight = limelightRight.getEntry("tv").getDouble(0.0);
        if (tvLeft < 0.5 && tvRight < 0.5) {
            return new Rotation2d();
        }

        Pose3d leftRobot = null;
        Pose3d rightRobot = null;

        if (tvLeft == 1.0) {
            double[] arr = limelightLeft.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
            leftRobot = tagCamToRobotSpace(arr, camPosePrimary);
        }
        if (tvRight == 1.0) {
            double[] arr = limelightRight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
            rightRobot = tagCamToRobotSpace(arr, camPoseSecondary);
        }

        double tx, ty;
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

        // bearing in robot frame + robot's field heading = absolute target heading
        return swerve.getHeading().plus(new Rotation2d(Math.atan2(ty, tx)));
    }

    /**
     * Returns the 2D distance (meters) from the robot to the score pillar, or NaN if no target.
     */
    public double getDistanceToScorePillar() {
        double tvLeft = limelightLeft.getEntry("tv").getDouble(0.0);
        double tvRight = limelightRight.getEntry("tv").getDouble(0.0);
        if (tvLeft < 0.5 && tvRight < 0.5) {
            return Double.NaN;
        }

        Pose3d leftRobot = null;
        Pose3d rightRobot = null;

        if (tvLeft == 1.0) {
            double[] arr = limelightLeft.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
            leftRobot = tagCamToRobotSpace(arr, camPosePrimary);
        }
        if (tvRight == 1.0) {
            double[] arr = limelightRight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
            rightRobot = tagCamToRobotSpace(arr, camPoseSecondary);
        }

        double dx, dy;
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

        return Math.hypot(dx, dy);
    }

    public void periodic() {
        SmartDashboard.putData("Vision/AutoAlignPID", Constants.Vision.rotationPID);
    }
}
