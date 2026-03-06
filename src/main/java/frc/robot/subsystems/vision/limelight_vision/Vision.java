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

    // Cached for SmartDashboard logging
    private double lastBearingDeg = 0;
    private double lastTagRobotX = 0;
    private double lastTagRobotY = 0;

    public Vision(Swerve swerve) {
        this.swerve = swerve;
        limelightLeft = NetworkTableInstance.getDefault().getTable(Constants.Vision.primaryLimelightName);
        limelightRight = NetworkTableInstance.getDefault().getTable(Constants.Vision.secondaryLimelightName);
        camPosePrimary = Constants.Vision.cameraPosePrimary != null ? Constants.Vision.cameraPosePrimary : new Pose3d();
        camPoseSecondary = Constants.Vision.cameraPoseSecondary != null ? Constants.Vision.cameraPoseSecondary : new Pose3d();
    }

    /** Returns true if at least one camera currently sees a valid hub tag for the given alliance. */
    public boolean hasHubTarget(boolean isRed) {
        double tvLeft = limelightLeft.getEntry("tv").getDouble(0.0);
        double tvRight = limelightRight.getEntry("tv").getDouble(0.0);
        if (tvLeft == 1.0) {
            int tagId = (int) limelightLeft.getEntry("tid").getDouble(-1);
            if (isHubTag(tagId, isRed)) return true;
        }
        if (tvRight == 1.0) {
            int tagId = (int) limelightRight.getEntry("tid").getDouble(-1);
            if (isHubTag(tagId, isRed)) return true;
        }
        return false;
    }

    /** Returns true if the given tag ID belongs to the correct alliance's hub. */
    private boolean isHubTag(int tagId, boolean isRed) {
        int[] hubTags = isRed ? Constants.Vision.RED_HUB_TAGS : Constants.Vision.BLUE_HUB_TAGS;
        for (int id : hubTags) {
            if (id == tagId) return true;
        }
        return false;
    }

    /**
     * Converts a tag pose from camera space to robot space using the camera's mount pose.
     * Returns null if the pose data is invalid (all-zero NT default).
     */
    private Pose3d tagCamToRobotSpace(double[] arr, Pose3d camPose) {
        // Reject zero/near-zero arrays — NT default before real data arrives
        if (arr[0] * arr[0] + arr[1] * arr[1] + arr[2] * arr[2] < 0.01) {
            return null;
        }
        Pose3d tagInCam = new Pose3d(
            new Translation3d(arr[0], arr[1], arr[2]),
            new Rotation3d(Math.toRadians(arr[3]), Math.toRadians(arr[4]), Math.toRadians(arr[5]))
        );
        return camPose.transformBy(new Transform3d(tagInCam.getTranslation(), tagInCam.getRotation()));
    }

    /**
     * Returns the absolute field heading the robot should face to point its launcher toward
     * the score pillar. Only uses readings from the correct alliance's hub tags.
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
            int tagId = (int) limelightLeft.getEntry("tid").getDouble(-1);
            if (isHubTag(tagId, isRed)) {
                double[] arr = limelightLeft.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
                leftRobot = tagCamToRobotSpace(arr, camPosePrimary);
            }
        }
        if (tvRight == 1.0) {
            int tagId = (int) limelightRight.getEntry("tid").getDouble(-1);
            if (isHubTag(tagId, isRed)) {
                double[] arr = limelightRight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
                rightRobot = tagCamToRobotSpace(arr, camPoseSecondary);
            }
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

        double bearingRad = Math.atan2(ty, tx);
        lastTagRobotX = tx;
        lastTagRobotY = ty;
        lastBearingDeg = Math.toDegrees(bearingRad);

        // +180° because the launcher faces the back of the robot
        return swerve.getHeading().plus(new Rotation2d(bearingRad + Math.PI));
    }

    /**
     * Returns the 2D distance (meters) from the robot to the score pillar, or NaN if no target.
     * Only uses readings from the correct alliance's hub tags.
     */
    public double getDistanceToScorePillar(boolean isRed) {
        double tvLeft = limelightLeft.getEntry("tv").getDouble(0.0);
        double tvRight = limelightRight.getEntry("tv").getDouble(0.0);
        if (tvLeft < 0.5 && tvRight < 0.5) {
            return Double.NaN;
        }

        Pose3d leftRobot = null;
        Pose3d rightRobot = null;

        if (tvLeft == 1.0) {
            int tagId = (int) limelightLeft.getEntry("tid").getDouble(-1);
            if (isHubTag(tagId, isRed)) {
                double[] arr = limelightLeft.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
                leftRobot = tagCamToRobotSpace(arr, camPosePrimary);
            }
        }
        if (tvRight == 1.0) {
            int tagId = (int) limelightRight.getEntry("tid").getDouble(-1);
            if (isHubTag(tagId, isRed)) {
                double[] arr = limelightRight.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
                rightRobot = tagCamToRobotSpace(arr, camPoseSecondary);
            }
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
        SmartDashboard.putNumber("Vision/TagRobotX", lastTagRobotX);
        SmartDashboard.putNumber("Vision/TagRobotY", lastTagRobotY);
        SmartDashboard.putNumber("Vision/BearingDeg", lastBearingDeg);
        SmartDashboard.putNumber("Vision/RobotHeadingDeg", swerve.getHeading().getDegrees());
        SmartDashboard.putData("Vision/AutoAlignPID", Constants.Vision.rotationPID);
    }
}
