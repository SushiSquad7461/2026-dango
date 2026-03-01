package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    // Reads from the primary Limelight which has been configured in the UI as a
    // "Limelight Vision Array". This camera will pull tag corners from the
    // secondary
    // camera and compute a single, unified MegaTag pose.
    private final NetworkTable limelightTable;

    public Vision() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Gets the latest, fully fused 3D pose from the Limelight array.
     * Uses the Megatag "botpose_wpiblue" array.
     * 
     * @return The fused Pose3d, or null if no tags are seen across any cameras in
     *         the array.
     */
    public Pose3d getEstimatedGlobalPose() {
        if (limelightTable.getEntry("tv").getDouble(0) != 1) {
            return null; // No target seen by any camera in the array
        }

        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        if (botpose.length < 6) {
            return null; // Malformed data
        }

        // botpose array format: [X, Y, Z, Roll, Pitch, Yaw(Angle)]
        return new Pose3d(
                botpose[0], // X
                botpose[1], // Y
                botpose[2], // Z
                new Rotation3d(
                        Math.toRadians(botpose[3]), // Roll
                        Math.toRadians(botpose[4]), // Pitch
                        Math.toRadians(botpose[5]) // Yaw
                ));
    }

    // TODO: feed getEstimatedGlobalPose() directly into the Swerve Drive's
    // SwerveDrivePoseEstimator in periodic().

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}