package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class Vision extends SubsystemBase {

    private final NetworkTable m_limelightTable;

    public Vision() {
        // Get the default instance of NetworkTables and the table for the limelight
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean getTV() {
        return m_limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
     */
    public double getTX() {
        return m_limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
     */
    public double getTY() {
        return m_limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTA() {
        return m_limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * @return Primary AprilTag ID
     */
    public long getTID() {
        return m_limelightTable.getEntry("tid").getInteger(-1);
    }

    /**
     * @return Robot transform in field-space (MegaTag)
     */
    public double[] getBotPose() {
        return m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    /**
     * @return Robot transform in field-space (Blue Origin)
     */
    public double[] getBotPoseWPIBlue() {
        return m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    /**
     * @return Robot transform in field-space (Red Origin)
     */
    public double[] getBotPoseWPIRed() {
        return m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }

    /**
     * Calculates the robot's pose based on AprilTags.
     * This uses the standard botpose array.
     * @return The 2D pose of the robot or a default pose if no target is seen.
     */
    public Pose2d getBotPose2d() {
        double[] botpose = getBotPose();
        if (getTV()) {
          // Extracts x, y, and rotation from the array
          return new Pose2d(new Translation2d(botpose[0], botpose[1]), Rotation2d.fromDegrees(botpose[5]));
        }
        return new Pose2d();
    }

    /**
     * Gets the pose relative to the Blue Alliance wall.
     * @return The 2D pose of the robot (Blue Origin)
     */
    public Pose2d getBotPoseBlue2d() {
        double[] botpose = getBotPoseWPIBlue();
        if (getTV()) {
            return new Pose2d(new Translation2d(botpose[0], botpose[1]), Rotation2d.fromDegrees(botpose[5]));
        }
        return new Pose2d();
    }

    /**
     * Gets the pose relative to the Red Alliance wall.
     * @return The 2D pose of the robot (Red Origin)
     */
    public Pose2d getBotPoseRed2d() {
        double[] botpose = getBotPoseWPIRed();
        if (getTV()) {
            return new Pose2d(new Translation2d(botpose[0], botpose[1]), Rotation2d.fromDegrees(botpose[5]));
        }
        return new Pose2d();
    }

    /**
     * Sets the vision pipeline.
     * @param pipelineIndex The index of the pipeline to use (0-9)
     */
    public void setPipeline(int pipelineIndex) {
        m_limelightTable.getEntry("pipeline").setNumber(pipelineIndex);
    }

    /**
     * Sets the LED mode.
     * @param mode 0: pipeline default, 1: off, 2: blink, 3: on
     */
    public void setLedMode(int mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the camera mode.
     * @param mode 0: Vision processor, 1: Driver Camera (Increases exposure, disables vision processing)
     */
    public void setCamMode(int mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
