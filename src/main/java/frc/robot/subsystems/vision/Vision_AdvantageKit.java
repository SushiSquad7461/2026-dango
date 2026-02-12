package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem that supports AdvantageKit logging and simulation.
 */
public class Vision_AdvantageKit extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    /**
     * Creates a new Vision_AdvantageKit subsystem.
     *
     * @param io The IO implementation to use (Real or Sim).
     */
    public Vision_AdvantageKit(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    /**
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public boolean getTV() {
        return inputs.hasTarget;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
     */
    public double getTX() {
        return inputs.tx;
    }

    /**
     * @return Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
     */
    public double getTY() {
        return inputs.ty;
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTA() {
        return inputs.ta;
    }

    /**
     * @return Primary AprilTag ID
     */
    public long getTID() {
        return inputs.tid;
    }

    /**
     * Calculates the robot's pose based on AprilTags.
     * This uses the standard botpose array.
     * 
     * @return The 2D pose of the robot or a default pose if no target is seen.
     */
    public Pose2d getBotPose2d() {
        if (inputs.hasTarget && inputs.botPose.length >= 6) {
            return new Pose2d(new Translation2d(inputs.botPose[0], inputs.botPose[1]),
                    Rotation2d.fromDegrees(inputs.botPose[5]));
        }
        return new Pose2d();
    }

    /**
     * Gets the pose relative to the Blue Alliance wall.
     * 
     * @return The 2D pose of the robot (Blue Origin)
     */
    public Pose2d getBotPoseBlue2d() {
        if (inputs.hasTarget && inputs.botPoseBlue.length >= 6) {
            return new Pose2d(new Translation2d(inputs.botPoseBlue[0], inputs.botPoseBlue[1]),
                    Rotation2d.fromDegrees(inputs.botPoseBlue[5]));
        }
        return new Pose2d();
    }

    /**
     * Gets the pose relative to the Red Alliance wall.
     * 
     * @return The 2D pose of the robot (Red Origin)
     */
    public Pose2d getBotPoseRed2d() {
        if (inputs.hasTarget && inputs.botPoseRed.length >= 6) {
            return new Pose2d(new Translation2d(inputs.botPoseRed[0], inputs.botPoseRed[1]),
                    Rotation2d.fromDegrees(inputs.botPoseRed[5]));
        }
        return new Pose2d();
    }
}
