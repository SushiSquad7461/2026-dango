package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOSim implements VisionIO {
    public VisionIOSim() {
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Simulate a target always being visible
        inputs.hasTarget = true;

        // Simulate some changing data (e.g., robot moving in a circle)
        double timestamp = Timer.getFPGATimestamp();
        inputs.tx = Math.sin(timestamp) * 10.0;
        inputs.ty = Math.cos(timestamp) * 10.0;
        inputs.ta = 5.0; // Constant area
        inputs.tid = 1;

        // Simulate bot pose
        inputs.botPose = new double[] { 5.0 + Math.sin(timestamp), 5.0 + Math.cos(timestamp), 0.0, 0.0, 0.0,
                (timestamp * 20) % 360 };
        inputs.botPoseBlue = inputs.botPose;
        inputs.botPoseRed = inputs.botPose; // Initialize with same theoretical pose for sim simplicity

        inputs.timestamp = timestamp;
    }
}
