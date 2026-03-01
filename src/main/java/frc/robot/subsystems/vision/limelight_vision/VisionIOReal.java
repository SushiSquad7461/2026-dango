package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOReal implements VisionIO {
    private final NetworkTable m_limelightTable;

    public VisionIOReal() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget = m_limelightTable.getEntry("tv").getDouble(0) == 1;
        inputs.tx = m_limelightTable.getEntry("tx").getDouble(0.0);
        inputs.ty = m_limelightTable.getEntry("ty").getDouble(0.0);
        inputs.ta = m_limelightTable.getEntry("ta").getDouble(0.0);
        inputs.tid = m_limelightTable.getEntry("tid").getInteger(-1);

        inputs.botPose = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        inputs.botPoseBlue = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        inputs.botPoseRed = m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);

        // Use latency from botpose index 6 if available, otherwise just use current
        // time
        if (inputs.botPose.length > 6) {
            inputs.timestamp = Timer.getFPGATimestamp() - (inputs.botPose[6] / 1000.0);
        } else {
            inputs.timestamp = Timer.getFPGATimestamp();
        }
    }
}
