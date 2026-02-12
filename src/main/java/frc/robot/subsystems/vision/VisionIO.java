package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public double tx = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;
        public long tid = -1;
        public double[] botPose = new double[] {};
        public double[] botPoseBlue = new double[] {};
        public double[] botPoseRed = new double[] {};
        public double timestamp = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
