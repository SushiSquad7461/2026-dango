package frc.robot.subsystems.shooter;

public class SOTMShotPhysicsSim {
    public static class Output {
        public double flywheelRPM = 0.0;
        public boolean shooterReady = false;
        public boolean shotActive = false;
        public double confidence = 0.0;
        public double tofSec = 0.0;
        public double dragCompensatedTofSec = 0.0;
    }

    private static final double DRAG_COEFF = 0.47;

    private final Output output = new Output();

    public Output update(double targetRPM, int feederDirection, double rpmTolerance, double dtSec) {
        double normalizedDt = dtSec / 0.02;
        if (normalizedDt < 0.0) normalizedDt = 0.0;
        if (normalizedDt > 1.0) normalizedDt = 1.0;
        double responseGain = 0.1 * normalizedDt;
        output.flywheelRPM += (targetRPM - output.flywheelRPM) * responseGain;

        double rpmError = Math.abs(output.flywheelRPM - targetRPM);
        output.shooterReady = rpmError < rpmTolerance;
        output.confidence = clamp(100.0 * (1.0 - (rpmError / (rpmTolerance * 2.0))), 0.0, 100.0);
        output.shotActive = feederDirection < 0 && output.shooterReady;

        if (output.shotActive) {
            double rps = Math.abs(output.flywheelRPM) / 60.0;
            output.tofSec = rps > 1e-6 ? (1.0 / rps) : 0.0;
            output.dragCompensatedTofSec = dragCompensatedTOF(output.tofSec);
        } else {
            output.tofSec = 0.0;
            output.dragCompensatedTofSec = 0.0;
        }

        return output;
    }

    private static double dragCompensatedTOF(double tofSec) {
        if (DRAG_COEFF < 1e-6) return tofSec;
        return (1.0 - Math.exp(-DRAG_COEFF * tofSec)) / DRAG_COEFF;
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
