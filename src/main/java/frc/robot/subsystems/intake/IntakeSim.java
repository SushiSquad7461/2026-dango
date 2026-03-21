package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.generated.Constants.IntakeConstants;

public class IntakeSim implements IntakeIO{
 @AutoLog
    public static class IntakeData {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    private static final double PIVOT_SUPPLY_CURRENT_LIMIT_AMPS = 10.0;
    private static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MAX_PIVOT_VELOCITY_DEG_PER_SEC =
        (IntakeConstants.cruiseVelocityRps / IntakeConstants.motorRotationsPerArmRotation) * 360.0;
    private static final double MAX_PIVOT_ACCEL_DEG_PER_SEC2 =
        (IntakeConstants.accelRps2 / IntakeConstants.motorRotationsPerArmRotation) * 360.0;

    public final IntakeData data = new IntakeData();

    // Pivot simulation
    private double pivotAngleDeg = 0.0;
    private double pivotTargetDeg = 0.0;
    private final double pivotToleranceDeg = IntakeConstants.angleToleranceDeg;
    private double pivotVelocityDegPerSec = 0.0;
    private double pivotAppliedVolts = 0.0;
    private double lastUpdateSec = 0.0;
    private boolean hasTimestamp = false;

    // Roller simulation
    private double rollerOutput = 0.0;

    @Override
    public void configurePivot() {
        // Nothing needed for sim
    }

    @Override
    public void configureRoller() {
        // Nothing needed for sim
    }

    @Override
    public void setState(double newState) {
        pivotTargetDeg = newState;
        updatePivotModel();
        updateElectricalTelemetry();
    }

    @Override
    public void getMotorPos() {
        // Simulate encoder output (put on SmartDashboard if you want)
    }




    @Override
    public boolean isPivotAtTarget() {
        updatePivotModel();
        updateElectricalTelemetry();
        return Math.abs(pivotAngleDeg - pivotTargetDeg) <= pivotToleranceDeg;
    }

    @Override
    public double getPivotAngle() {
        updatePivotModel();
        updateElectricalTelemetry();
        return pivotAngleDeg;
    }

    @Override
    public double getPivotTargetAngle() {
        return pivotTargetDeg;
    }

    @Override
    public void zeroPivot() {
        pivotAngleDeg = 0.0;
        pivotTargetDeg = IntakeConstants.stowedAngleDeg;
        pivotVelocityDegPerSec = 0.0;
        pivotAppliedVolts = 0.0;
        updateElectricalTelemetry();
    }

    @Override
    public void runRollers() {
        rollerOutput = IntakeConstants.rollerSpeed;
        updateElectricalTelemetry();
    }

    @Override
    public void stopRollers() {
        rollerOutput = 0.0;
        updateElectricalTelemetry();
    }

    @Override
    public void setStateRollers(double rollerSpeed) {
        rollerOutput = rollerSpeed;
        updateElectricalTelemetry();
    }

    @Override
    public boolean isPivotAtSetpoint(double targetDeg) {
        updatePivotModel();
        updateElectricalTelemetry();
        return Math.abs(pivotAngleDeg - targetDeg) <= pivotToleranceDeg;
    }

    private void updatePivotModel() {
        double nowSec = Timer.getFPGATimestamp();
        if (!hasTimestamp) {
            lastUpdateSec = nowSec;
            hasTimestamp = true;
            return;
        }

        double dtSec = nowSec - lastUpdateSec;
        lastUpdateSec = nowSec;
        if (dtSec <= 0.0) return;

        double errorDeg = pivotTargetDeg - pivotAngleDeg;
        double desiredVelocityDegPerSec = 0.0;
        if (Math.abs(errorDeg) > pivotToleranceDeg) {
            desiredVelocityDegPerSec = Math.copySign(MAX_PIVOT_VELOCITY_DEG_PER_SEC, errorDeg);
        }

        double deltaVel = desiredVelocityDegPerSec - pivotVelocityDegPerSec;
        double maxDeltaVel = MAX_PIVOT_ACCEL_DEG_PER_SEC2 * dtSec;
        if (deltaVel > maxDeltaVel) deltaVel = maxDeltaVel;
        if (deltaVel < -maxDeltaVel) deltaVel = -maxDeltaVel;
        pivotVelocityDegPerSec += deltaVel;

        pivotAngleDeg += pivotVelocityDegPerSec * dtSec;
        double newErrorDeg = pivotTargetDeg - pivotAngleDeg;
        if (Math.signum(errorDeg) != Math.signum(newErrorDeg) || Math.abs(newErrorDeg) <= pivotToleranceDeg) {
            pivotAngleDeg = pivotTargetDeg;
            pivotVelocityDegPerSec = 0.0;
        }
    }

    private void updateElectricalTelemetry() {
        double rollerAppliedVolts = clamp(rollerOutput * NOMINAL_VOLTAGE, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
        double pivotVelocityRatio = MAX_PIVOT_VELOCITY_DEG_PER_SEC > 0.0
            ? Math.abs(pivotVelocityDegPerSec) / MAX_PIVOT_VELOCITY_DEG_PER_SEC
            : 0.0;
        if (pivotVelocityRatio > 1.0) pivotVelocityRatio = 1.0;
        pivotAppliedVolts = Math.copySign(pivotVelocityRatio * NOMINAL_VOLTAGE, pivotVelocityDegPerSec);

        double rollerCurrentAmps = (Math.abs(rollerAppliedVolts) / NOMINAL_VOLTAGE) * ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
        double pivotCurrentAmps = (Math.abs(pivotAppliedVolts) / NOMINAL_VOLTAGE) * PIVOT_SUPPLY_CURRENT_LIMIT_AMPS;

        data.appliedVolts = Math.abs(rollerAppliedVolts) >= Math.abs(pivotAppliedVolts)
            ? rollerAppliedVolts
            : pivotAppliedVolts;
        data.currentAmps = rollerCurrentAmps + pivotCurrentAmps;
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
