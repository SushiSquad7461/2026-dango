package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSim implements IntakeIO{
 @AutoLog
    public static class IntakeData {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public final IntakeData data = new IntakeData();

    private double pivotAngleDeg = 0.0;
    private double pivotTargetDeg = 0.0;
    private final double pivotToleranceDeg = 2.0;

    private double rollerOutput = 0.0;

    private boolean wiggleHigh = false;
    private boolean wiggleReady = true;

    private IntakeState state = IntakeState.IDLE;

    @Override
    public void configurePivot() {
    }

    @Override
    public void configureRoller() {
    }

    @Override
    public void setState(double newState) {
        pivotTargetDeg = newState;
        pivotAngleDeg += Math.signum(pivotTargetDeg - pivotAngleDeg)
                * Math.min(4.0, Math.abs(pivotTargetDeg - pivotAngleDeg));
    }

    @Override
    public void getMotorPos() {
    }

    @Override
    public boolean isPivotAtTarget() {
        return Math.abs(pivotAngleDeg - pivotTargetDeg) <= pivotToleranceDeg;
    }

    @Override
    public double getPivotAngle() {
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
    }

    @Override
    public void runRollers() {
        rollerOutput = IntakeConstants.rollerSpeed;
    }

    @Override
    public void stopRollers() {
        rollerOutput = 0.0;
    }

    @Override
    public void setStateRollers(double rollerSpeed) {
        rollerOutput = rollerSpeed;
    }

    @Override
    public boolean isPivotAtSetpoint(double targetDeg) {
        return Math.abs(pivotAngleDeg - targetDeg) <= pivotToleranceDeg;
    }
}