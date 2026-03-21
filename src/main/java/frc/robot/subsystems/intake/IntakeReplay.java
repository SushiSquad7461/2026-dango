package frc.robot.subsystems.intake;

import frc.robot.generated.Constants.IntakeConstants;

public class IntakeReplay implements IntakeIO {
    private double pivotAngleDeg = IntakeConstants.stowedAngleDeg;
    private double pivotTargetDeg = IntakeConstants.stowedAngleDeg;
    private double rollerSpeed = 0.0;

    @Override
    public void configurePivot() {}

    @Override
    public void configureRoller() {}

    @Override
    public boolean isPivotAtTarget() {
        return Math.abs(pivotAngleDeg - pivotTargetDeg) <= IntakeConstants.angleToleranceDeg;
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
        rollerSpeed = IntakeConstants.rollerSpeed;
    }

    @Override
    public void stopRollers() {
        rollerSpeed = 0.0;
    }

    @Override
    public void setState(double newState) {
        pivotTargetDeg = newState;
        pivotAngleDeg = pivotTargetDeg;
    }

    @Override
    public void getMotorPos() {}

    @Override
    public void setStateRollers(double rollerSpeed) {
        this.rollerSpeed = rollerSpeed;
    }

    @Override
    public boolean isPivotAtSetpoint(double targetDeg) {
        return Math.abs(pivotAngleDeg - targetDeg) <= IntakeConstants.angleToleranceDeg;
    }
}
