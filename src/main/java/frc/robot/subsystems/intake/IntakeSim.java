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

    // Matches IntakeReal roller supply current limit.
    private static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;

    public final IntakeData data = new IntakeData();

    // Pivot simulation
    private double pivotAngleDeg = 0.0;
    private double pivotTargetDeg = 0.0;
    private final double pivotToleranceDeg = 2.0;

    // Roller simulation
    private double rollerOutput = 0.0;

    // Wiggle logic
    private boolean wiggleHigh = false;
    private boolean wiggleReady = true;

    private IntakeState state = IntakeState.IDLE;

    @Override
    public void configurePivot() {
        // Nothing needed for sim
    }

    @Override
    public void configureRoller() {
        // Nothing needed for sim
    }

    @Override
    public void setState(IntakeState newState) {
        
    }

    @Override
    public void getMotorPos() {
        // Simulate encoder output (put on SmartDashboard if you want)
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
        data.appliedVolts = rollerOutput * 12.0;
        data.currentAmps = Math.abs(rollerOutput) * ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
    }

    @Override
    public void stopRollers() {
        rollerOutput = 0.0;
        data.appliedVolts = 0.0;
        data.currentAmps = 0.0;
    }

    @Override
    public void setStateRollers(double rollerSpeed) {
        rollerOutput = rollerSpeed;
        data.appliedVolts = rollerSpeed * 12.0;
        data.currentAmps = Math.abs(rollerSpeed) * ROLLER_SUPPLY_CURRENT_LIMIT_AMPS;
    }
}

