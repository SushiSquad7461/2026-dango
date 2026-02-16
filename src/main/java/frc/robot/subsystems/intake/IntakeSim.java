package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeSim implements IntakeIO{
    private IntakeState state = IntakeState.IDLE;

    public IntakeSim() {

    }

    @Override
    public void configurePivot() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configurePivot'");
    }

    @Override
    public void configureRoller() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configureRoller'");
    }

    @Override
    public void changeIfWiggle(boolean atTarget) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'changeState'");
    }

    @Override
    public void runPivotToTarget() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runPivotToTarget'");
    }

    @Override
    public void updateRollers() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateRollers'");
    }

    @Override
    public boolean isPivotAtTarget() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isPivotAtTarget'");
    }

    public void simulationPeriodic() {
        if (state == IntakeState.WIGGLING) {
            changeIfWiggle(isPivotAtTarget());
        }   
        SmartDashboard.putString("Intake/State", state.name());
     //   SmartDashboard.putNumber("Intake/PivotDeg", getPivotAngle());
     //   SmartDashboard.putNumber("Intake/PivotTargetDeg", pivotTargetDeg);
        SmartDashboard.putBoolean("Intake/PivotAtTarget", isPivotAtTarget());
    }

    

    @Override
    public void zeroPivot() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroPivot'");
    }

    @Override
    public double getPivotAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivotAngle'");
    }

    @Override
    public double getPivotTargetAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivotTargetAngle'");
    }

    @Override
    public void setState(IntakeState newState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setState'");
    }

    @Override
    public void runRollers() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runRollers'");
    }

    @Override
    public void stopRollers() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopRollers'");
    }
}
