package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
import frc.robot.generated.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private boolean wiggleUp = true;

    
    
    public enum IntakeState {
            IDLE(false, 0,IntakeConstants.stowedAngleDeg),
            DEPLOYED(true, 0.35,IntakeConstants.intakeAngleDeg),
            WIGGLING(true, 0,80);

            public final boolean intakeExtended;
            public final double rollerSpeed;
            public final double pivotAngle;

            private IntakeState(boolean extended, double rollerSpeed, double pivotAngle) {
                this.intakeExtended = extended;
                this.rollerSpeed = rollerSpeed;
                this.pivotAngle = pivotAngle;
            }
    }

    public enum IntakePivotState {
        IDLE,
        MOVING_TO_SETPOINT,
        AT_SETPOINT,
        SWITCHING_AGITATE_TARGET_HIGH,
        SWITCHING_AGITATE_TARGET_LOW
    }
    private IntakeState wantedState = IntakeState.IDLE;
    private IntakePivotState currentState = IntakePivotState.IDLE;
    private IntakePivotState previousState = IntakePivotState.IDLE;
    
    public Intake(IntakeIO io){
        io.zeroPivot();
        this.io = io;
        io.setState(IntakeState.IDLE.pivotAngle);
    }
    public boolean intakeAtTargetPos(){
        return (io.isPivotAtTarget());
    }
    public IntakeState getState(){
        return this.wantedState;
    }

    public void setWantedState(IntakeState state) {
        this.wantedState = state;
    }
    public void changeState() {
            previousState = this.currentState;
            if (this.wantedState == IntakeState.WIGGLING) {
                double target = (wiggleUp)? Constants.IntakeConstants.HIGH_WIGGLE_POSITION_DEGREES : Constants.IntakeConstants.LOW_WIGGLE_POSITION_DEGREES;
                boolean atTarget = io.isPivotAtSetpoint(target);
                if (previousState == IntakePivotState.MOVING_TO_SETPOINT && atTarget) {
                    currentState =
                        wiggleUp
                            ? IntakePivotState.SWITCHING_AGITATE_TARGET_LOW
                            : IntakePivotState.SWITCHING_AGITATE_TARGET_HIGH;
                } else if (currentState == IntakePivotState.SWITCHING_AGITATE_TARGET_HIGH) {
                    wiggleUp = true;
                    currentState = IntakePivotState.AT_SETPOINT;
                } else if (currentState == IntakePivotState.SWITCHING_AGITATE_TARGET_LOW) {
                    wiggleUp = false;
                    currentState = IntakePivotState.AT_SETPOINT;
                } else {
                    currentState =
                        atTarget
                            ? IntakePivotState.AT_SETPOINT
                            : IntakePivotState.MOVING_TO_SETPOINT;
                }
            }else if(wantedState == IntakeState.DEPLOYED){
                currentState = io.isPivotAtSetpoint(Constants.IntakeConstants.intakeAngleDeg)
                    ? IntakePivotState.AT_SETPOINT
                    : IntakePivotState.MOVING_TO_SETPOINT;
            }else{
                if (io.isPivotAtSetpoint(Constants.IntakeConstants.stowedAngleDeg)) {
                    currentState = IntakePivotState.AT_SETPOINT;
                } else {
                    currentState = IntakePivotState.MOVING_TO_SETPOINT;
                }
            }
    }
 
    private void applyState() {
        switch (currentState) {
            case MOVING_TO_SETPOINT:
                io.setState(getTargetPos());
                if(wantedState==IntakeState.DEPLOYED){
                    io.setStateRollers(wantedState.rollerSpeed*-1);
                }
                break;
            case AT_SETPOINT:
                io.setState(getTargetPos());
                if(wantedState==IntakeState.DEPLOYED){
                    io.setStateRollers(wantedState.rollerSpeed);
                }
                break;
            case SWITCHING_AGITATE_TARGET_HIGH:
                io.setState(getTargetPos());
                break;
            case SWITCHING_AGITATE_TARGET_LOW:
                io.setState(getTargetPos());
                break;
            case IDLE:
                io.setStateRollers(0);
                break;
            default:
                io.zeroPivot(); 
                break;
        }
    }

    private double getTargetPos() {
    switch (wantedState) {
        case WIGGLING:
            return wiggleUp ? Constants.IntakeConstants.HIGH_WIGGLE_POSITION_DEGREES 
                           : Constants.IntakeConstants.LOW_WIGGLE_POSITION_DEGREES;
        case DEPLOYED:
            return Constants.IntakeConstants.intakeAngleDeg;
        default:
            return Constants.IntakeConstants.stowedAngleDeg;
    }
}

    public double getSimulatedCurrentDrawAmps() {
        if (io instanceof IntakeSim simIo) {
            return simIo.data.currentAmps;
        }
        return 0.0;
    }

    @Override
    public void periodic(){
        if (DriverStation.isEnabled()) {
            changeState();
            applyState();
        }
        SmartDashboard.putNumber("Intake/kP", IntakeConstants.pivotP);
        SmartDashboard.putString("Intake/State", wantedState.name());
        SmartDashboard.putNumber("Intake/PivotDeg", io.getPivotAngle());
        SmartDashboard.putNumber("Intake/PivotTargetDeg", io.getPivotTargetAngle());
        SmartDashboard.putBoolean("Intake/PivotAtTarget", io.isPivotAtTarget());
    }
}
