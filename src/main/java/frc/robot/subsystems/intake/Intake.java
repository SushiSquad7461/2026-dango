package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeState state;
    public enum IntakeState {
            IDLE(false, 0,IntakeConstants.stowedAngleDeg),
            DEPLOYED(true, 0.35,IntakeConstants.intakeAngleDeg),
            //ROLLERS_IN(true, 0.5),
            //ROLLERS_OUT(true, -0.5),

            //TODO: Tune wiggle angle
            WIGGLING(true, 0,80);
            // UPPER_WIGGLE(true, 0.35),
            // SLAM_DOWN(true,0.35);

            public final boolean intakeExtended;
            public final double rollerSpeed;
            public final double pivotAngle;

            private IntakeState(boolean extended, double rollerSpeed, double pivotAngle) {
                this.intakeExtended = extended;
                this.rollerSpeed = rollerSpeed;
                this.pivotAngle = pivotAngle;
            }
    }
    
    public Intake(IntakeIO io){
        io.zeroPivot();
        this.io = io;
        this.state = IntakeState.IDLE;
        io.setState(IntakeState.IDLE);
    }
    public boolean intakeAtTargetPos(){
        return (io.isPivotAtTarget());
    }
    public Command changeState(IntakeState newState) {
        return Commands.runOnce(()->{
            this.state = newState;
            if(newState == IntakeState.DEPLOYED){
                io.setStateRollers(newState.rollerSpeed*-1);
            }else{
                io.setStateRollers(0);
            }
            io.setState(newState);
        }, this).andThen(Commands.waitUntil(this::intakeAtTargetPos))
                .andThen(Commands.runOnce(()->io.setStateRollers(newState.rollerSpeed)));
    }

    @Override
    public void periodic(){
        // io.getMotorPos();
        //  io.runPivotToTarget();
        // io.changeIfWiggle(io.isPivotAtTarget());
        //  if (!manualRoll) {
        //      io.updateRollers();
        //  }
        SmartDashboard.putNumber("Intake/kP", IntakeConstants.pivotP);
        SmartDashboard.putString("Intake/State", state.name());
        SmartDashboard.putNumber("Intake/PivotDeg", io.getPivotAngle());
        SmartDashboard.putNumber("Intake/PivotTargetDeg", io.getPivotTargetAngle());
        SmartDashboard.putBoolean("Intake/PivotAtTarget", io.isPivotAtTarget());
    }
}
