package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Direction;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeState state;
    private boolean manualRoll;
    public enum IntakeState {
            IDLE(false, Direction.OFF),
            DEPLOYED(true, Direction.OFF),
            ROLLERS_IN(true, Direction.FORWARD),
            ROLLERS_OUT(true, Direction.REVERSE),
            WIGGLING(true, Direction.FORWARD);

            public final boolean intakeExtended;
            public final Direction direction;

            private IntakeState(boolean extended, Direction direction) {
                this.intakeExtended = extended;
                this.direction = direction;
            }
    }
    public Intake(IntakeIO io){
        this.io = io;
        this.state = IntakeState.IDLE;
        io.setState(IntakeState.IDLE);
    }
    public boolean intakeAtTargetPos(){
        return io.isPivotAtTarget();
    }
    public Command changeState(IntakeState newState) {
        return Commands.runOnce(()->{
            this.state = newState;
            io.setState(newState);
        }, this);

    }
    public Command runRollers(){
        // return Commands.runOnce(()->{
        //     io.runRollers();
        // });
       return Commands.startEnd(
        () -> {
            manualRoll = true;
            io.runRollers();
        },
        () -> {
            manualRoll = false;
            io.stopRollers();
        },
        this
    );
    }
    public Command stopRollers(){
        return Commands.runOnce(()->{
            io.stopRollers();
        });
    }

    @Override
    public void periodic(){
        io.runPivotToTarget();
        io.changeIfWiggle(io.isPivotAtTarget());
        if (!manualRoll) {
            io.updateRollers();
        }
        SmartDashboard.putString("Intake/State", state.name());
        SmartDashboard.putNumber("Intake/PivotDeg", io.getPivotAngle());
        SmartDashboard.putNumber("Intake/PivotTargetDeg", io.getPivotTargetAngle());
        SmartDashboard.putBoolean("Intake/PivotAtTarget", io.isPivotAtTarget());
    }
}
