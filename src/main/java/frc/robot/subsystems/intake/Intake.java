package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.util.Direction;

public class Intake {
    private IntakeIO io;
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
    }
    public boolean intakeAtTargetPos(){
        return io.isPivotAtTarget();
    }
    public Command changeState(IntakeState newState) {
        return Commands.runOnce(()->{
            io.changeState(io.isPivotAtTarget());
            io.runPivotToTarget();
            io.updateRollers();
        });

    }
}
