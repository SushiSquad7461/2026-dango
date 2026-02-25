package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Direction;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeState state;
    public enum IntakeState {
            IDLE(false, 0),
            DEPLOYED(true, 0),
            ROLLERS_IN(true, 0.5),
            ROLLERS_OUT(true, -0.5);

            public final boolean intakeExtended;
            public final double rollerSpeed;


            private IntakeState(boolean extended, double speed) {
                this.intakeExtended = extended;
                this.rollerSpeed = speed;
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
         return Commands.run(()->{
             io.runRollers();
         });

    }
    public Command stopRollers(){
        return Commands.runOnce(()->{
            io.stopRollers();
        });
    }

    @Override
    public void periodic(){
        // io.getMotorPos();
        //  io.runPivotToTarget();
        // io.changeIfWiggle(io.isPivotAtTarget());
        //  if (!manualRoll) {
        //      io.updateRollers();
        //  }
        SmartDashboard.putString("Intake/State", state.name());
        SmartDashboard.putNumber("Intake/PivotDeg", io.getPivotAngle());
        SmartDashboard.putNumber("Intake/PivotTargetDeg", io.getPivotTargetAngle());
        SmartDashboard.putBoolean("Intake/PivotAtTarget", io.isPivotAtTarget());
    }
}
