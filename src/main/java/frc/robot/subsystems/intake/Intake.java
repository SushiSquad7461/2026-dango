package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.generated.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeState state;
    
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
    
    public Intake(IntakeIO io){
        io.zeroPivot();
        this.io = io;
        this.state = IntakeState.IDLE;
        io.setState(IntakeState.IDLE);
    }
    public boolean intakeAtTargetPos(){
        return (io.isPivotAtTarget());
    }
    public IntakeState getState(){
        return this.state;
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

    public double getSimulatedCurrentDrawAmps() {
        if (io instanceof IntakeSim simIo) {
            return simIo.data.currentAmps;
        }
        return 0.0;
    }

    @Override
    public void periodic(){
        // io.getMotorPos();
        //  io.runPivotToTarget();
        // io.changeIfWiggle(io.isPivotAtTarget());
        //  if (!manualRoll) {
        //      io.updateRollers();
        //  }
        double pivotDeg = io.getPivotAngle();
        double pivotTargetDeg = io.getPivotTargetAngle();
        boolean pivotAtTarget = io.isPivotAtTarget();
        SmartDashboard.putNumber("Intake/kP", IntakeConstants.pivotP);
        SmartDashboard.putString("Intake/State", state.name());
        SmartDashboard.putNumber("Intake/PivotDeg", pivotDeg);
        SmartDashboard.putNumber("Intake/PivotTargetDeg", pivotTargetDeg);
        SmartDashboard.putBoolean("Intake/PivotAtTarget", pivotAtTarget);
        if (Constants.currentMode != Constants.Mode.REAL) {
            Logger.recordOutput("Intake/State", state.name());
            Logger.recordOutput("Intake/PivotDeg", pivotDeg);
            Logger.recordOutput("Intake/PivotTargetDeg", pivotTargetDeg);
            Logger.recordOutput("Intake/PivotAtTarget", pivotAtTarget);
            if (io instanceof IntakeSim simIo) {
                Logger.recordOutput("Intake/AppliedVolts", simIo.data.appliedVolts);
                Logger.recordOutput("Intake/CurrentAmps", simIo.data.currentAmps);
            }
        }
    }
}
