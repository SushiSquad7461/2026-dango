package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake.IntakeState;

public interface IntakeIO {
 

    //TODO: Clean up this code/Get rid of unnecessary methods
    void configurePivot();
    void configureRoller();
    void changeIfWiggle(boolean atTarget);
    void runPivotToTarget();
    void updateRollers();
    boolean isPivotAtTarget();
    double getPivotAngle();
    double getPivotTargetAngle();
    void zeroPivot();
    void runRollers();
    void stopRollers();
    void setState(IntakeState newState);
    void getMotorPos();
}
