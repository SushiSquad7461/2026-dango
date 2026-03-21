package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.Intake.IntakeState;

public interface IntakeIO {
 

    //TODO: Clean up this code/Get rid of unnecessary methods
    void configurePivot();
    void configureRoller();

    boolean isPivotAtTarget();
    double getPivotAngle();
    double getPivotTargetAngle();
    void zeroPivot();
    void runRollers();
    void stopRollers();
    void setState(IntakeState newState);
    void getMotorPos();
     void setStateRollers(double rollerSpeed);
}
