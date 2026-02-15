package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    void configurePivot();
    void configureRoller();
    void changeIfWiggle(boolean atTarget);
    void runPivotToTarget();
    void updateRollers();
    boolean isPivotAtTarget();
}
