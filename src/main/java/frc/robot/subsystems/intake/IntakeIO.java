package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake.IntakeState;

public interface IntakeIO {
    @AutoLog
    public class IntakeData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    void configurePivot();
    void configureRoller();
    void changeState(boolean atTarget);
    void runPivotToTarget();
    void updateRollers();
    boolean isPivotAtTarget();
}
