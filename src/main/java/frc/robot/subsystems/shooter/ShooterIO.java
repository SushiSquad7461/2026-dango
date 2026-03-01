// Exposes methods for shooter control
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    void setFlywheelRPM(double rpm);
    void stopFlywheel();
    void runFeeder();
    void stopFeeder();
    double getFlywheelRPM();
    double getHoodPos();
    boolean isShooterReady();
}