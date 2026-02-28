// Exposes methods for shooter control
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    void setFlywheelRPM(double rpm);
    void stopFlywheel();
    void runFeeder();
    void stopFeeder();
    double getFlywheelRPM();
    double getHoodPos();
    boolean isShooterReady();
}
