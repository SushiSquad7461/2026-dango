// Exposes methods for shooter control
package frc.robot.subsystems.shooter;

public interface ShooterIO {
    
    void runShooter(double rpm);
    void stopShooter();
    void runFeeder();
    void runFeederBack();
    void stopFeeder();
    double getFlywheelRPM();
    double getFlywheelTargetRPM();
    boolean isShooterReady();
}
