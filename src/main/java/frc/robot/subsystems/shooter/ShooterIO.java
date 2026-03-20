// Exposes methods for shooter control
package frc.robot.subsystems.shooter;

public interface ShooterIO {
    
    void runShooter(double rpm);
    void stopShooter(double rpm);
    void runFeeder();
    void runFeederBack();
    void stopFeeder();
    double getFlywheelRPM();
    double getFlywheelTargetRPM();
    boolean isShooterReady();
    default void setRobotVelocity(double vxMetersPerSecond, double vyMetersPerSecond) {}
    default void runHood(double speed) {}
    default void stopHood() {}
}
