package frc.robot.subsystems.shooter;

public interface ShooterIO {
    void setFlywheelRPM(double rpm);
    void stopFlywheel();
    void runFeeder();
    void stopFeeder();
    double getFlywheelRPM();
}

