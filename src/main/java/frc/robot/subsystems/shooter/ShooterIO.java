package frc.robot.subsystems.shooter;

interface ShooterIO {
    void setRPM(double rpm);
    double getRPM();
    void stopShooter();
    void setFeeder(boolean on);
    boolean getFeeder();
}
