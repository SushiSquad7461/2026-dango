package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void setRPM(double rpm);
    public double getRPM();
    public void stopShooter();
    public void setFeeder(boolean on);
    public boolean getFeeder();
}
