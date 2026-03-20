package frc.robot.subsystems.shooter;

import frc.robot.generated.Constants;

public class ShooterIOReplay implements ShooterIO {
    private double targetRPM = 0.0;
    private double flywheelRPM = 0.0;

    @Override
    public void runShooter(double rpm) {
        targetRPM = rpm;
        flywheelRPM = rpm;
    }

    @Override
    public void stopShooter(double rpm) {
        targetRPM = rpm / 2.0;
        flywheelRPM = targetRPM;
    }

    @Override
    public void runFeeder() {}

    @Override
    public void runFeederBack() {}

    @Override
    public void stopFeeder() {}

    @Override
    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    @Override
    public double getFlywheelTargetRPM() {
        return targetRPM;
    }

    @Override
    public boolean isShooterReady() {
        return Math.abs(getFlywheelTargetRPM() - getFlywheelRPM()) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;
    }
}
