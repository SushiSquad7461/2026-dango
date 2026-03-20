package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.generated.Constants;

public class ShooterIOSim implements ShooterIO {

    @AutoLog
    public static class ShooterData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public final ShooterData data = new ShooterData();

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    private static final double FEEDER_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    private double simulatedRPM = 0; 
    private double targetRPM = 0;  
    private int feederDirection = 0;
    private double flywheelAppliedVolts = 0.0;
    private double hoodPos = 0;    

    @Override
    public void runShooter(double rpm) {
        targetRPM = rpm;
        flywheelAppliedVolts = calculateFlywheelAppliedVolts(targetRPM);
        data.appliedVolts = flywheelAppliedVolts;
        updateCurrentDraw();
    }


    @Override
    public void runFeeder() {
        feederDirection = -1;
        updateCurrentDraw();
    }

    @Override
    public void stopFeeder() {
        feederDirection = 0;
        updateCurrentDraw();
    }

    @Override
    public double getFlywheelRPM() {
        double diff = targetRPM - simulatedRPM;
        simulatedRPM += diff * 0.1; 
        updateCurrentDraw();
        return simulatedRPM;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public void setHoodPos(double pos) {
        if (pos < 0) pos = 0;
        if (pos > 90) pos = 90;
        hoodPos = pos;
    }

    @Override
    public boolean isShooterReady() {
        return Math.abs(simulatedRPM - targetRPM) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;
    }

    @Override
    public double getFlywheelTargetRPM() {
        return targetRPM;
    }

    @Override
    public void runFeederBack() {
        feederDirection = 1;
        updateCurrentDraw();
    }

    @Override
    public void stopShooter(double rpm) {
        targetRPM = rpm / 2.0;
        flywheelAppliedVolts = calculateFlywheelAppliedVolts(targetRPM);
        data.appliedVolts = flywheelAppliedVolts;
        updateCurrentDraw();
    }

    private double calculateFlywheelAppliedVolts(double rpm) {
        double rps = rpm / 60.0;
        if (Math.abs(rps) < 1e-9) return 0.0;

        double volts =
            Math.copySign(Constants.Shooter.SHOOTER_KS, rps)
                + (Constants.Shooter.SHOOTER_KV * rps);
        return clamp(volts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    }

    private double calculateFeederAppliedVolts() {
        if (feederDirection == 0) return 0.0;

        double rps = feederDirection * (Constants.Shooter.FEEDER_RPM / 60.0);
        double volts =
            Math.copySign(Constants.Shooter.KICKER_KS, rps)
                + (Constants.Shooter.KICKER_KV * rps);
        return clamp(volts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    }

    private void updateCurrentDraw() {
        double feederAppliedVolts = calculateFeederAppliedVolts();
        double shooterCurrentAmps =
            (Math.abs(flywheelAppliedVolts) / NOMINAL_VOLTAGE) * SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS;
        double feederCurrentAmps =
            (Math.abs(feederAppliedVolts) / NOMINAL_VOLTAGE) * FEEDER_SUPPLY_CURRENT_LIMIT_AMPS;
        data.currentAmps = shooterCurrentAmps + feederCurrentAmps;
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
