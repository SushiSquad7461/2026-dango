package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public class ShooterIOSim implements ShooterIO {

    @AutoLog
    public static class ShooterData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public final ShooterData data = new ShooterData();

    // Matches ShooterIOKraken kicker supply current limit.
    private static final double FEEDER_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

    private double simulatedRPM = 0; 
    private double targetRPM = 0;  
    private boolean feederRunning = false;
    private double hoodPos = 0;    
    private final double RPM_TOLERANCE = 50;

    @Override
    public void runShooter(double rpm) {
        targetRPM = rpm;
        data.appliedVolts = rpm / 5000.0 * 12.0; 
    }


    @Override
    public void runFeeder() {
        feederRunning = true;
        data.currentAmps = FEEDER_SUPPLY_CURRENT_LIMIT_AMPS; 
    }

    @Override
    public void stopFeeder() {
        feederRunning = false;
        data.currentAmps = 0.0;
    }

    @Override
    public double getFlywheelRPM() {
        double diff = targetRPM - simulatedRPM;
        simulatedRPM += diff * 0.1; 
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
        return Math.abs(simulatedRPM - targetRPM) < RPM_TOLERANCE && feederRunning;
    }

    @Override
    public double getFlywheelTargetRPM() {
        return targetRPM;
    }

    @Override
    public void runFeederBack() {
        feederRunning = true;
        data.currentAmps = FEEDER_SUPPLY_CURRENT_LIMIT_AMPS;
    }

    @Override
    public void stopShooter(double rpm) {
        targetRPM = rpm / 2.0;
        data.appliedVolts = targetRPM / 5000.0 * 12.0;
    }

}
