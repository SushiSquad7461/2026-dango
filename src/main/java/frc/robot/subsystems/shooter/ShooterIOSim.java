package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public class ShooterIOSim implements ShooterIO {

    @AutoLog
    public static class ShooterData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public final ShooterData data = new ShooterData();

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
        data.currentAmps = 5.0; 
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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelTargetRPM'");
    }

    @Override
    public void runFeederBack() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runFeederBack'");
    }

    @Override
    public void stopShooter(double rpm) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopShooter'");
    }

}