package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.Constants;

public class ShooterIOSim implements ShooterIO {

    @AutoLog
    public static class ShooterData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double legacyFlywheelRPM = 0.0;
        public boolean legacyReady = false;
        public double sotmFlywheelRPM = 0.0;
        public boolean sotmReady = false;
        public boolean sotmShotActive = false;
        public double sotmConfidence = 0.0;
        public double sotmTofSec = 0.0;
        public double sotmDragCompensatedTofSec = 0.0;
        public boolean shotSourceIsSotm = true;
        public Pose3d sotmNotePose = new Pose3d();
        public Pose3d[] sotmTrajectory = new Pose3d[] {};
    }

    public final ShooterData data = new ShooterData();

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;
    private static final double FEEDER_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
    private static final double HOOD_SUPPLY_CURRENT_LIMIT_AMPS = 10.0;
    private static final double HOOD_MAX_VELOCITY_DEG_PER_SEC =
        (Constants.HoodedShooterConstants.cruiseVelocityRps / Constants.IntakeConstants.motorRotationsPerArmRotation) * 360.0;

    private double legacySimulatedRPM = 0; 
    private double targetRPM = 0;  
    private int feederDirection = 0;
    private double flywheelAppliedVolts = 0.0;
    private double hoodPos = 0;    
    private double hoodVelocityCmd = 0.0;
    private double robotVxMetersPerSecond = 0.0;
    private double robotVyMetersPerSecond = 0.0;
    private double lastModelUpdateSec = Timer.getFPGATimestamp();
    private final SOTMShotPhysicsSim sotmShotSim = new SOTMShotPhysicsSim();

    @Override
    public void runShooter(double rpm) {
        targetRPM = rpm;
        flywheelAppliedVolts = calculateFlywheelAppliedVolts(targetRPM);
        data.appliedVolts = flywheelAppliedVolts;
        updateShotModels();
        updateCurrentDraw();
    }


    @Override
    public void runFeeder() {
        feederDirection = -1;
        updateShotModels();
        updateCurrentDraw();
    }

    @Override
    public void stopFeeder() {
        feederDirection = 0;
        updateShotModels();
        updateCurrentDraw();
    }

    @Override
    public double getFlywheelRPM() {
        updateShotModels();
        updateCurrentDraw();
        return data.sotmFlywheelRPM;
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
    public void runHood(double speed) {
        hoodVelocityCmd = speed;
        updateShotModels();
        updateCurrentDraw();
    }

    @Override
    public void stopHood() {
        hoodVelocityCmd = 0.0;
        updateShotModels();
        updateCurrentDraw();
    }

    @Override
    public void setRobotVelocity(double vxMetersPerSecond, double vyMetersPerSecond) {
        robotVxMetersPerSecond = vxMetersPerSecond;
        robotVyMetersPerSecond = vyMetersPerSecond;
    }

    @Override
    public boolean isShooterReady() {
        updateShotModels();
        return data.sotmReady;
    }

    @Override
    public double getFlywheelTargetRPM() {
        return targetRPM;
    }

    @Override
    public void runFeederBack() {
        feederDirection = 1;
        updateShotModels();
        updateCurrentDraw();
    }

    @Override
    public void stopShooter(double rpm) {
        targetRPM = rpm / 2.0;
        flywheelAppliedVolts = calculateFlywheelAppliedVolts(targetRPM);
        data.appliedVolts = flywheelAppliedVolts;
        updateShotModels();
        updateCurrentDraw();
    }

    private void updateShotModels() {
        double nowSec = Timer.getFPGATimestamp();
        double dtSec = nowSec - lastModelUpdateSec;
        if (dtSec <= 0.0) {
            data.legacyFlywheelRPM = legacySimulatedRPM;
            data.legacyReady =
                Math.abs(legacySimulatedRPM - targetRPM) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;
            return;
        }
        if (dtSec > 0.1) dtSec = 0.1;
        lastModelUpdateSec = nowSec;

        hoodPos += hoodVelocityCmd * HOOD_MAX_VELOCITY_DEG_PER_SEC * dtSec;
        if (hoodPos < 0.0) hoodPos = 0.0;
        if (hoodPos > 90.0) hoodPos = 90.0;

        double legacyGain = 0.1 * (dtSec / 0.02);
        if (legacyGain < 0.0) legacyGain = 0.0;
        if (legacyGain > 1.0) legacyGain = 1.0;
        legacySimulatedRPM += (targetRPM - legacySimulatedRPM) * legacyGain;
        data.legacyFlywheelRPM = legacySimulatedRPM;
        data.legacyReady =
            Math.abs(legacySimulatedRPM - targetRPM) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;

        var sotmOutput =
            sotmShotSim.update(
                targetRPM,
                feederDirection,
                Constants.Shooter.SHOOTER_RPM_TOLERANCE,
                dtSec,
                hoodPos,
                Constants.Vision.targetDistanceMeters,
                robotVxMetersPerSecond,
                robotVyMetersPerSecond);
        data.sotmFlywheelRPM = sotmOutput.flywheelRPM;
        data.sotmReady = sotmOutput.shooterReady;
        data.sotmShotActive = sotmOutput.shotActive;
        data.sotmConfidence = sotmOutput.confidence;
        data.sotmTofSec = sotmOutput.tofSec;
        data.sotmDragCompensatedTofSec = sotmOutput.dragCompensatedTofSec;
        data.sotmNotePose = sotmOutput.notePose;
        data.sotmTrajectory = sotmOutput.trajectory;
        data.shotSourceIsSotm = true;
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
        double hoodAppliedVolts = clamp(hoodVelocityCmd * NOMINAL_VOLTAGE, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
        double shooterCurrentAmps =
            (Math.abs(flywheelAppliedVolts) / NOMINAL_VOLTAGE) * SHOOTER_SUPPLY_CURRENT_LIMIT_AMPS;
        double feederCurrentAmps =
            (Math.abs(feederAppliedVolts) / NOMINAL_VOLTAGE) * FEEDER_SUPPLY_CURRENT_LIMIT_AMPS;
        double hoodCurrentAmps =
            (Math.abs(hoodAppliedVolts) / NOMINAL_VOLTAGE) * HOOD_SUPPLY_CURRENT_LIMIT_AMPS;
        data.currentAmps = shooterCurrentAmps + feederCurrentAmps + hoodCurrentAmps;
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
