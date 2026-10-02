package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;

public class Shooter extends SubsystemBase {
    private final ShooterHW flywheel = new ShooterHW();
    private final HoodHW hood = new HoodHW();
    private final ShotLUT lut = ShotTable.buildLUT();
    private double targetRPM = 0.0;

    public void setShot(double rpm, double hoodDegrees) {
        targetRPM = rpm;
        flywheel.setRPM(rpm);
        hood.setPosition(hoodDegrees);
    }

    public void stop() {
        targetRPM = 0.0;
        flywheel.stopShooter(500);
        flywheel.setFeeder(false);
    }

    public boolean isReady() {
        return targetRPM > 0
                && Math.abs(Math.abs(flywheel.getRPM()) - targetRPM) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;
    }

    public Command shoot(double rpm, double hoodDegrees) {
        return runOnce(() -> setShot(rpm, hoodDegrees))
                .andThen(Commands.waitUntil(() -> isReady()))
                .andThen(runOnce(() -> flywheel.setFeeder(true)));
                //.finallyDo(() -> stop());
    }

    /** Continuously aims RPM/hood from the LUT based on distance, feeding once ready. Runs until interrupted. */
    public Command autoShoot(DoubleSupplier distanceSupplier) {
        return run(() -> {
            ShotLUT.ShotParameters shot = lut.get(distanceSupplier.getAsDouble());
            setShot(shot.rpm(), shot.angle());
            if (isReady()) {
                flywheel.setFeeder(true);
            }
        });
    }

    public Command stepHood(double deltaDegrees) {
        return Commands.runOnce(() -> hood.stepHood(deltaDegrees));
    }

    public Command zeroHood() {
        return runOnce(() -> hood.zeroHood());
    }

    public Command idleRPM() {
        return runOnce(() -> flywheel.setRPM(Constants.Shooter.SHOOTER_IDLE_RPM));
    }
    
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", flywheel.getRPM());
        SmartDashboard.putNumber("Hood Position", hood.getPosition());
        SmartDashboard.putNumber("Target RPM", targetRPM);
        SmartDashboard.putBoolean("Feeder Status", flywheel.getFeeder());
        SmartDashboard.putBoolean("Shooter Ready", isReady());
        SmartDashboard.putNumber("Shooter Supply Current",flywheel.getSupplyCurrent());
        SmartDashboard.putNumber("Shooter Stator Current",flywheel.getStatorCurrent());
    }
}