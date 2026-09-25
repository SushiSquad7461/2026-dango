package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;

public class Shooter extends SubsystemBase {
    private final ShooterHW flywheel = new ShooterHW();
    private final HoodHW hood = new HoodHW();
    private double targetRPM = 0.0;

    public void setShot(double rpm, double hoodDegrees) {
        targetRPM = rpm;
        flywheel.setRPM(rpm);
        hood.setPosition(hoodDegrees);
    }

    public void stop() {
        targetRPM = 0.0;
        flywheel.stopShooter();
        flywheel.setFeeder(false);
    }

    public boolean isReady() {
        return targetRPM > 0
                && Math.abs(Math.abs(flywheel.getRPM()) - targetRPM) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;
    }

    public Command shoot(double rpm, double hoodDegrees) {
        return runOnce(() -> setShot(rpm, hoodDegrees))
                .andThen(Commands.waitUntil(() -> isReady()))
                .andThen(runOnce(() -> flywheel.setFeeder(true)))
                .andThen(Commands.idle())
                .finallyDo(() -> stop());
    }

    public Command stepHood(double deltaDegrees) {
        return Commands.runOnce(() -> hood.stepHood(deltaDegrees));
    }

    public Command zeroHood() {
        return runOnce(() -> hood.zeroHood());
    }
}