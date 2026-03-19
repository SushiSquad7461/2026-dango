package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PassCalculator;
import frc.robot.subsystems.vision.Vision;

/**
 * Command that executes a pass from the central zone into the alliance zone.
 * Follows the same pattern as AutoAlign: rotates toward target, commands shooter
 * directly, driver retains translational control.
 */
public class AutoPass extends Command {

    private final Swerve swerve;
    private final Vision vision;
    private final ShooterSubsystem shooter;
    private final HoodedShooter hoodedShooter;
    private final Hopper hopper;
    private final DoubleSupplier xTranslation;
    private final DoubleSupplier yTranslation;
    private final DoubleSupplier driverRotation;
    private boolean hasFed = false;

    public AutoPass(Swerve swerve, Vision vision, ShooterSubsystem shooter, HoodedShooter hoodedShooter,
            Hopper hopper,
            DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier driverRotation) {
        this.swerve = swerve;
        this.vision = vision;
        this.shooter = shooter;
        this.hoodedShooter = hoodedShooter;
        this.hopper = hopper;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.driverRotation = driverRotation;
        // Only require swerve — same reasoning as AutoAlign:
        // avoids conflict with StateMachine commands on shooter/hood.
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Constants.Vision.rotationPID.reset();
        hasFed = false;
    }

    @Override
    public void execute() {
        // Driver translational input — same deadband/cubic/scaling as TeleopSwerve and AutoAlign
        double x = MathUtil.applyDeadband(xTranslation.getAsDouble(), Constants.stickDeadband);
        double y = MathUtil.applyDeadband(yTranslation.getAsDouble(), Constants.stickDeadband);
        Translation2d raw = new Translation2d(x, y);
        double magnitude = raw.getNorm();
        Translation2d driverInput = raw.times(Math.pow(magnitude, 2)).times(Constants.Swerve.maxSpeed);

        PassCalculator.PassParameters pass = vision.getCurrentPass();

        if (pass.isValid()) {
            // Rotate toward pass target.
            // The shooter faces backward (π offset), so we rotate the aim angle by 180°.
            double currentHeading = swerve.getHeading().getDegrees();
            double targetHeading  = Math.toDegrees(pass.aimAngleRad()) + 180.0;
            double pidOutput      = Constants.Vision.rotationPID.calculate(currentHeading, targetHeading);
            double rotationSpeed  = MathUtil.clamp(pidOutput,
                -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);

            // Normalize heading error to [-180, 180] using WPILib utility
            double headingError = Math.abs(MathUtil.inputModulus(currentHeading - targetHeading, -180, 180));

            // Always spin up shooter and set hood when heading is close
            if (headingError < 15.0) {
                shooter.commandRPM(pass.rpm());
                hoodedShooter.moveHoodToAngleWithOffset(pass.hoodAngleDeg());

                // Feed ball once shooter is at speed (one-shot: don't re-feed)
                if (!hasFed && shooter.isShooterReady()) {
                    hopper.runHopper().schedule();
                    shooter.runFeeder().schedule();
                    hasFed = true;
                }
            } else {
                // Still turning — keep shooter warm
                shooter.setTargetRPM(pass.rpm());
            }

            swerve.drive(driverInput, rotationSpeed, true, true);

            SmartDashboard.putNumber("Pass/RPM", pass.rpm());
            SmartDashboard.putNumber("Pass/HoodAngleDeg", pass.hoodAngleDeg());
            SmartDashboard.putNumber("Pass/Distance", pass.distance());
            SmartDashboard.putNumber("Pass/AimAngleDeg", Math.toDegrees(pass.aimAngleRad()));
            SmartDashboard.putNumber("Pass/HeadingErrorDeg", headingError);
            SmartDashboard.putBoolean("Pass/Blocked", pass.isBlocked());
        } else {
            // Not in central zone or no valid pass — give driver full control
            double rotation = MathUtil.applyDeadband(driverRotation.getAsDouble(), Constants.stickDeadband);
            rotation = Math.pow(rotation, 3) * Constants.Swerve.maxAngularVelocity;
            swerve.drive(driverInput, rotation, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs until button released (whileTrue in RobotContainer)
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, true);
        hopper.stopHopper().schedule();
        shooter.stopFeeder().schedule();
    }
}
