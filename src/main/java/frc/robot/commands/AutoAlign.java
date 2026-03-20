package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

/**
 * Points the robot at the hub using raw Limelight tx, and commands
 * shooter RPM + hood angle from the ty-based distance lookup.
 *
 * Driver keeps full translation control; rotation is overridden by the PID.
 */
public class AutoAlign extends Command {

    private final Swerve swerve;
    private final Vision vision;
    private final ShooterSubsystem shooter;
    private final HoodedShooter hoodedShooter;
    private final DoubleSupplier xTranslation;
    private final DoubleSupplier yTranslation;
    private final DoubleSupplier driverRotation;
    private final PIDController txPID;

    public AutoAlign(Swerve swerve, Vision vision, ShooterSubsystem shooter, HoodedShooter hoodedShooter,
            DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier driverRotation) {
        this.swerve = swerve;
        this.vision = vision;
        this.shooter = shooter;
        this.hoodedShooter = hoodedShooter;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.driverRotation = driverRotation;
        addRequirements(swerve);

        // PID on shooter error (degrees). 0 = shooter axis aligned with target.
        txPID = new PIDController(
                Constants.Vision.rotationPID.getP(),
                Constants.Vision.rotationPID.getI(),
                Constants.Vision.rotationPID.getD());
        txPID.setSetpoint(0);
        txPID.setTolerance(2.0);
    }

    @Override
    public void initialize() {
        txPID.reset();
    }

    @Override
    public void execute() {
        // Driver translation (same deadband + cubic + maxSpeed as TeleopSwerve).
        double x = MathUtil.applyDeadband(xTranslation.getAsDouble(), Constants.stickDeadband);
        double y = MathUtil.applyDeadband(yTranslation.getAsDouble(), Constants.stickDeadband);
        Translation2d raw = new Translation2d(x, y);
        double magnitude = raw.getNorm();
        Translation2d driverInput = raw.times(Math.pow(magnitude, 2)).times(Constants.Swerve.maxSpeed);

        if (vision.hasTarget()) {
            // Rotate to zero out the shooter-to-target error.
            double error = vision.getShooterErrorDeg();
            double pidOutput = txPID.calculate(error);
            double rotationSpeed = MathUtil.clamp(pidOutput,
                    -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);

            // Command shooter RPM and hood angle from distance-based LUT.
            shooter.commandRPM(vision.getTargetRPM());
            hoodedShooter.moveHoodToAngleWithOffset(vision.getTargetHoodAngleDeg());

            swerve.drive(driverInput, rotationSpeed, true, true);

            SmartDashboard.putNumber("AutoAlign/ShooterErrorDeg", error);
            SmartDashboard.putNumber("AutoAlign/DistanceM", vision.getDistanceM());
            SmartDashboard.putNumber("AutoAlign/RPM", vision.getTargetRPM());
            SmartDashboard.putNumber("AutoAlign/HoodAngleDeg", vision.getTargetHoodAngleDeg());
            SmartDashboard.putNumber("AutoAlign/RotationPID", pidOutput);
        } else {
            // No target — give driver full rotation control, keep shooter warm.
            shooter.setTargetRPM(4500);
            double rotation = MathUtil.applyDeadband(driverRotation.getAsDouble(), Constants.stickDeadband);
            rotation = Math.pow(rotation, 3) * Constants.Swerve.maxAngularVelocity;
            swerve.drive(driverInput, rotation, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs until trigger is released
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, true);
    }
}
