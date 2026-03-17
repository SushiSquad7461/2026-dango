package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.ShotCalculator;
import frc.robot.subsystems.vision.Vision;

public class AutoAlign extends Command {
    // confidence() returns 0-100 (frc-fire-control: weighted geometric mean of 5 factors).
    // 50 is the midpoint; below this the solver, vision, or heading quality has degraded.
    private static final double CONFIDENCE_THRESHOLD = 50.0;

    private final Swerve swerve;
    private final Vision vision;
    private final ShooterSubsystem shooter;
    private final DoubleSupplier xTranslation;
    private final DoubleSupplier yTranslation;
    private final DoubleSupplier driverRotation;

    public AutoAlign(Swerve swerve, Vision vision, ShooterSubsystem shooter,
            DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier driverRotation) {
        this.swerve = swerve;
        this.vision = vision;
        this.shooter = shooter;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.driverRotation = driverRotation;
        // Only require swerve — setTargetRPM() is a setter, not a motor command.
        // Requiring shooter would conflict with StateMachine's SHOOT_ONLY command
        // when right trigger is held simultaneously with left trigger.
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Clear integral windup from any previous run.
        Constants.Vision.rotationPID.reset();
    }

    @Override
    public void execute() {
        Translation2d driverInput = new Translation2d(xTranslation.getAsDouble(), yTranslation.getAsDouble());
        ShotCalculator.LaunchParameters shot = vision.getCurrentShot();

        if (shot.isValid() && shot.confidence() > CONFIDENCE_THRESHOLD) {
            // Primes the flywheel to the SOTM RPM. Actual motor command happens when
            // StateMachine enters SHOOT state (right trigger held).
            shooter.setTargetRPM(shot.rpm());

            // driveAngle() points the robot front at the hub.
            // Rotate by π so the rear-facing shooter faces the hub instead.
            double currentHeading = swerve.getPose().getRotation().getDegrees();
            double targetHeading  = shot.driveAngle().rotateBy(Rotation2d.kPi).getDegrees();
            double pidOutput      = Constants.Vision.rotationPID.calculate(currentHeading, targetHeading);
            double rotationSpeed  = pidOutput + shot.driveAngularVelocityRadPerSec();

            swerve.drive(driverInput, rotationSpeed, true, true);
        } else {
            // No valid solution — keep shooter warm and give driver full rotation control.
            shooter.setTargetRPM(4500);
            swerve.drive(driverInput, driverRotation.getAsDouble(), true, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs until trigger is released (whileTrue in RobotContainer)
    }

    @Override
    public void end(boolean interrupted) {}
}
