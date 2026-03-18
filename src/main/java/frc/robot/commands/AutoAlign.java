package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.HoodedShooter;
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
    private final HoodedShooter hoodedShooter;
    private final DoubleSupplier xTranslation;
    private final DoubleSupplier yTranslation;
    private final DoubleSupplier driverRotation;

    public AutoAlign(Swerve swerve, Vision vision, ShooterSubsystem shooter, HoodedShooter hoodedShooter,
            DoubleSupplier xTranslation, DoubleSupplier yTranslation, DoubleSupplier driverRotation) {
        this.swerve = swerve;
        this.vision = vision;
        this.shooter = shooter;
        this.hoodedShooter = hoodedShooter;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        this.driverRotation = driverRotation;
        // Only require swerve — shooter/hood methods are setters, not subsystem-exclusive.
        // Not requiring shooter avoids conflict with StateMachine's SHOOT_ONLY command
        // when right trigger is held simultaneously with left trigger.
        // Not requiring hoodedShooter allows d-pad trim (stepHood) to update
        // hoodSetpointDegrees while AutoAlign is running.
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Clear integral windup from any previous run.
        Constants.Vision.rotationPID.reset();
    }

    @Override
    public void execute() {
        // Apply the same deadband + cubic + maxSpeed scaling as TeleopSwerve.
        double x = MathUtil.applyDeadband(xTranslation.getAsDouble(), Constants.stickDeadband);
        double y = MathUtil.applyDeadband(yTranslation.getAsDouble(), Constants.stickDeadband);
        Translation2d raw = new Translation2d(x, y);
        double magnitude = raw.getNorm();
        Translation2d driverInput = raw.times(Math.pow(magnitude, 2)).times(Constants.Swerve.maxSpeed);

        ShotCalculator.LaunchParameters shot = vision.getCurrentShot();

        if (shot.isValid() && shot.confidence() > CONFIDENCE_THRESHOLD) {
            // RPM from the physics-sim LUT inside ShotCalculator.
            double rpm = shot.rpm();
            // Derive hood angle: RPM -> exit velocity -> kinematic launch angle.
            double exitVelocity = vision.getExitVelocity(rpm);
            double hoodAngle = hoodedShooter.calculateDesiredAngle(
                    shot.solvedDistanceM(), exitVelocity);

            // Command flywheel and hood every cycle so they track the changing solution.
            shooter.commandRPM(rpm);
            hoodedShooter.moveHoodToAngleWithOffset(hoodAngle);

            // Use gyro heading (same frame as swerve.drive's field-relative conversion)
            // so the PID error and the drive reference frame are consistent.
            double currentHeading = swerve.getHeading().getDegrees();
            double targetHeading  = shot.driveAngle().rotateBy(Rotation2d.kPi).getDegrees();
            double pidOutput      = Constants.Vision.rotationPID.calculate(currentHeading, targetHeading);
            double rotationSpeed  = pidOutput + shot.driveAngularVelocityRadPerSec();
            // Clamp so large heading errors don't steal translation bandwidth via desaturation.
            rotationSpeed = MathUtil.clamp(rotationSpeed, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);

            swerve.drive(driverInput, rotationSpeed, true, true);

            SmartDashboard.putNumber("SOTM/RPM", rpm);
            SmartDashboard.putNumber("SOTM/ExitVelocityMps", exitVelocity);
            SmartDashboard.putNumber("SOTM/HoodAngleDeg", hoodAngle);
            SmartDashboard.putNumber("SOTM/DistanceM", shot.solvedDistanceM());
            SmartDashboard.putNumber("SOTM/Confidence", shot.confidence());
            SmartDashboard.putNumber("SOTM/HeadingErrorDeg", currentHeading - targetHeading);
            SmartDashboard.putNumber("SOTM/RotationPID", pidOutput);
        } else {
            // No valid solution — keep shooter warm and give driver full rotation control.
            shooter.setTargetRPM(4500);
            double rotation = MathUtil.applyDeadband(driverRotation.getAsDouble(), Constants.stickDeadband);
            rotation = Math.pow(rotation, 3) * Constants.Swerve.maxAngularVelocity;
            swerve.drive(driverInput, rotation, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs until trigger is released (whileTrue in RobotContainer)
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, true);
    }
}
