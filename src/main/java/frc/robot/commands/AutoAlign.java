package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
// import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;

public class AutoAlign extends Command {
    private final Swerve swerve;
    private final Vision vision;
    // private final ShooterSubsystem shooter;
    private final BooleanSupplier isRedSupplier;
    private final PIDController rotationPID;
    private final PIDController distancePID;
    private boolean isRed;

    public AutoAlign(Swerve swerve, Vision vision, /*ShooterSubsystem shooter,*/ BooleanSupplier isRedSupplier) {
        this.swerve = swerve;
        this.vision = vision;
        // this.shooter = shooter;
        this.isRedSupplier = isRedSupplier;
        rotationPID = Constants.Vision.rotationPID;
        rotationPID.setTolerance(2.0);
        rotationPID.enableContinuousInput(-180, 180);
        distancePID = Constants.Vision.distancePID;
        distancePID.setTolerance(0.1);
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        // Evaluate alliance now, when FMS is actually connected
        isRed = isRedSupplier.getAsBoolean();
        rotationPID.reset();
        distancePID.reset();
    }

    @Override
    public void execute() {
        if (!vision.hasHubTarget(isRed)) {
            swerve.drive(new Translation2d(0, 0), 0, true, true);
            return;
        }

        Rotation2d targetHeading = vision.getHeadingToScorePillar(isRed);
        double distance = vision.getDistanceToScorePillar(isRed);
        // shooter.setTargetRPM(distance); // TODO: enable variable RPM once tuned

        double rotation = rotationPID.calculate(
            swerve.getHeading().getDegrees(),
            targetHeading.getDegrees()
        );
        rotation = MathUtil.clamp(rotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);

        Translation2d translation = new Translation2d(0, 0);
        if (!Double.isNaN(distance)) {
            double translationSpeed = -distancePID.calculate(distance, Constants.Vision.targetDistanceMeters);
            translationSpeed = MathUtil.clamp(translationSpeed, -Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed);
            // targetHeading has +PI for rear launcher, so subtract PI to get the direction toward the tag
            Rotation2d directionToTag = targetHeading.minus(new Rotation2d(Math.PI));
            translation = new Translation2d(translationSpeed, directionToTag);
        }
        swerve.drive(translation, rotation, true, true);

        SmartDashboard.putNumber("Vision/Distance", Double.isNaN(distance) ? -1 : distance);
        SmartDashboard.putNumber("Vision/TargetHeading", targetHeading.getDegrees());
        SmartDashboard.putNumber("Vision/CurrentHeading", swerve.getHeading().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint() && distancePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // shooter.setTargetRPM(Constants.Shooter.TARGET_RPM_DEFAULT);
        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }
}
