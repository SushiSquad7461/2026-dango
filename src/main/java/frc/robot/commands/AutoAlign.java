package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;

public class AutoAlign extends Command {
    private final Swerve swerve;
    private final Vision vision;
    private final boolean isRed;

    public AutoAlign(Swerve drive, Vision vision, boolean isRed) {
        this.swerve = drive;
        this.vision = vision;
        this.isRed = isRed;
        addRequirements(drive);
    }

    @Override
    public void execute() {
 
        Rotation2d rotationError = vision.getHeadingToScorePillar(isRed).minus(swerve.getHeading());
        double rotation = rotationError.getRadians() * Constants.Vision.KP;
        swerve.drive(new Translation2d(0, 0), rotation, true, true);
    }

    @Override
    public boolean isFinished() {
        Rotation2d targetHeading = vision.getHeadingToScorePillar(isRed);
        Rotation2d currentHeading = swerve.getHeading();
        double errorDegrees = Math.abs(targetHeading.minus(currentHeading).getDegrees());
        //TODO: Tune error degrees if needed
        return errorDegrees < Constants.Vision.ERROR_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motors
        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }
}