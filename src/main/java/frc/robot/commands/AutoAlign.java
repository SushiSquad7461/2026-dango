package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;

public class AutoAlign extends Command {
    private final Swerve swerve;
    private final Vision vision;
    private final boolean isRed;
    private final PIDController rotationPID;

    public AutoAlign(Swerve swerve, Vision vision, boolean isRed) {
        this.swerve = swerve;
        this.vision = vision;
        this.isRed = isRed;
        rotationPID = new PIDController(0.15, 0, 0);
        rotationPID.setTolerance(2.0);
        rotationPID.enableContinuousInput(-180, 180);
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        Rotation2d targetHeading = vision.getHeadingToScorePillar(isRed);
        double rotation = rotationPID.calculate(
            swerve.getHeading().getDegrees(),
            targetHeading.getDegrees()
        );
        swerve.drive(new Translation2d(0, 0), rotation, true, true);
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }
}