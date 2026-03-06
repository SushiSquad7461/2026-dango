package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;

public class AutoAlign extends Command {
    private final Swerve swerve;
    private final Vision vision;
    private final BooleanSupplier isRedSupplier;
    private final PIDController rotationPID;
    private boolean isRed;

    public AutoAlign(Swerve swerve, Vision vision, BooleanSupplier isRedSupplier) {
        this.swerve = swerve;
        this.vision = vision;
        this.isRedSupplier = isRedSupplier;
        rotationPID = Constants.Vision.rotationPID;
        rotationPID.setTolerance(2.0);
        rotationPID.enableContinuousInput(-180, 180);
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        // Evaluate alliance now, when FMS is actually connected
        isRed = isRedSupplier.getAsBoolean();
        rotationPID.reset();
    }

    @Override
    public void execute() {
        if (!vision.hasHubTarget(isRed)) {
            swerve.drive(new Translation2d(0, 0), 0, true, true);
            return;
        }
        Rotation2d targetHeading = vision.getHeadingToScorePillar(isRed);
        double rotation = rotationPID.calculate(
            swerve.getHeading().getDegrees(),
            targetHeading.getDegrees()
        );
        rotation = MathUtil.clamp(rotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);
        swerve.drive(new Translation2d(0, 0), rotation, true, true);

        SmartDashboard.putNumber("Vision/Distance", vision.getDistanceToScorePillar(isRed));
        SmartDashboard.putNumber("Vision/TargetHeading", targetHeading.getDegrees());
        SmartDashboard.putNumber("Vision/CurrentHeading", swerve.getHeading().getDegrees());
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
