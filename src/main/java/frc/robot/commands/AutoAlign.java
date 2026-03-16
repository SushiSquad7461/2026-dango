package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;

public class AutoAlign extends Command {
    private final Swerve swerve;
    private final Vision vision;
    private final ShooterSubsystem shooter;
    private final HoodedShooter hoodedShooter;
    private final BooleanSupplier isRedSupplier;
    private final PIDController rotationPID;
    private boolean isRed;

    public AutoAlign(Swerve swerve, Vision vision, ShooterSubsystem shooter, HoodedShooter hoodedShooter, BooleanSupplier isRedSupplier) {
        this.swerve = swerve;
        this.vision = vision;
        this.shooter = shooter;
        this.hoodedShooter = hoodedShooter;
        this.isRedSupplier = isRedSupplier;
        rotationPID = Constants.Vision.rotationPID;
        rotationPID.setTolerance(2.0);
        rotationPID.enableContinuousInput(-180, 180);
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        isRed = isRedSupplier.getAsBoolean();
        rotationPID.reset();
    }

    @Override
    public void execute() {
        Rotation2d targetHeading = vision.getHeadingToHub(isRed);
        double distance = vision.getDistanceToHub(isRed);

        shooter.applyRPMFromDistance(distance);
        hoodedShooter.moveHoodToSetpoint(hoodedShooter.calculateDesiredAngle(distance, shooter.getTargetSpeedMS()));

        double rotation = rotationPID.calculate(
            swerve.getHeading().getDegrees(),
            targetHeading.getDegrees()
        );
        rotation = MathUtil.clamp(rotation, -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity);

        swerve.drive(new Translation2d(0, 0), rotation, true, true);

        SmartDashboard.putNumber("Vision/Distance", distance);
        SmartDashboard.putNumber("Vision/TargetHeading", targetHeading.getDegrees());
        SmartDashboard.putNumber("Vision/CurrentHeading", swerve.getHeading().getDegrees());
        SmartDashboard.putBoolean("Vision/HasPoseEstimate", vision.hasPoseEstimate());
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
