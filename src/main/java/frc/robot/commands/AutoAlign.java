package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

public class AutoAlign extends Command {

    private final Swerve swerve;
    private final DoubleSupplier xTranslation;
    private final DoubleSupplier yTranslation;
    private final PIDController rotationPID;
    private final Pose2d hubPose;

    public AutoAlign(Swerve swerve, DoubleSupplier xTranslation, DoubleSupplier yTranslation) {
        this.swerve = swerve;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        addRequirements(swerve);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        int hubTagId = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 10 : 26;
        hubPose = fieldLayout.getTagPose(hubTagId).get().toPose2d();

        rotationPID = new PIDController(
                Constants.Vision.rotationPID.getP(),
                Constants.Vision.rotationPID.getI(),
                Constants.Vision.rotationPID.getD());
        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(4.0);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(xTranslation.getAsDouble(), Constants.stickDeadband); // driver inputs
        double y = MathUtil.applyDeadband(yTranslation.getAsDouble(), Constants.stickDeadband);
        Translation2d raw = new Translation2d(x, y);
        double magnitude = raw.getNorm();
        Translation2d driverInput = raw.times(Math.pow(magnitude, 2)).times(Constants.Swerve.maxSpeed); // same scaling as normal driving

        Pose2d currentPose = swerve.getPose(); // get robot pose
        double desiredHeadingDeg = hubPose.getTranslation().minus(currentPose.getTranslation()) // find heading difference
                .getAngle().getDegrees();
        double pidOutput = rotationPID.calculate(currentPose.getRotation().getDegrees(), desiredHeadingDeg);
        double rotationSpeed = MathUtil.clamp(pidOutput,
                -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity); // clamps heading change speed

        swerve.drive(driverInput, rotationSpeed, true, true);

        SmartDashboard.putNumber("AutoAlign/DesiredHeadingDeg", desiredHeadingDeg);
        SmartDashboard.putNumber("AutoAlign/HeadingErrorDeg", rotationPID.getError());
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, true);
    }
}
