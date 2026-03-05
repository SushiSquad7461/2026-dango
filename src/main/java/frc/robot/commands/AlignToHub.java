package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;
import java.util.function.DoubleSupplier;

public class AlignToHub extends Command {
    private final Swerve swerve;
    private final Vision vision;
    private final DoubleSupplier strafeInput;
    private final PIDController distancePID;
    private final PIDController rotationPID;

    public AlignToHub(Swerve swerve, Vision vision, DoubleSupplier strafeInput) {
        this.swerve = swerve;
        this.vision = vision;
        this.strafeInput = strafeInput;

        rotationPID = new PIDController(0.15, 0, 0);
        rotationPID.setTolerance(2.0);
        rotationPID.enableContinuousInput(-180, 180);

        distancePID = new PIDController(
            Constants.Vision.DISTANCE_KP,
            Constants.Vision.DISTANCE_KI,
            Constants.Vision.DISTANCE_KD
        );
        distancePID.setTolerance(Constants.Vision.DISTANCE_TOLERANCE_METERS);

        addRequirements(swerve);
    }

    private Translation2d getHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return new Translation2d(Constants.Vision.RED_HUB_X, Constants.Vision.RED_HUB_Y);
        }
        return new Translation2d(Constants.Vision.BLUE_HUB_X, Constants.Vision.BLUE_HUB_Y);
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Translation2d hub = getHubPosition();

        double dx = hub.getX() - robotPose.getX();
        double dy = hub.getY() - robotPose.getY();
        double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double shooterAngleDeg = targetAngleDeg + 180.0;
        double currentAngleDeg = robotPose.getRotation().getDegrees();
        double rotation = -rotationPID.calculate(currentAngleDeg, shooterAngleDeg);

        double currentDistance = robotPose.getTranslation().getDistance(hub);
        double forwardSpeed = distancePID.calculate(
            currentDistance,
            Constants.Vision.HUB_SHOOTING_DISTANCE_METERS
        );

        double angle = Math.atan2(dy, dx);
        double forwardX = forwardSpeed * Math.cos(angle);
        double forwardY = forwardSpeed * Math.sin(angle);

        double perpAngle = angle + Math.PI / 2.0;
        double strafeVal = strafeInput.getAsDouble();
        double strafeX = strafeVal * Math.cos(perpAngle) * Constants.Swerve.maxSpeed;
        double strafeY = strafeVal * Math.sin(perpAngle) * Constants.Swerve.maxSpeed;

        swerve.drive(
            new Translation2d(forwardX + strafeX, forwardY + strafeY),
            rotation,
            true,
            false
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, false);
    }
}