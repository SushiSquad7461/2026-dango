package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Turns the robot to face the hub while the driver keeps translation control, and sets flywheel
 * RPM and hood angle from the LUT based on distance to the hub. Feeding is SHOOT_ONLY's job.
 * Only requires swerve, so it can run alongside SHOOT_ONLY (which requires the shooter).
 * Runs until interrupted.
 */
public class AutoAlign extends Command {

    // Hub centers in blue-origin field coordinates (meters), precomputed from 2026-rebuilt-andymark.json
    // as the midpoint of each hub's tag bounding box (blue tags 18-21, 24-27; red tags 2-5, 8-11).
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6115097, 4.0213534);
    private static final Translation2d RED_HUB_CENTER = new Translation2d(11.9015002, 4.0213534);

    private final Swerve swerve;
    private final DoubleSupplier xTranslation;
    private final DoubleSupplier yTranslation;
    private final PIDController rotationPID;
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2 * Constants.Swerve.maxAngularVelocity);

    public AutoAlign(Swerve swerve, DoubleSupplier xTranslation, DoubleSupplier yTranslation) {
        this.swerve = swerve;
        this.xTranslation = xTranslation;
        this.yTranslation = yTranslation;
        addRequirements(swerve);

        rotationPID = new PIDController(
                Constants.Vision.rotationPID.getP(),
                Constants.Vision.rotationPID.getI(),
                Constants.Vision.rotationPID.getD());
        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(4.0);
    }

    // Looked up every loop, not at construction: the alliance is often unknown when robot code boots.
    private static Translation2d getHubCenter() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rotationLimiter.reset(0);
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(xTranslation.getAsDouble(), Constants.stickDeadband); // driver inputs
        double y = MathUtil.applyDeadband(yTranslation.getAsDouble(), Constants.stickDeadband);
        Translation2d raw = new Translation2d(x, y);
        double magnitude = raw.getNorm();
        Translation2d driverInput = raw.times(Math.pow(magnitude, 2)).times(Constants.Swerve.maxSpeed); // same scaling as normal driving

        Pose2d currentPose = swerve.getPose(); // get robot pose
        Translation2d toHub = getHubCenter().minus(currentPose.getTranslation());
        double desiredHeadingDeg = toHub.getAngle().getDegrees(); // find heading difference
        double pidOutput = rotationPID.calculate(currentPose.getRotation().getDegrees()+180, desiredHeadingDeg);
        double rotationSpeed = MathUtil.clamp(pidOutput,
                -Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity); // clamps heading change speed
        rotationSpeed = rotationLimiter.calculate(rotationSpeed); // prevents instantaneous +/-max flips when error noise crosses the +-180 wrap boundary

        swerve.drive(driverInput, rotationSpeed, true, true);

        SmartDashboard.putNumber("AutoAlign/DesiredHeadingDeg", desiredHeadingDeg);
        SmartDashboard.putNumber("AutoAlign/HeadingErrorDeg", rotationPID.getError());
    }

    @Override
    public boolean isFinished() {
        return false; // only ends when the button is released (see whileTrue binding)
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, true);
    }
}
