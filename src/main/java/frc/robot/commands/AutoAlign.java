package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.AllianceUtil;

/**
 * Turns the robot to face the hub while the driver keeps translation control. Flywheel, hood and
 * feeding are AutoShot's job (run by SHOOT_ONLY).
 * Only requires swerve, so it can run alongside SHOOT_ONLY (which requires the shooter).
 * Runs until interrupted.
 */
public class AutoAlign extends Command {

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
        Translation2d toHub = AllianceUtil.getHubCenter().minus(currentPose.getTranslation());
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
        return false; // cancelled on trigger release: the IDLE onFalse command shares the hopper requirement
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, true, true);
    }
}
