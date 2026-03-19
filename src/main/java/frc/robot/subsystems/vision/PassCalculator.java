package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.Constants;

/**
 * Computes pass target point and parameters for lobbing balls from the central zone
 * into the alliance zone, routing around the hub.
 */
public class PassCalculator {

    public record PassParameters(
        Translation2d targetPoint,
        double rpm,
        double hoodAngleDeg,
        double tofSec,
        double aimAngleRad,
        double distance,
        boolean isValid,
        boolean isBlocked
    ) {
        public static final PassParameters INVALID =
            new PassParameters(new Translation2d(), 0, 0, 0, 0, 0, false, false);
    }

    private final ShotLUT passLUT;

    public PassCalculator(ShotLUT passLUT) {
        this.passLUT = passLUT;
    }

    public PassParameters calculate(Pose2d robotPose, Alliance alliance) {
        double rx = robotPose.getX();
        double ry = robotPose.getY();

        if (!isInCentralZone(rx)) {
            return PassParameters.INVALID;
        }

        double tx = (alliance == Alliance.Blue)
            ? Constants.Passing.PASS_TARGET_X_BLUE
            : Constants.Passing.PASS_TARGET_X_RED;

        double targetY = ry;
        boolean blocked = false;

        double hubBlockMin = Constants.Passing.HUB_CENTER_Y
            - Constants.Passing.HUB_HALF_WIDTH
            - Constants.Passing.HUB_AVOIDANCE_MARGIN;
        double hubBlockMax = Constants.Passing.HUB_CENTER_Y
            + Constants.Passing.HUB_HALF_WIDTH
            + Constants.Passing.HUB_AVOIDANCE_MARGIN;

        if (targetY > hubBlockMin && targetY < hubBlockMax) {
            blocked = true;
            if (ry < Constants.Passing.HUB_CENTER_Y) {
                targetY = hubBlockMin;
            } else {
                targetY = hubBlockMax;
            }
        }

        targetY = MathUtil.clamp(targetY,
            Constants.Passing.PASS_TARGET_Y_MIN,
            Constants.Passing.PASS_TARGET_Y_MAX);

        Translation2d target = new Translation2d(tx, targetY);
        double distance = robotPose.getTranslation().getDistance(target);
        double aimAngleRad = Math.atan2(
            target.getY() - ry,
            target.getX() - rx
        );

        // Clamp distance to PassTable range to prevent extrapolation
        double clampedDistance = MathUtil.clamp(distance, 2.5, 10.0);
        ShotLUT.ShotParameters shotParams = passLUT.get(clampedDistance);

        return new PassParameters(
            target,
            shotParams.rpm(),
            shotParams.angle(),
            shotParams.tof(),
            aimAngleRad,
            distance,
            true,
            blocked
        );
    }

    public static boolean isInCentralZone(double robotX) {
        return robotX >= Constants.Passing.CENTRAL_ZONE_MIN_X
            && robotX <= Constants.Passing.CENTRAL_ZONE_MAX_X;
    }
}
