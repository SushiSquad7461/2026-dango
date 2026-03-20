package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants;

/**
 * Vision subsystem — uses raw Limelight tx/ty to aim at AprilTags on the hub.
 *
 * tx + camera yaw → shooter alignment error (degrees off from shooter axis).
 * ty + trig → distance to target → ShotLUT → RPM + hood angle.
 *
 * No global pose estimation, no MegaTag, no Kalman filter.
 */
public class Vision extends SubsystemBase {

    // AprilTag center height on the hub (44.25 inches).
    private static final double TAG_HEIGHT_M = 44.25 * 0.0254;  // 1.12395 m

    // Camera mounting — from Limelight UI (both cameras identical).
    private static final double CAMERA_HEIGHT_M  = 0.2439162;
    private static final double CAMERA_PITCH_DEG = 20.0;

    // Camera yaw offsets from robot forward (from Limelight UI).
    // Shooter faces 180° from robot forward.
    private static final double LEFT_CAM_YAW_DEG  =  150.0;
    private static final double RIGHT_CAM_YAW_DEG = -150.0;
    private static final double SHOOTER_YAW_DEG   =  180.0;

    // -------------------------------------------------------------------------

    private final ShotLUT lut;

    // Copilot RPM trim (flat offset applied during match).
    private double rpmOffset = 0;

    // Cached per-cycle results, read by AutoAlign.
    private boolean hasTarget = false;
    private double shooterErrorDeg = 0;  // positive = target is CW from shooter axis
    private double distanceM = 0;
    private double targetRPM = 0;
    private double targetHoodAngleDeg = 0;

    public Vision() {
        lut = ShotTable.buildLUT();
    }

    @Override
    public void periodic() {
        String bestCam = pickBestCamera();
        hasTarget = (bestCam != null);

        if (hasTarget) {
            double tx = LimelightHelpers.getTX(bestCam);
            double ty = LimelightHelpers.getTY(bestCam);
            double camYaw = bestCam.equals(Constants.Vision.primaryLimelightName)
                    ? LEFT_CAM_YAW_DEG : RIGHT_CAM_YAW_DEG;

            // Shooter alignment error: how far the shooter axis is off from the target.
            // cameraYaw + tx = target angle from robot forward.
            // Subtract shooter yaw (180°) to get error relative to shooter axis.
            shooterErrorDeg = MathUtil.inputModulus(camYaw + tx - SHOOTER_YAW_DEG, -180, 180);

            // Distance via trig: d = (tagH - camH) / tan(camPitch + ty)
            double angleDeg = CAMERA_PITCH_DEG + ty;
            double angleRad = Math.toRadians(angleDeg);
            if (angleRad > 0.01) {
                distanceM = (TAG_HEIGHT_M - CAMERA_HEIGHT_M) / Math.tan(angleRad);
            } else {
                distanceM = 0;
            }

            // Clamp distance to LUT range for sensible outputs.
            double clampedDist = MathUtil.clamp(distanceM, 1.5, 5.0);
            ShotLUT.ShotParameters params = lut.get(clampedDist);
            targetRPM = params.rpm() + rpmOffset;
            targetHoodAngleDeg = params.angle();
        } else {
            shooterErrorDeg = 0;
            distanceM = 0;
            targetRPM = 0;
            targetHoodAngleDeg = 0;
        }

        // Telemetry
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
        SmartDashboard.putNumber("Vision/ShooterErrorDeg", shooterErrorDeg);
        SmartDashboard.putNumber("Vision/DistanceM", distanceM);
        SmartDashboard.putNumber("Vision/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Vision/HoodAngleDeg", targetHoodAngleDeg);
        SmartDashboard.putNumber("Vision/RPMOffset", rpmOffset);
    }

    /**
     * Returns the camera name that currently has a valid target.
     * Prefers the camera whose target is closer to the shooter axis.
     * Returns null if neither sees a tag.
     */
    private String pickBestCamera() {
        boolean leftHas  = LimelightHelpers.getTV(Constants.Vision.primaryLimelightName);
        boolean rightHas = LimelightHelpers.getTV(Constants.Vision.secondaryLimelightName);
        if (leftHas && rightHas) {
            double leftError  = Math.abs(MathUtil.inputModulus(
                    LEFT_CAM_YAW_DEG + LimelightHelpers.getTX(Constants.Vision.primaryLimelightName)
                            - SHOOTER_YAW_DEG, -180, 180));
            double rightError = Math.abs(MathUtil.inputModulus(
                    RIGHT_CAM_YAW_DEG + LimelightHelpers.getTX(Constants.Vision.secondaryLimelightName)
                            - SHOOTER_YAW_DEG, -180, 180));
            return (leftError <= rightError) ? Constants.Vision.primaryLimelightName
                                             : Constants.Vision.secondaryLimelightName;
        }
        if (leftHas)  return Constants.Vision.primaryLimelightName;
        if (rightHas) return Constants.Vision.secondaryLimelightName;
        return null;
    }

    // --- Public getters for AutoAlign ---

    public boolean hasTarget()              { return hasTarget; }
    /** Degrees the shooter axis is off from the target. Positive = target is CW from shooter. */
    public double getShooterErrorDeg()      { return shooterErrorDeg; }
    public double getDistanceM()            { return distanceM; }
    public double getTargetRPM()            { return targetRPM; }
    public double getTargetHoodAngleDeg()   { return targetHoodAngleDeg; }

    // --- Copilot offset controls ---

    /** Bump the RPM offset by delta. Clamped to +/- 200. Bind to copilot D-pad. */
    public void adjustOffset(double delta) {
        rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
    }

    /** Reset the RPM offset to zero. */
    public void resetOffset() {
        rpmOffset = 0;
    }
}
