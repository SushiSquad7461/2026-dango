package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.generated.Constants;
import java.util.ArrayList;

public class SOTMShotPhysicsSim {
    public static class Output {
        public double flywheelRPM = 0.0;
        public boolean shooterReady = false;
        public boolean shotActive = false;
        public double confidence = 0.0;
        public double tofSec = 0.0;
        public double dragCompensatedTofSec = 0.0;
        public Pose3d notePose = new Pose3d();
        public Pose3d[] trajectory = new Pose3d[] {};
    }

    private static final double GRAVITY_MPS2 = 9.81;
    private static final double DRAG_COEFF = 0.47;
    private static final double INTEGRATION_STEP_SEC = 0.005;
    private static final double MAX_FLIGHT_TIME_SEC = 4.0;
    private static final double EPSILON = 1e-6;

    private final Output output = new Output();
    private final ArrayList<Pose3d> trajectory = new ArrayList<>();
    private boolean shotInFlight = false;
    private double shotTimeSec = 0.0;
    private double shotX = 0.0;
    private double shotY = 0.0;
    private double shotZ = 0.0;
    private double shotVx = 0.0;
    private double shotVy = 0.0;
    private double shotVz = 0.0;
    private double shotInitialSpeedMps = 0.0;
    private double shotTargetDistanceMeters = 0.0;
    private double shotTargetYMeters = 0.0;
    private double shotTargetHeightMeters = 0.0;
    private double shotNoDragTofSec = 0.0;
    private double shotCooldownSec = 0.0;

    public Output update(
            double targetRPM,
            int feederDirection,
            double rpmTolerance,
            double dtSec,
            double hoodAngleDeg,
            double targetDistanceMeters,
            double robotVxMetersPerSecond,
            double robotVyMetersPerSecond) {
        double normalizedDt = dtSec / 0.02;
        if (normalizedDt < 0.0) normalizedDt = 0.0;
        if (normalizedDt > 1.0) normalizedDt = 1.0;
        double responseGain = 0.1 * normalizedDt;
        output.flywheelRPM += (targetRPM - output.flywheelRPM) * responseGain;

        double rpmError = Math.abs(output.flywheelRPM - targetRPM);
        output.shooterReady = rpmError < rpmTolerance;

        if (dtSec < 0.0) dtSec = 0.0;
        if (dtSec > 0.1) dtSec = 0.1;
        shotCooldownSec += dtSec;

        if (!shotInFlight
                && feederDirection < 0
                && output.shooterReady
                && shotCooldownSec >= minShotIntervalSec()) {
            startShot(hoodAngleDeg, targetDistanceMeters, robotVxMetersPerSecond, robotVyMetersPerSecond);
        }

        boolean shotResolvedThisCycle = false;
        boolean shotReachedTargetThisCycle = false;
        if (shotInFlight && dtSec > EPSILON) {
            shotReachedTargetThisCycle = simulateShot(dtSec);
            shotResolvedThisCycle = !shotInFlight;
        }

        output.shotActive = shotInFlight;
        if (shotInFlight || shotResolvedThisCycle) {
            output.tofSec = shotTimeSec;
            output.dragCompensatedTofSec = shotNoDragTofSec;
            output.notePose = new Pose3d(shotX, shotY, shotZ, Rotation3d.kZero);
            output.trajectory = trajectory.toArray(new Pose3d[0]);

            if (shotReachedTargetThisCycle) {
                double speedAtTarget = hypot3(shotVx, shotVy, shotVz);
                double speedRetention =
                        shotInitialSpeedMps > EPSILON ? speedAtTarget / shotInitialSpeedMps : 0.0;
                double heightError = Math.abs(shotZ - shotTargetHeightMeters);
                double heightTolerance = Math.max(shotTargetHeightMeters, 0.1);
                double heightFactor = clamp(1.0 - (heightError / heightTolerance), 0.0, 1.0);
                double lateralError = Math.abs(shotY - shotTargetYMeters);
                double lateralTolerance = Math.max(shotTargetDistanceMeters * 0.1, 0.1);
                double lateralFactor = clamp(1.0 - (lateralError / lateralTolerance), 0.0, 1.0);
                output.confidence = clamp(100.0 * speedRetention * heightFactor * lateralFactor, 0.0, 100.0);
            } else if (shotResolvedThisCycle) {
                output.confidence = 0.0;
            } else {
                double speed = hypot3(shotVx, shotVy, shotVz);
                double speedRetention = shotInitialSpeedMps > EPSILON ? speed / shotInitialSpeedMps : 0.0;
                output.confidence = clamp(100.0 * speedRetention, 0.0, 100.0);
            }
        } else {
            output.tofSec = 0.0;
            output.dragCompensatedTofSec = 0.0;
            output.notePose = new Pose3d();
            output.trajectory = new Pose3d[] {};
            output.confidence = clamp(100.0 * (1.0 - (rpmError / (rpmTolerance * 2.0))), 0.0, 100.0);
        }

        return output;
    }

    private void startShot(
            double hoodAngleDeg,
            double targetDistanceMeters,
            double robotVxMetersPerSecond,
            double robotVyMetersPerSecond) {
        double launchAngleDeg = hoodAngleDeg;
        if (!Double.isFinite(launchAngleDeg)) launchAngleDeg = 0.0;
        if (launchAngleDeg < 0.0) launchAngleDeg = 0.0;
        if (launchAngleDeg > 90.0) launchAngleDeg = 90.0;
        double launchHeightMeters = Constants.Vision.cameraPosePrimary.getZ();
        double targetHeightMeters = Constants.Vision.cameraPosePrimary.getZ();
        double launchSpeedMps =
                calculateLaunchSpeedMps(
                        output.flywheelRPM,
                        launchAngleDeg,
                        targetDistanceMeters,
                        launchHeightMeters,
                        targetHeightMeters);
        double launchAngleRad = Math.toRadians(launchAngleDeg);

        shotInFlight = launchSpeedMps > EPSILON;
        shotTimeSec = 0.0;
        shotX = 0.0;
        shotY = 0.0;
        shotZ = launchHeightMeters;
        shotVx = launchSpeedMps * Math.cos(launchAngleRad) + robotVxMetersPerSecond;
        shotVy = robotVyMetersPerSecond;
        shotVz = launchSpeedMps * Math.sin(launchAngleRad);
        shotInitialSpeedMps = hypot3(shotVx, shotVy, shotVz);
        shotTargetDistanceMeters = Math.max(targetDistanceMeters, 0.0);
        shotTargetYMeters = 0.0;
        shotTargetHeightMeters = targetHeightMeters;
        shotNoDragTofSec = shotVx > EPSILON ? shotTargetDistanceMeters / shotVx : 0.0;
        shotCooldownSec = 0.0;
        trajectory.clear();
        trajectory.add(new Pose3d(shotX, shotY, shotZ, Rotation3d.kZero));
    }

    private boolean simulateShot(double dtSec) {
        double remainingSec = dtSec;
        while (remainingSec > EPSILON && shotInFlight) {
            double stepSec = Math.min(remainingSec, INTEGRATION_STEP_SEC);
            remainingSec -= stepSec;

            double previousX = shotX;
            double previousY = shotY;
            double previousZ = shotZ;
            double previousVx = shotVx;
            double previousVy = shotVy;
            double previousVz = shotVz;
            double previousTimeSec = shotTimeSec;

            double speed = hypot3(shotVx, shotVy, shotVz);
            double dragAx = -DRAG_COEFF * speed * shotVx;
            double dragAy = -DRAG_COEFF * speed * shotVy;
            double dragAz = -GRAVITY_MPS2 - (DRAG_COEFF * speed * shotVz);

            shotVx += dragAx * stepSec;
            shotVy += dragAy * stepSec;
            shotVz += dragAz * stepSec;
            shotX += shotVx * stepSec;
            shotY += shotVy * stepSec;
            shotZ += shotVz * stepSec;
            shotTimeSec += stepSec;
            trajectory.add(new Pose3d(shotX, shotY, shotZ, Rotation3d.kZero));

            if (previousX < shotTargetDistanceMeters && shotX >= shotTargetDistanceMeters) {
                double deltaX = shotX - previousX;
                double alpha = Math.abs(deltaX) > EPSILON
                        ? (shotTargetDistanceMeters - previousX) / deltaX
                        : 0.0;
                alpha = clamp(alpha, 0.0, 1.0);
                shotTimeSec = previousTimeSec + (stepSec * alpha);
                shotX = shotTargetDistanceMeters;
                shotY = previousY + ((shotY - previousY) * alpha);
                shotZ = previousZ + ((shotZ - previousZ) * alpha);
                shotVx = previousVx + ((shotVx - previousVx) * alpha);
                shotVy = previousVy + ((shotVy - previousVy) * alpha);
                shotVz = previousVz + ((shotVz - previousVz) * alpha);
                shotInFlight = false;
                trajectory.add(new Pose3d(shotX, shotY, shotZ, Rotation3d.kZero));
                return true;
            }

            if (shotZ <= 0.0 || shotTimeSec >= MAX_FLIGHT_TIME_SEC) {
                shotInFlight = false;
                return false;
            }
        }
        return false;
    }

    private static double calculateLaunchSpeedMps(
            double flywheelRpm,
            double hoodAngleDeg,
            double targetDistanceMeters,
            double launchHeightMeters,
            double targetHeightMeters) {
        double distanceMeters = Math.max(targetDistanceMeters, 0.0);
        double launchAngleRad = Math.toRadians(clamp(hoodAngleDeg, 0.0, 89.0));
        double cos = Math.cos(launchAngleRad);
        double tan = Math.tan(launchAngleRad);
        double deltaHeightMeters = targetHeightMeters - launchHeightMeters;
        double denominator = 2.0 * cos * cos * ((distanceMeters * tan) - deltaHeightMeters);
        if (distanceMeters <= EPSILON || denominator <= EPSILON) {
            return 0.0;
        }
        double speedAtDefaultTargetRpmMps =
                Math.sqrt((GRAVITY_MPS2 * distanceMeters * distanceMeters) / denominator);
        if (!Double.isFinite(speedAtDefaultTargetRpmMps)) {
            return 0.0;
        }
        double defaultTargetRpm = Math.abs(Constants.Shooter.TARGET_RPM_DEFAULT);
        if (defaultTargetRpm <= EPSILON) {
            return 0.0;
        }
        double rpmScale = Math.abs(flywheelRpm) / defaultTargetRpm;
        return speedAtDefaultTargetRpmMps * rpmScale;
    }

    private static double minShotIntervalSec() {
        double feederRpm = Math.abs(Constants.Shooter.FEEDER_RPM);
        if (feederRpm < EPSILON) return 0.02;
        return 60.0 / feederRpm;
    }

    private static double hypot3(double a, double b, double c) {
        return Math.sqrt((a * a) + (b * b) + (c * c));
    }

    private static double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
