package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.Constants;
import frc.robot.subsystems.vision.ProjectileSimulator;
import frc.robot.subsystems.vision.SOTMShotPhysicsSim;
import frc.robot.subsystems.vision.ShotCalculator;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public class ShooterIOSim implements ShooterIO {

    @AutoLog
    public static class ShooterData {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public final ShooterData data = new ShooterData();

    private static final double RPM_RESPONSE = 0.1;
    private static final double RPM_TOLERANCE = Constants.Shooter.SHOOTER_RPM_TOLERANCE;

    private final ProjectileSimulator projectileModel =
            new ProjectileSimulator(Constants.Vision.SOTM_PARAMETERS);

    private double simulatedRPM = 0;
    private double targetRPM = 0;
    private boolean feederRunning = false;
    private double hoodPos = 0;

    private Supplier<ShotCalculator.LaunchParameters> shotSupplier =
            () -> ShotCalculator.LaunchParameters.INVALID;
    private Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
    private Supplier<ChassisSpeeds> fieldVelocitySupplier = ChassisSpeeds::new;
    private Consumer<SOTMShotPhysicsSim.LaunchState> shotSpawnConsumer = launch -> {};

    public void configureShotSpawning(
            Supplier<ShotCalculator.LaunchParameters> shotSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<ChassisSpeeds> fieldVelocitySupplier,
            Consumer<SOTMShotPhysicsSim.LaunchState> shotSpawnConsumer) {
        if (shotSupplier != null) {
            this.shotSupplier = shotSupplier;
        }
        if (robotPoseSupplier != null) {
            this.robotPoseSupplier = robotPoseSupplier;
        }
        if (fieldVelocitySupplier != null) {
            this.fieldVelocitySupplier = fieldVelocitySupplier;
        }
        if (shotSpawnConsumer != null) {
            this.shotSpawnConsumer = shotSpawnConsumer;
        }
    }

    @Override
    public void runShooter(double rpm) {
        targetRPM = Math.max(0.0, rpm);
        data.appliedVolts = Math.min(12.0, targetRPM / 5000.0 * 12.0);
    }

    @Override
    public void stopShooter(double rpm) {
        targetRPM = Math.max(0.0, rpm / 2.0);
        data.appliedVolts = Math.min(12.0, targetRPM / 5000.0 * 12.0);
    }

    @Override
    public void runFeeder() {
        boolean wasFeederRunning = feederRunning;
        feederRunning = true;
        data.currentAmps = 5.0;

        if (!wasFeederRunning && isShooterNearTarget()) {
            spawnSimProjectile();
        }
    }

    @Override
    public void runFeederBack() {
        feederRunning = true;
        data.currentAmps = 4.0;
    }

    @Override
    public void stopFeeder() {
        feederRunning = false;
        data.currentAmps = 0.0;
    }

    @Override
    public double getFlywheelRPM() {
        double diff = targetRPM - simulatedRPM;
        simulatedRPM += diff * RPM_RESPONSE;
        return simulatedRPM;
    }

    @Override
    public double getFlywheelTargetRPM() {
        return targetRPM;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public void setHoodPos(double pos) {
        hoodPos = Math.max(0.0, Math.min(90.0, pos));
    }

    @Override
    public boolean isShooterReady() {
        return isShooterNearTarget();
    }

    private boolean isShooterNearTarget() {
        return Math.abs(getFlywheelRPM() - targetRPM) < RPM_TOLERANCE;
    }

    private void spawnSimProjectile() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds fieldVelocity = fieldVelocitySupplier.get();
        if (robotPose == null || fieldVelocity == null) {
            return;
        }

        ShotCalculator.LaunchParameters shot = shotSupplier.get();
        if (shot == null) {
            shot = ShotCalculator.LaunchParameters.INVALID;
        }

        double launchRpm = shot.isValid() ? shot.rpm() : targetRPM;
        double hoodAngleDeg = shot.isValid() ? shot.hoodAngleDeg() : Constants.Vision.TXTY_DEFAULT_HOOD_ANGLE_DEG;
        double driveHeadingRad = shot.isValid()
                ? shot.driveAngle().getRadians()
                : robotPose.getRotation().getRadians() + Math.PI;

        double launcherOffsetX = Constants.Vision.LAUNCHER_OFFSET_METERS.getX();
        double launcherOffsetY = Constants.Vision.LAUNCHER_OFFSET_METERS.getY();

        double robotHeading = robotPose.getRotation().getRadians();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double launcherFieldOffsetX = launcherOffsetX * cosH - launcherOffsetY * sinH;
        double launcherFieldOffsetY = launcherOffsetX * sinH + launcherOffsetY * cosH;

        Pose3d releasePose = new Pose3d(
                robotPose.getX() + launcherFieldOffsetX,
                robotPose.getY() + launcherFieldOffsetY,
                Constants.Vision.LAUNCHER_RELEASE_HEIGHT_METERS,
                new Rotation3d());

        double inheritedVx = fieldVelocity.vxMetersPerSecond + (-launcherFieldOffsetY) * fieldVelocity.omegaRadiansPerSecond;
        double inheritedVy = fieldVelocity.vyMetersPerSecond + launcherFieldOffsetX * fieldVelocity.omegaRadiansPerSecond;

        double muzzleSpeed = projectileModel.exitVelocity(launchRpm);
        double launchRad = Math.toRadians(hoodAngleDeg);
        double muzzleHorizontal = muzzleSpeed * Math.cos(launchRad);
        double muzzleVz = muzzleSpeed * Math.sin(launchRad);

        double muzzleVx = muzzleHorizontal * Math.cos(driveHeadingRad);
        double muzzleVy = muzzleHorizontal * Math.sin(driveHeadingRad);

        Translation3d launchVelocity = new Translation3d(
                inheritedVx + muzzleVx,
                inheritedVy + muzzleVy,
                muzzleVz);

        shotSpawnConsumer.accept(new SOTMShotPhysicsSim.LaunchState(releasePose, launchVelocity));
    }
}

