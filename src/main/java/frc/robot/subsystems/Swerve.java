package frc.robot.subsystems;

import frc.robot.util.AllianceUtil;
import frc.robot.generated.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Volts;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] mSwerveMods;
    private final BaseStatusSignal[] modStatusSignals;
    private final Pigeon2 gyro;
    private final StatusSignal<Angle> gyroYaw;
    private final SysIdRoutine driveSysIdRoutine;
    private final SysIdRoutine steerSysIdRoutine;

    private final PIDController alignmentPID;

    private final NetworkTable table;

    private final DoublePublisher gyroDoublePublisher;
    private final Field2d field;
    private final DoublePublisher[] cancoderPubs;
    private final DoublePublisher[] anglePubs;
    private final DoublePublisher[] velocityPubs;

    // private final HttpCamera camStream;

    private double simCurrentDrawAmps = 0;
    private final DoubleEntry xPosEntry;
    private long xPosEntryLastChanged;
    private final DoubleEntry yPosEntry;
    private long yPosEntryLastChanged;
    private final DoubleEntry rotEntry;
    private long rotEntryLastChanged;

    public Swerve() {
        field = new Field2d();
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        gyroYaw = gyro.getYaw();
        alignmentPID = new PIDController(0.15, 0, 0);
        alignmentPID.setTolerance(10, 10);
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants), //Front Left Module
                new SwerveModule(1, Constants.Swerve.Mod1.constants), //Front Right Module
                new SwerveModule(2, Constants.Swerve.Mod2.constants), //Back Left Module
                new SwerveModule(3, Constants.Swerve.Mod3.constants) //Back Right Module
        };
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d());


        modStatusSignals = new BaseStatusSignal[] {
                mSwerveMods[0].getDrivePosition(),
                mSwerveMods[0].getDriveVelocity(),
                mSwerveMods[0].getAnglePosition(),
                mSwerveMods[0].getEncoderPosition(),
                mSwerveMods[1].getDrivePosition(),
                mSwerveMods[1].getDriveVelocity(),
                mSwerveMods[1].getAnglePosition(),
                mSwerveMods[1].getEncoderPosition(),
                mSwerveMods[2].getDrivePosition(),
                mSwerveMods[2].getDriveVelocity(),
                mSwerveMods[2].getAnglePosition(),
                mSwerveMods[2].getEncoderPosition(),
                mSwerveMods[3].getDrivePosition(),
                mSwerveMods[3].getDriveVelocity(),
                mSwerveMods[3].getAnglePosition(),
                mSwerveMods[3].getEncoderPosition(),
                gyroYaw
        };

        table = NetworkTableInstance.getDefault().getTable("Swerve");
        gyroDoublePublisher = table.getDoubleTopic("GyroYaw").publish();
        cancoderPubs = new DoublePublisher[4];
        anglePubs = new DoublePublisher[4];
        velocityPubs = new DoublePublisher[4];
        for (int i = 0; i < 4; i++) {
            cancoderPubs[i] = table.getDoubleTopic("Module " + i + "/CANcoder").publish();
            anglePubs[i] = table.getDoubleTopic("Module " + i + "/Angle").publish();
            velocityPubs[i] = table.getDoubleTopic("Module " + i + "/Velocity").publish();
        }
        if (Constants.IS_SIM) {
            xPosEntry = table.getDoubleTopic("Simulation/SetOdom/X").getEntry(0);
            xPosEntry.set(0);
            yPosEntry = table.getDoubleTopic("Simulation/SetOdom/Y").getEntry(0);
            yPosEntry.set(0);
            rotEntry = table.getDoubleTopic("Simulation/SetOdom/Rotation").getEntry(0);
            rotEntry.set(0);
        } else {
            xPosEntry = null;
            yPosEntry = null;
            rotEntry = null;
        }
        driveSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                        null, // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            // Apply the same voltage to all drive motors
                            for (SwerveModule mod : mSwerveMods) {
                                mod.setDriveVoltage(volts.in(Volts));
                            }
                        },
                        null,
                        this));

        steerSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                        null, // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            // Apply the same voltage to all steer motors
                            for (SwerveModule mod : mSwerveMods) {
                                mod.setSteerVoltage(volts.in(Volts));
                            }
                        },
                        null,
                        this));

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT RELATIVE ChassisSpeeds. Also
                                                                          // optionally outputs individual module
                                                                          // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(Constants.AutoConstants.kPTranslationController, 0, 0), // Translation PID
                                                                                                     // constants
                            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0.01) // Rotation PID
                                                                                                 // constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {

            e.printStackTrace();
        }
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getState().angle.getDegrees(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond,
                        null);

                builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getState().angle.getDegrees(),
                        null);
                builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond,
                        null);

                builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getState().angle.getDegrees(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond,
                        null);

                builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getState().angle.getDegrees(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> mSwerveMods[3].getState().speedMetersPerSecond,
                        null);

                builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getDegrees(), null);
            }
        });
    }

    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.dynamic(direction);
    }

    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.dynamic(direction);
    }

    public static enum AlignmentPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    public ChassisSpeeds getFieldVelocity() { // Added this new method
        // Uses your existing methods to get the robot speeds and the gyro heading
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    }

    private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

     private void stop() {
         for (SwerveModule mod : mSwerveMods) {
             mod.setDriveVoltage(0);
             mod.setSteerVoltage(0);

        }
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Command resetHeading() {
        return runOnce(() -> {
            setPose(
                    new Pose2d(
                            getPose().getTranslation(),
                            AllianceUtil.isRedAlliance() ? new Rotation2d(Math.PI) : new Rotation2d()));
        });
    }
    

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public Rotation2d getHeading() {
        return getGyroYaw();// getPose().getRotation();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            if (Math.abs(mod.getCANcoderWithOffset().getDegrees() - mod.getState().angle.getDegrees()) > 10)
                mod.resetToAbsolute();
        }
    }

    public Command resetPositionToFrontReef() {
        Waypoint bluePoint = new Waypoint(null, new Translation2d(3.171, 4.024), null);
        return runOnce(() -> {
            Pose2d targetPose = AllianceUtil.isRedAlliance()
                    ? new Pose2d(bluePoint.flip().anchor(), Rotation2d.fromDegrees(180))
                    : new Pose2d(bluePoint.anchor(), new Rotation2d(0.0));
            gyro.setYaw(targetPose.getRotation().getDegrees());
            setPose(targetPose);
        });
    }

    public void resetGyro() {
        if (AllianceUtil.isRedAlliance()) {
            gyro.setYaw(180);
            setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            gyro.setYaw(0);
            setPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */

    @Override
    public void periodic() {
        SmartDashboard.putNumber("GyroYaw", getGyroYaw().getDegrees());
        SmartDashboard.putNumber("PoseYaw", getPose().getRotation().getDegrees());
        BaseStatusSignal.refreshAll(modStatusSignals);
        for (SwerveModule mod : mSwerveMods) {
            cancoderPubs[mod.moduleNumber].set(mod.getCANcoder().getDegrees());
            var modState = mod.getState();
            anglePubs[mod.moduleNumber].set(modState.angle.getDegrees());
            velocityPubs[mod.moduleNumber].set(modState.speedMetersPerSecond);
        }

        updateOdom();

        Pose2d currentPose = getPose();
        currentPose = getPose();
        field.setRobotPose(currentPose);
        gyroDoublePublisher.set(getGyroYaw().getDegrees());
    }

    @Override
    public void simulationPeriodic() {
        simCurrentDrawAmps = 0;
        for (var mod : mSwerveMods) {
            simCurrentDrawAmps += mod.simulationPeriodic();
        }

        boolean resetRequested = false;
        var curPose = getPose();
        var x = curPose.getX();
        if (xPosEntry.getLastChange() != xPosEntryLastChanged) {
            resetRequested = true;
            xPosEntryLastChanged = xPosEntry.getLastChange();
            x = xPosEntry.get();
        }
        var y = curPose.getY();
        if (yPosEntry.getLastChange() != yPosEntryLastChanged) {
            resetRequested = true;
            yPosEntryLastChanged = yPosEntry.getLastChange();
            y = yPosEntry.get();
        }
        var rot = curPose.getRotation().getDegrees();
        if (rotEntry.getLastChange() != rotEntryLastChanged) {
            resetRequested = true;
            rotEntryLastChanged = rotEntry.getLastChange();
            rot = rotEntry.get();
        }

        if (resetRequested) {
            setPose(new Pose2d(x, y, Rotation2d.fromDegrees(rot)));
        } else {
            xPosEntry.set(x);
            xPosEntryLastChanged = xPosEntry.getLastChange();
            yPosEntry.set(y);
            yPosEntryLastChanged = yPosEntry.getLastChange();
            rotEntry.set(rot);
            rotEntryLastChanged = rotEntry.getLastChange();
        }
    }

    public double getSimulatedCurrentDrawAmps() {
        return simCurrentDrawAmps;
    }

    private void updateOdom() {
        gyroYaw.refresh();
        poseEstimator.update(getGyroYaw(), getModulePositions());
        
    }
}