package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.commands.TrajectoryAlign;
import frc.robot.commands.VisionAlign;

import frc.robot.util.AllianceUtil;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] mSwerveMods;
    private final BaseStatusSignal[] modStatusSignals;
    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final StatusSignal<Angle> gyroYaw;
    private final SysIdRoutine driveSysIdRoutine;
    private final SysIdRoutine steerSysIdRoutine;

    public final PhotonCamera leftCamera;
    public final PhotonCamera rightCamera;

    private final PhotonPoseEstimator photonPoseEstimatorLeft;
    private final PhotonPoseEstimator photonPoseEstimatorRight;
    private Matrix<N3, N1> curStdDevs;
    private final PIDController alignmentPID;

    private AlignmentPosition currentAlignmentPosition = AlignmentPosition.CENTER;
    
    private final NetworkTable table;
    private final StringPublisher alignmentPositionPub;

    private final DoublePublisher gyroDoublePublisher;
    private final Field2d field;
    public final AprilTagFieldLayout aprilTagFieldLayout;

    private final DoublePublisher[] cancoderPubs;
    private final DoublePublisher[] anglePubs;
    private final DoublePublisher[] velocityPubs;

    private final Alert robotConfigAlert;
    private final Alert leftCameraAlert;
    private final Alert rightCameraAlert;
    //private final HttpCamera camStream;

    private double simCurrentDrawAmps = 0;
    private final DoubleEntry xPosEntry;
    private long xPosEntryLastChanged;
    private final DoubleEntry yPosEntry;
    private long yPosEntryLastChanged;
    private final DoubleEntry rotEntry;
    private long rotEntryLastChanged;
    
    public Swerve() {
        field = new Field2d();
        gyro = new Pigeon2(frc.robot.subsystems.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        gyroSim = gyro.getSimState();
        gyroYaw = gyro.getYaw();
        alignmentPID = new PIDController(0.15, 0, 0); 
        alignmentPID.setTolerance(10, 10);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, frc.robot.subsystems.constants),
            new SwerveModule(1, frc.robot.subsystems.constants),
            new SwerveModule(2, frc.robot.subsystems.constants),
            new SwerveModule(3, frc.robot.subsystems.constants)
        };
        modStatusSignals = new BaseStatusSignal[]{
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

        //TODO: rename cameras
        leftCamera = new PhotonCamera(Constants.VisionConstants.leftCameraName);
        leftCameraAlert = new Alert(
            String.format("Left camera %s is not connected", Constants.VisionConstants.leftCameraName), 
            AlertType.kError);

        //camStream = new HttpCamera("Photonvison Left", "http://photonvision.local:1181");
        rightCamera = new PhotonCamera(Constants.VisionConstants.rightCameraName);
        rightCameraAlert = new Alert(
            String.format("Right camera %s is not connected", Constants.VisionConstants.rightCameraName), 
            AlertType.kError);
        
        poseEstimator = new SwerveDrivePoseEstimator(
            frc.robot.subsystems.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );
        
        if(Constants.IS_SIM) {
            Robot.registerFastPeriodic(() -> updateOdom());
        }
    
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        photonPoseEstimatorLeft = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.leftCamera 
        );
        photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonPoseEstimatorRight = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.VisionConstants.rightCamera
        );
        photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        table = NetworkTableInstance.getDefault().getTable("Swerve");
        alignmentPositionPub = table.getStringTopic("Alignment/Position").publish();
        if(Constants.IS_SIM) {
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
        gyroDoublePublisher = table.getDoubleTopic("GyroYaw").publish();
        cancoderPubs = new DoublePublisher[4];
        anglePubs = new DoublePublisher[4];
        velocityPubs = new DoublePublisher[4];
        for (int i = 0; i < 4; i++) {
            cancoderPubs[i] = table.getDoubleTopic("Module " + i + "/CANcoder").publish();
            anglePubs[i] = table.getDoubleTopic("Module " + i + "/Angle").publish();
            velocityPubs[i] = table.getDoubleTopic("Module " + i + "/Velocity").publish();
        }

        driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    // Apply the same voltage to all drive motors
                    for(SwerveModule mod : mSwerveMods){
                        mod.setDriveVoltage(volts.in(Volts));
                    }
                },
                null,
                this
            )
        );

        steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    // Apply the same voltage to all steer motors
                    for(SwerveModule mod : mSwerveMods){
                        mod.setSteerVoltage(volts.in(Volts));
                    }
                },
                null,
                this
            )
        );

        robotConfigAlert = new Alert(
            "Failed to get PathPlanner robot config from GUI settings, please ensure this file is present and restart the robot code", 
            AlertType.kError);
        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(Constants.AutoConstants.kPTranslationController, 0, 0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0.01) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {
            robotConfigAlert.set(true);
            e.printStackTrace();
        }
    
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Align Center ODOM", defer(() -> runTrajectoryAlign(AlignmentPosition.CENTER)));
        SmartDashboard.putData("Align Left ODOM", defer(() -> runTrajectoryAlign(AlignmentPosition.LEFT)));
        SmartDashboard.putData("Align Right ODOM", defer(() -> runTrajectoryAlign(AlignmentPosition.RIGHT)));

        SmartDashboard.putData("Align Center ROYAL", defer(() -> runRoyalAlign(AlignmentPosition.CENTER)));
        SmartDashboard.putData("Align Left ROYAL", defer(() -> runRoyalAlign(AlignmentPosition.LEFT)));
        SmartDashboard.putData("Align Right ROYAL", defer(() -> runRoyalAlign(AlignmentPosition.RIGHT)));

        SmartDashboard.putData("Align Center VISION", defer(() -> runVisionAlign(AlignmentPosition.CENTER)));
        SmartDashboard.putData("Align Left VISION", defer(() -> runVisionAlign(AlignmentPosition.LEFT)));
        SmartDashboard.putData("Align Right VISION", defer(() -> runVisionAlign(AlignmentPosition.RIGHT)));


        SmartDashboard.putData("Reset Position", defer(() -> resetPositionToFrontReef()));
        SmartDashboard.putData("Stop Drive", runOnce(() -> stop()));

        SmartDashboard.putData("DriveSysIdQuasiFwd", sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("DriveSysIdQuasiRev", sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("DriveSysIdDynFwd", sysIdDriveDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("DriveSysIdDynRev", sysIdDriveDynamic(SysIdRoutine.Direction.kReverse));
        
        SmartDashboard.putData("SteerSysIdQuasiFwd", sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SteerSysIdQuasiRev", sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SteerSysIdDynFwd", sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SteerSysIdDynRev", sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("Swerve Drive", new Sendable() {    
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> mSwerveMods[0].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> mSwerveMods[0].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> mSwerveMods[1].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> mSwerveMods[1].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> mSwerveMods[2].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> mSwerveMods[2].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> mSwerveMods[3].getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () ->mSwerveMods[3].getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
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

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return frc.robot.subsystems.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        SwerveModuleState[] states = frc.robot.subsystems.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, frc.robot.subsystems.maxSpeed);
        setModuleStates(states);
    }

    private void stop() {
        for (SwerveModule mod : mSwerveMods) {
            mod.setDriveVoltage(0);
        }
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            frc.robot.subsystems.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, frc.robot.subsystems.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, frc.robot.subsystems.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Command resetHeading() {
        return runOnce(() -> {
            setPose(
                new Pose2d(
                    getPose().getTranslation(),
                    AllianceUtil.isRedAlliance() ? new Rotation2d(Math.PI) : new Rotation2d()
                )
            );
        });
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
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

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            if (Math.abs(mod.getCANcoderWithOffset().getDegrees() - mod.getState().angle.getDegrees()) > 10)
                mod.resetToAbsolute();
        }
    }

    public Command resetPositionToFrontReef() {
        Waypoint bluePoint = new Waypoint(null, new Translation2d(3.171, 4.024), null);
        return Commands.sequence(
            runOnce(() -> {
                setPose(AllianceUtil.isRedAlliance() ? new Pose2d(bluePoint.flip().anchor(), new Rotation2d(180.0)) : new Pose2d(bluePoint.anchor(), new Rotation2d(0.0)));
                resetGyro();
            })
        );   

    }

    public void resetGyro() {
        if (AllianceUtil.isRedAlliance()) gyro.setYaw(180);
        else gyro.setYaw(0);
    }

    public Command runTrajectoryAlign(AlignmentPosition position) {
        return new TrajectoryAlign(this, field, position);
    }
    
    public Command runRoyalAlign(AlignmentPosition position) {
        return new TrajectoryAlign(this, field, position);
    }

    public Command runVisionAlign(AlignmentPosition position) {
        return new VisionAlign(this, field, position, aprilTagFieldLayout);
    }
    
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            if (change.hasTargets()) {
                if (change.multitagResult.isPresent() || change.getBestTarget() != null && change.getBestTarget().poseAmbiguity < 0.15) {
                    visionEst = photonEstimator.update(change);
                    updateEstimationStdDevs(photonEstimator, visionEst, change.getTargets());
                }
            }
        }
        return visionEst;
    }

    private void updateEstimationStdDevs(PhotonPoseEstimator photonEstimator, Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.VisionConstants.SINGLE_TAG_STD_DEVS;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = Constants.VisionConstants.SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.MULTI_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 3)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }


    
    private boolean poseIsValid(EstimatedRobotPose pose) {
        return pose.estimatedPose.getZ() < 0.75 &&
            pose.estimatedPose.getX() > 0.0 &&
            pose.estimatedPose.getX() < aprilTagFieldLayout.getFieldLength() &&
            pose.estimatedPose.getY() > 0.0 &&
            pose.estimatedPose.getY() < aprilTagFieldLayout.getFieldWidth();
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(modStatusSignals);
        for(SwerveModule mod : mSwerveMods){
            cancoderPubs[mod.moduleNumber].set(mod.getCANcoder().getDegrees());
            var modState = mod.getState();
            anglePubs[mod.moduleNumber].set(modState.angle.getDegrees());
            velocityPubs[mod.moduleNumber].set(modState.speedMetersPerSecond);
        }

        updateOdom(); 


        if (!Constants.isAuto) {
            var leftGotPose = false;
            if (leftCamera.isConnected()) {
                var estOpt = getEstimatedGlobalPose(leftCamera, photonPoseEstimatorLeft);
                if (estOpt.isPresent()) {
                    var est = estOpt.get();
                    poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
                    leftGotPose = true;
                }
                leftCameraAlert.set(false);
            } else {
                leftCameraAlert.set(true);
            }
            if (rightCamera.isConnected()) {
                if (!leftGotPose) {
                    var estOpt = getEstimatedGlobalPose(rightCamera, photonPoseEstimatorRight);
                    if(estOpt.isPresent()) {
                        var est = estOpt.get();
                        poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
                    }
                }
                rightCameraAlert.set(false);
            } else {
                rightCameraAlert.set(true);
            }   
        }
        
        Pose2d currentPose = getPose();
        currentPose = getPose();
        field.setRobotPose(currentPose);
        gyroDoublePublisher.set(getGyroYaw().getDegrees());
        alignmentPositionPub.set(currentAlignmentPosition.toString());
    }

    @Override
    public void simulationPeriodic() {
        simCurrentDrawAmps = 0;
        for(var mod : mSwerveMods) {
            simCurrentDrawAmps += mod.simulationPeriodic();
        }

        boolean resetRequested = false;
        var curPose = getPose();
        var x = curPose.getX();
        if(xPosEntry.getLastChange() != xPosEntryLastChanged) {
            resetRequested = true;
            xPosEntryLastChanged = xPosEntry.getLastChange();
            x = xPosEntry.get();
        }
        var y = curPose.getY();
        if(yPosEntry.getLastChange() != yPosEntryLastChanged) {
            resetRequested = true;
            yPosEntryLastChanged = yPosEntry.getLastChange();
            y = yPosEntry.get();
        }
        var rot = curPose.getRotation().getDegrees();
        if(rotEntry.getLastChange() != rotEntryLastChanged) {
            resetRequested = true;
            rotEntryLastChanged = rotEntry.getLastChange();
            rot = rotEntry.get();
        }
        
        if(resetRequested) {
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

    priavate void updateOdom() {
        if (Constants.IS_SIM) {
            gyroSim.setRawYaw(Units.radiansToDegrees(
                getGyroYaw().getRadians() + getRobotRelativeSpeeds().omegaRadiansPerSecond * Constants.LOOP_TIME_SECONDS));
        }
        poseEstimator.update(getGyroYaw(), getModulePositions());
    }
}