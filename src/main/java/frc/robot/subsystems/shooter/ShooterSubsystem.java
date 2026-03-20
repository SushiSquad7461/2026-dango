// Shooter Subsystem: controls shooter state using a state machine, switches between IDLE, PRESHOOT, and SHOOT
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
  // private double shootStartTime = 0; // could come in useful later, especially for logging
  private final PIDController shooterPidController = new PIDController(Constants.Shooter.SHOOTER_KP, Constants.Shooter.SHOOTER_KI, Constants.Shooter.SHOOTER_KD);
  private double targetRPM = Constants.Shooter.TARGET_RPM_DEFAULT;

  public enum ShooterState {
    IDLE, // shooter inactive
    PRESHOOT, // shooter spinning up, waiting for hood to come into position, or waiting for robot to turn to goal
    SHOOT // shooting
  }

  private ShooterState state = ShooterState.IDLE;
  private final ShooterIO io;
  private Supplier<ChassisSpeeds> robotSpeedsSupplier = ChassisSpeeds::new;
  //ShooterDataAutoLogged data = new ShooterDataAutoLogged();

  public LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 5);


  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void startShoot() {
    state = ShooterState.PRESHOOT;
  }

  public void stop() {
    state = ShooterState.IDLE;
  }

  public Command changeState(ShooterState newState){
    this.state = newState;
    switch (newState) {
      case IDLE:
        return Commands.parallel(
            Commands.runOnce(()->{
                io.stopShooter(targetRPM);    
            }),
            Commands.runOnce(()->{
                io.stopFeeder();
            }));

      case PRESHOOT:
        return Commands.parallel(
          Commands.runOnce(()->io.runShooter(targetRPM)),
           Commands.waitUntil(() -> isShooterReady()).andThen(Commands.runOnce(() -> {
               this.state = ShooterState.SHOOT;
               io.runShooter(targetRPM);
               io.runFeeder();
           })));
      case SHOOT:
        return Commands.parallel(
            Commands.runOnce(()->{
                io.runShooter(targetRPM);
            }),
            Commands.runOnce(()->{
                io.runFeeder();
            }));
      default:
        return Commands.none();
    }

  }

  public Command runFeeder() {
   return Commands.runOnce(()->io.runFeeder());
  }
  public Command runFeederBack(){
   return Commands.runOnce(()->io.runFeederBack());
  }
  public Command stopFeeder() {
   return Commands.runOnce(()->io.stopFeeder());
  }
  // public void setTargetRPM(double distance) {
  //   double rpm = distance * Constants.Shooter.RPM_DISTANCE_MULTIPLIER + Constants.Shooter.RPM_DISTANCE_OFFSET;
  //   this.targetRPM = rpm;
  // }
  public void setTargetRPM(String location) {
    switch (location) {
      case "hub":
        this.targetRPM = Constants.Shooter.TARGET_RPM_HUB;
        break;
      case "outpost":
        this.targetRPM = Constants.Shooter.TARGET_RPM_OUTPOST;
        break;
      case "trench":
        this.targetRPM = Constants.Shooter.TARGET_RPM_TRENCH;
        break;
      default:
        this.targetRPM = Constants.Shooter.TARGET_RPM_DEFAULT;
        break;
    }
  }
  public boolean isShooterReady() {
    return io.isShooterReady();
  }

  public void setRobotSpeedsSupplier(Supplier<ChassisSpeeds> supplier) {
    robotSpeedsSupplier = supplier != null ? supplier : ChassisSpeeds::new;
  }

  public void runHood(double speed) {
    io.runHood(speed);
  }

  public void stopHood() {
    io.stopHood();
  }

  public double getSimulatedCurrentDrawAmps() {
    if (io instanceof ShooterIOSim simIo) {
      return simIo.data.currentAmps;
    }
    return 0.0;
  }

  @Override
  public void periodic() {
      ChassisSpeeds robotSpeeds = robotSpeedsSupplier.get();
      if (robotSpeeds != null) {
        io.setRobotVelocity(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
      }
      double flywheelRPM = io.getFlywheelRPM();
      double flywheelTargetRPM = io.getFlywheelTargetRPM();
      boolean shooterReady = io.isShooterReady();

      SmartDashboard.putNumber("Shooter/FlywheelRPM",flywheelRPM);
      SmartDashboard.putNumber("Shooter/FlywheelTargetRPM",flywheelTargetRPM);
      if (Constants.currentMode != Constants.Mode.REAL) {
        Logger.recordOutput("Shooter/State", state.name());
        Logger.recordOutput("Shooter/FlywheelRPM", flywheelRPM);
        Logger.recordOutput("Shooter/FlywheelTargetRPM", flywheelTargetRPM);
        Logger.recordOutput("Shooter/Ready", shooterReady);
        if (io instanceof ShooterIOSim simIo) {
          Logger.recordOutput("Shooter/AppliedVolts", simIo.data.appliedVolts);
          Logger.recordOutput("Shooter/CurrentAmps", simIo.data.currentAmps);
          Logger.recordOutput("Shooter/LegacyFlywheelRPM", simIo.data.legacyFlywheelRPM);
          Logger.recordOutput("Shooter/LegacyReady", simIo.data.legacyReady);
          Logger.recordOutput("Shooter/SOTMFlywheelRPM", simIo.data.sotmFlywheelRPM);
          Logger.recordOutput("Shooter/SOTMReady", simIo.data.sotmReady);
          Logger.recordOutput("Shooter/SOTMShotActive", simIo.data.sotmShotActive);
          Logger.recordOutput("Shooter/SOTMConfidence", simIo.data.sotmConfidence);
          Logger.recordOutput("Shooter/SOTMTofSec", simIo.data.sotmTofSec);
          Logger.recordOutput(
              "Shooter/SOTMDragCompensatedTofSec", simIo.data.sotmDragCompensatedTofSec);
          Logger.recordOutput("Shooter/ShotSourceIsSotm", simIo.data.shotSourceIsSotm);
          Logger.recordOutput("Shooter/SOTMNotePose", simIo.data.sotmNotePose);
          Logger.recordOutput("Shooter/SOTMTrajectory", simIo.data.sotmTrajectory);
        }
      }
      
      //System.out.println(io.getFlywheelRPM());

       /*  Logger.processInputs("HoodedShooter/data", data);

        Logger.recordOutput("HoodedShooter/state", getState());

        Logger.recordOutput("HoodedShooter/realAngle", getAngle().getDegrees());
        

        Logger.recordOutput("autoStowEnabled", autoStowEnabled);
        Logger.recordOutput("stateBeforeAutoStow", stateBeforeAutoStow);*/
      SmartDashboard.putData("Shooter/Shooter_PID_Controller", shooterPidController);

  }
}
