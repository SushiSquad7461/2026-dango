// Shooter Subsystem: controls shooter state using a state machine, switches between IDLE, PRESHOOT, and SHOOT
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private double shootStartTime = 0; // could come in useful later, especially for logging
  private final PIDController shooterPidController = new PIDController(Constants.Shooter.SHOOTER_KP, Constants.Shooter.SHOOTER_KI, Constants.Shooter.SHOOTER_KD);
  private double targetRPM = Constants.Shooter.TARGET_RPM_0;

  public enum ShooterState {
    IDLE, // shooter inactive
    PRESHOOT, // shooter spinning up, waiting for hood to come into position, or waiting for robot to turn to goal
    SHOOT // shooting
  }

  private ShooterState state = ShooterState.IDLE;
  private final ShooterIO io;
  //ShooterDataAutoLogged data = new ShooterDataAutoLogged();

  public LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 5);


  public ShooterSubsystem(ShooterIO io) {
    this.io = io;

    LoggedNetworkNumber voltage = new LoggedNetworkNumber("HoodedShooter/voltage", 0);
    LoggedNetworkNumber kG = new LoggedNetworkNumber("HoodedShooter/kG", 0);
    LoggedNetworkNumber kP = new LoggedNetworkNumber("HoodedShooter/kP", 0);
    LoggedNetworkNumber kI = new LoggedNetworkNumber("HoodedShooter/kI", 0);
    LoggedNetworkNumber kD = new LoggedNetworkNumber("HoodedShooter/kD", 0);
    LoggedNetworkNumber kS = new LoggedNetworkNumber("HoodedShooter/kS", 0);
    LoggedNetworkNumber kV = new LoggedNetworkNumber("HoodedShooter/kV", 0);
    LoggedNetworkNumber kA = new LoggedNetworkNumber("HoodedShooter/kA", 0);


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
                io.stopFlywheel();    
            }),
            Commands.runOnce(()->{
                io.stopFeeder();
            }));

      case PRESHOOT: // TODO: check whether shooter is at rpm before going to SHOOT state
        return Commands.parallel(
            Commands.runOnce(()->{
                io.setFlywheelRPM(targetRPM);
            }),
            Commands.runOnce(()->{
                shootStartTime = Timer.getFPGATimestamp();
            }));
        
      case SHOOT:
        return Commands.parallel(
            Commands.runOnce(()->{
                io.setFlywheelRPM(targetRPM);
            }),
            Commands.runOnce(()->{
                io.runFeeder();
            }));
      default:
        return Commands.none();
    }

  }

  public Command runFeederBack(){
   return Commands.runOnce(()->io.runFeederBack());
  }
  public Command stopFeeder() {
   return Commands.runOnce(()->io.stopFeeder());
  }
  public void setTargetRPM(double distance) {
    double rpm = distance * Constants.Shooter.RPM_DISTANCE_MULTIPLIER + Constants.Shooter.RPM_DISTANCE_OFFSET;
    this.targetRPM = rpm;
  }

  @Override
  public void periodic() {

      SmartDashboard.putNumber("Shooter/FlywheelRPM",io.getFlywheelRPM());
      SmartDashboard.putNumber("Shooter/FlywheelTargetRPM",io.getFlywheelTargetRPM());
      
      //System.out.println(io.getFlywheelRPM());

       /*  Logger.processInputs("HoodedShooter/data", data);

        Logger.recordOutput("HoodedShooter/state", getState());

        Logger.recordOutput("HoodedShooter/realAngle", getAngle().getDegrees());
        

        Logger.recordOutput("autoStowEnabled", autoStowEnabled);
        Logger.recordOutput("stateBeforeAutoStow", stateBeforeAutoStow);*/
      SmartDashboard.putData("Shooter/Shooter_PID_Controller", shooterPidController);

  }
}
