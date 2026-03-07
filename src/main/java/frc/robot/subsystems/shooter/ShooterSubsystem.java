// Shooter Subsystem: controls shooter state using a state machine, switches between IDLE, PRESHOOT, and SHOOT
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;

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
                io.stopShooter();    
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
