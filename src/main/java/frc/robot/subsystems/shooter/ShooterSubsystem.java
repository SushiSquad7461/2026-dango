// Shooter Subsystem: controls shooter state using a state machine, switches between IDLE, PRESHOOT, and SHOOT
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public enum ShooterState {
    IDLE, // shooter inactive
    PRESHOOT, // shooter spinning up, waiting for hood to come into position, or waiting for robot to turn to goal
    SHOOT // shooting
  }

  private ShooterState state = ShooterState.IDLE;
  private final ShooterIO io;

  private static final double TARGET_RPM = 4500;

  private double shootStartTime = 0; // could come in useful later, especially for logging

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void startShoot() {
    state = ShooterState.PRESHOOT;
  }

  public void stop() {
    state = ShooterState.IDLE;
  }

  @Override
  public void periodic() {
    switch (state) {

      case IDLE:
        io.stopFlywheel();
        io.stopFeeder();
        break;

      case PRESHOOT: // TODO: check whether shooter is at rpm before going to SHOOT state
        io.setFlywheelRPM(TARGET_RPM);
        shootStartTime = Timer.getFPGATimestamp();
        
      case SHOOT:
        io.setFlywheelRPM(TARGET_RPM);
        io.runFeeder();
        break;
    }
  }
}
