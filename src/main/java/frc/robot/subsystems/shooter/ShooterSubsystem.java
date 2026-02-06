package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  public enum ShooterState {
    IDLE,
    PRESHOOT,
    SHOOT
  }

  private ShooterState state = ShooterState.IDLE;
  private final ShooterIO io;

  private static final double TARGET_RPM = 4500;
  private static final double FEED_TIME = 0.4;

  private double shootStartTime = 0;

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

      case PRESHOOT:
        io.setFlywheelRPM(TARGET_RPM);

        // rookie-friendly: RPM check works later, ignored now
        shootStartTime = Timer.getFPGATimestamp();
        state = ShooterState.SHOOT;
        break;

      case SHOOT:
        io.setFlywheelRPM(TARGET_RPM);
        io.runFeeder();

        if (Timer.getFPGATimestamp() - shootStartTime > FEED_TIME) {
          state = ShooterState.IDLE;
        }
        break;
    }
  }
}
