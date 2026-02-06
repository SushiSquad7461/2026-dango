package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOKrakenOpenLoop implements ShooterIO {

  private final TalonFX leftShooter = new TalonFX(0, "rio");
  private final TalonFX rightShooter = new TalonFX(1, "rio");
  private final TalonFX feeder = new TalonFX(2, "rio");

  public ShooterIOKrakenOpenLoop() {

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.StatorCurrentLimit = 100;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftShooter.getConfigurator().apply(shooterConfig);
    rightShooter.getConfigurator().apply(shooterConfig);

    // rookie-style manual inversion
    rightShooter.setInverted(true);

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits.StatorCurrentLimit = 30;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    feeder.getConfigurator().apply(feederConfig);
  }

  @Override
  public void setFlywheelRPM(double rpm) {
    // open-loop "good enough" for first iteration
    leftShooter.set(0.8);
    rightShooter.set(0.8);
  }

  @Override
  public void stopFlywheel() {
    leftShooter.set(0);
    rightShooter.set(0);
  }

  @Override
  public void runFeeder() {
    feeder.set(0.6);
  }

  @Override
  public void stopFeeder() {
    feeder.set(0);
  }

  @Override
  public double getFlywheelRPM() {
    // not implemented yet
    return 0;
  }
}
