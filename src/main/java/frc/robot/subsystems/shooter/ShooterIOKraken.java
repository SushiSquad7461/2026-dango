/* Shooter configuration and motor control
Motors: Shooter Left, Shooter Right, Hood, Feeder
Motor control methods accessible through interface ShooterIO.java
*/ 
package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.Constants;

public class ShooterIOKraken implements ShooterIO {
  CANBus rioCanBus = new CANBus("rio");

  // initialize shooter, shooter intake, and hood motors
  private final TalonFX krakenShooterLeft = new TalonFX(5, rioCanBus);
  private final TalonFX krakenShooterRight = new TalonFX(2, rioCanBus);
  private final TalonFX krakenShooterKicker = new TalonFX(14, rioCanBus);
  private final VelocityDutyCycle shooterRequest = new VelocityDutyCycle(0).withSlot(0); // create a velocity closed-loop request, voltage output, slot 0 configs
  private final VelocityDutyCycle feederRequest = new VelocityDutyCycle(0).withSlot(0);

  public ShooterIOKraken() {
    // shooter flywheel motor config
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // adapts for different battery voltages
    shooterConfig.Voltage.PeakForwardVoltage = 12.0;
    shooterConfig.Voltage.PeakReverseVoltage = -12.0;
    krakenShooterLeft.getConfigurator().apply(shooterConfig);
    // sets second shooter motor w same config, opposite direction
    krakenShooterRight.setControl(new Follower(krakenShooterLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    // setup PID
    Slot0Configs shooterPID = new Slot0Configs();
    shooterPID.kS = Constants.Shooter.SHOOTER_KS;
    shooterPID.kV = Constants.Shooter.SHOOTER_KV; 
    shooterPID.kP = Constants.Shooter.SHOOTER_KP;
    shooterPID.kI = Constants.Shooter.SHOOTER_KI; 
    shooterPID.kD = Constants.Shooter.SHOOTER_KD;
    krakenShooterLeft.getConfigurator().apply(shooterPID);
    krakenShooterRight.getConfigurator().apply(shooterPID);

    // shooter intake wheel motor config
    TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();
    shooterIntakeConfig.CurrentLimits.StatorCurrentLimit = 50.0;
    shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterIntakeConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterIntakeConfig.Voltage.PeakForwardVoltage = 12.0;
    shooterIntakeConfig.Voltage.PeakReverseVoltage = -12.0;
    krakenShooterKicker.getConfigurator().apply(shooterIntakeConfig);

    // setup PID
    Slot0Configs shooterIntakePID = new Slot0Configs();
    shooterIntakePID.kS = Constants.Shooter.SHOOTER_INTAKE_KS;
    shooterIntakePID.kV = Constants.Shooter.SHOOTER_INTAKE_KV; 
    shooterIntakePID.kP = Constants.Shooter.SHOOTER_INTAKE_KP;
    shooterIntakePID.kI = Constants.Shooter.SHOOTER_INTAKE_KI; 
    shooterIntakePID.kD = Constants.Shooter.SHOOTER_INTAKE_KD;
    krakenShooterKicker.getConfigurator().apply(shooterIntakePID);
  }

  @Override
  public void setFlywheelRPM(double rpm) {
    double rps = rpm / 60;
    krakenShooterLeft.setControl(shooterRequest.withVelocity(rps));
  }

  @Override
  public void stopFlywheel() {
    krakenShooterLeft.setControl(shooterRequest.withVelocity(0));
  }

  @Override
  public void runFeeder() {
    double rps = Constants.Shooter.FEEDER_RPM / 60;
    krakenShooterKicker.setControl(feederRequest.withVelocity(rps));
  }

  @Override
  public void stopFeeder() {
    krakenShooterKicker.setControl(feederRequest.withVelocity(0));
  }

  @Override
  public double getFlywheelRPM() {
    return krakenShooterLeft.getVelocity().getValueAsDouble() * 60;
  }

  @Override
  public double getFlywheelTargetRPM() {
    return shooterRequest.Velocity * 60;
  }

  @Override
  public boolean isShooterReady() {
    return Math.abs(getFlywheelTargetRPM() - getFlywheelRPM()) < Constants.Shooter.SHOOTER_RPM_TOLERANCE;
  }

  public void periodic(){
    SmartDashboard.putNumber("Shooter/CurrentRPM", getFlywheelRPM());
    SmartDashboard.putNumber("Shooter/TargetRPM", getFlywheelTargetRPM());
  }
}
