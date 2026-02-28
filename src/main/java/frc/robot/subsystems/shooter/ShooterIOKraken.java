/* Shooter configuration and motor control
Motors: Shooter Left, Shooter Right, Hood, Feeder
Motor control methods accessible through interface ShooterIO.java
*/ 
package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.generated.Constants;

public class ShooterIOKraken implements ShooterIO {
  CANBus rioCanBus = new CANBus("rio");

  // initialize shooter, shooter intake, and hood motors
  private final TalonFX krakenShooterLeft = new TalonFX(5, rioCanBus);
  private final TalonFX krakenShooterRight = new TalonFX(2, rioCanBus);
  private final TalonFX krakenShooterKicker = new TalonFX(14, rioCanBus);
 // private final TalonFX krakenShooterHood = new TalonFX(3, rioCanBus);

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

        // shooter intake wheel motor config
        TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
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

        // shooter adjustable hood motor config
  //       TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  //       hoodConfig.CurrentLimits.StatorCurrentLimit = 20.0;
  //       hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  //       hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // note the brake behavior because we need to stop the hood in place
  //       krakenShooterHood.getConfigurator().apply(hoodConfig);
  //       // setup PID
  //       Slot0Configs hoodPID = new Slot0Configs();
  //       hoodPID.kP = Constants.Shooter.HOOD_KP;
  //       hoodPID.kI = Constants.Shooter.HOOD_KI; 
  //       hoodPID.kD = Constants.Shooter.HOOD_KD;
  //       krakenShooterHood.getConfigurator().apply(hoodPID);
   }

  @Override
  public void setFlywheelRPM(double rpm) {
    // open-loop "good enough" for first iteration
    krakenShooterLeft.set(-0.8);
  }

  @Override
  public void stopFlywheel() {
    krakenShooterLeft.set(-0.2);
  }

  @Override
  public void runFeeder() { // open-loop "good enough" for first iteration
    krakenShooterKicker.set(-1);
  }

  @Override
  public void stopFeeder() {
    krakenShooterKicker.set(0);
  }

  @Override
  public double getFlywheelRPM() {
    // TODO: implement checking flywheel RPM
    return 0;
  }

  // TODO: implement hood angle check
  public double getHoodPos() {
    return 0;
  }
  // TODO: implement shooter readiness check
  public boolean isShooterReady() {
    return true;
  }
}