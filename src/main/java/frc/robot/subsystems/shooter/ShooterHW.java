package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.generated.Constants;

class ShooterHW implements ShooterIO {
    CANBus rioCanBus = new CANBus("rio");

    // initialize shooter, shooter intake, and hood motors
    private final TalonFX krakenShooterLeft = new TalonFX(14, rioCanBus);
    private final TalonFX krakenShooterRight = new TalonFX(2, rioCanBus);
    private final TalonFX krakenShooterFeeder = new TalonFX(5, rioCanBus);
    private final VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0); // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage feederRequest = new VelocityVoltage(0).withSlot(0);

    ShooterHW() {
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
        TalonFXConfiguration shooterFeederConfig = new TalonFXConfiguration();
        shooterFeederConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        shooterFeederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterFeederConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooterFeederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterFeederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterFeederConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterFeederConfig.Voltage.PeakReverseVoltage = -12.0;
        krakenShooterFeeder.getConfigurator().apply(shooterFeederConfig);

        // setup PID
        Slot0Configs shooterFeederPID = new Slot0Configs();
        shooterFeederPID.kS = Constants.Shooter.FEEDER_KS;
        shooterFeederPID.kV = Constants.Shooter.FEEDER_KV;
        shooterFeederPID.kP = Constants.Shooter.FEEDER_KP;
        shooterFeederPID.kI = Constants.Shooter.FEEDER_KI;
        shooterFeederPID.kD = Constants.Shooter.FEEDER_KD;
        krakenShooterFeeder.getConfigurator().apply(shooterFeederPID);
    }
    //sets shooter motor rpm
    @Override
    public void setRPM(double rpm) {
        double rps = rpm / 60.0;
        krakenShooterLeft.setControl(shooterRequest.withVelocity(-rps));
    }
    //gets shooter motor rpm
    @Override
    public double getRPM() {
        return krakenShooterLeft.getVelocity().getValueAsDouble() * 60.0;
    }
    //stops shooter by coasting to 0, rather than setting 0rpm and the shooter braking
    @Override
    public void stopShooter() {
        krakenShooterLeft.set(0);
    }
    //feeder is on/off
    @Override
    public void setFeeder(boolean on) {
        double rps = Constants.Shooter.FEEDER_RPM / 60;
        if(on) {
            krakenShooterFeeder.setControl(feederRequest.withVelocity(-rps));
        } else {
            krakenShooterFeeder.setControl(feederRequest.withVelocity(0));
        }
    }
    //check if feeder is running
    @Override
    public boolean getFeeder() {
        return krakenShooterFeeder.getVelocity().getValueAsDouble() > 2; //margin of error of 120rpm/2rps
    }
}