package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.generated.Constants;

public class ShooterHW implements ShooterIO {
    private final CANBus rioCanBus = new CANBus("rio");

    private final TalonFX krakenShooterLeft = new TalonFX(14, rioCanBus);
    private final TalonFX krakenShooterRight = new TalonFX(2, rioCanBus);
    private final TalonFX krakenShooterFeeder = new TalonFX(5, rioCanBus);

    private final VelocityVoltage shooterRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage feederRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutral = new NeutralOut();

    public ShooterHW() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterConfig.Voltage.PeakReverseVoltage = -12.0;
        shooterConfig.Slot0.kS = Constants.Shooter.SHOOTER_KS;
        shooterConfig.Slot0.kV = Constants.Shooter.SHOOTER_KV;
        shooterConfig.Slot0.kP = Constants.Shooter.SHOOTER_KP;
        shooterConfig.Slot0.kI = Constants.Shooter.SHOOTER_KI;
        shooterConfig.Slot0.kD = Constants.Shooter.SHOOTER_KD;
        krakenShooterLeft.getConfigurator().apply(shooterConfig);
        krakenShooterRight.getConfigurator().apply(shooterConfig);
        krakenShooterRight.setControl(new Follower(krakenShooterLeft.getDeviceID(), MotorAlignmentValue.Opposed));

        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfig.Voltage.PeakForwardVoltage = 12.0;
        feederConfig.Voltage.PeakReverseVoltage = -12.0;
        feederConfig.Slot0.kS = Constants.Shooter.FEEDER_KS;
        feederConfig.Slot0.kV = Constants.Shooter.FEEDER_KV;
        feederConfig.Slot0.kP = Constants.Shooter.FEEDER_KP;
        feederConfig.Slot0.kI = Constants.Shooter.FEEDER_KI;
        feederConfig.Slot0.kD = Constants.Shooter.FEEDER_KD;
        krakenShooterFeeder.getConfigurator().apply(feederConfig);
    }

    @Override
    public void setRPM(double rpm) {
        krakenShooterLeft.setControl(shooterRequest.withVelocity(rpm / 60.0));
    }

    @Override
    public double getRPM() {
        return krakenShooterLeft.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void stopShooter() {
        krakenShooterLeft.setControl(neutral);
    }

    @Override
    public void setFeeder(boolean on) {
        if (on) {
            krakenShooterFeeder.setControl(feederRequest.withVelocity(Constants.Shooter.FEEDER_RPM / 60.0));
        } else {
            krakenShooterFeeder.setControl(neutral);
        }
    }

    @Override
    public boolean getFeeder() {
        return Math.abs(krakenShooterFeeder.getVelocity().getValueAsDouble()) > 2.0;
    }
}