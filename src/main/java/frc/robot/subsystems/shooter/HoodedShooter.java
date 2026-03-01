package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodedShooter extends SubsystemBase{
    TalonFX hoodMotor;
    public HoodedShooter(){
        //TODO: Set the device ID
        hoodMotor = new TalonFX(15);
        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        hoodMotorConfig.Voltage.PeakReverseVoltage = -12.0;
        hoodMotor.getConfigurator().apply(hoodMotorConfig);
    }
    public void moveHood(double speed){
        hoodMotor.set(speed);
    }
}
