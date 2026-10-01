package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{
    private TalonFX motor;
    public Hopper(){
        motor = new TalonFX(7);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // adapts for different battery voltages
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motor.getConfigurator().apply(motorConfig);
    }
    public Command runHopper(){
        return runOnce(()->{setSpeed(-0.75);});
    }
    public Command runHopperBack(){
        return runOnce(()->{setSpeed(0.75);});
    }
    public Command stopHopper(){
        return runOnce(()->{setSpeed(0);});
    }

    void setSpeed(double speed){
        motor.set(speed);
    }
}
