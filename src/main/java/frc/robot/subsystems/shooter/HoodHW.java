package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.generated.Constants;



class HoodHW {
    private TalonFX hoodMotor;
    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);
    private double hoodSetpointDegrees = 0.0;
    private double lastCommandedDegrees = 0.0;

    HoodHW(){
        hoodMotor = new TalonFX(15);
        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        hoodMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        Slot0Configs slot0 = hoodMotorConfig.Slot0;
        slot0.kP = Constants.HoodedShooterConstants.hoodP;
        slot0.kI = Constants.HoodedShooterConstants.hoodI;
        slot0.kD = Constants.HoodedShooterConstants.hoodD;
        hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodedShooterConstants.cruiseVelocityRps;
        hoodMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodedShooterConstants.accelRps2;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 10;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

        zeroHood();
    }
    void setPosition(double angleInDegrees){
        hoodSetpointDegrees = MathUtil.clamp(
                angleInDegrees,
                Constants.HoodedShooterConstants.hoodMinDegrees,
                Constants.HoodedShooterConstants.hoodMaxDegrees
        );
        double rotations = (hoodSetpointDegrees / 360.0) * Constants.HoodedShooterConstants.motorRotationsPerHoodRotation;
        hoodMotor.setControl(hoodControl.withPosition(rotations));
    }

    double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble() * 360.0
                / Constants.HoodedShooterConstants.motorRotationsPerHoodRotation;
    }

    void zeroHood(){
        hoodMotor.setPosition(0);
        hoodSetpointDegrees = 0.0;
    }

    void stepHood(double deltaDegrees) {
        setPosition(hoodSetpointDegrees + deltaDegrees);
    }
}