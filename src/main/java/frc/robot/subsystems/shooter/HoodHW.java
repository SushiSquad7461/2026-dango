package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.generated.Constants;



public class HoodHW implements HoodIO {
    private TalonFX hoodMotor;
    private final MotionMagicVelocityVoltage hoodControlV = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);
    private double hoodSetpointDegrees = 0.0;
    private double lastCommandedDegrees = 0.0;

    public HoodHW(){
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

    //TODO: Make the degrees negative if hood moves in the wrong direction
    public void setPosition(double angleInDegrees){
        // Convert hood degrees → motor rotations via gear ratio
        double rotations = (angleInDegrees / 360.0) * Constants.HoodedShooterConstants.motorRotationsPerHoodRotation;
        lastCommandedDegrees = angleInDegrees;
        hoodMotor.setControl(hoodControl.withPosition(rotations));
    }

    @Override
    public double getPosition() {
        return 0;
    }

    public void zeroHood(){
        hoodMotor.setPosition(0);
    }


    /** D-pad: steps the hood setpoint by ±5° and holds position via Motion Magic. */
    public void stepHood(double deltaDegrees) {
        hoodSetpointDegrees = MathUtil.clamp(
                hoodSetpointDegrees + deltaDegrees,
                Constants.HoodedShooterConstants.hoodMinDegrees,
                Constants.HoodedShooterConstants.hoodMaxDegrees
        );
        setPosition(hoodSetpointDegrees);
    }
}
