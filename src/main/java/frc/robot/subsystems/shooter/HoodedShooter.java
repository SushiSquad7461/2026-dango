package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.generated.Constants;



public class HoodedShooter extends SubsystemBase{
    private TalonFX hoodMotor;
    private final MotionMagicVelocityVoltage hoodControlV = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);
    private double hoodSetpointDegrees = 0.0;

    public HoodedShooter(){
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
    public void moveHoodToSetpoint(double angleInDegrees){
        // Convert hood degrees → motor rotations via gear ratio
        double rotations = (angleInDegrees / 360.0) * Constants.HoodedShooterConstants.motorRotationsPerHoodRotation;
        hoodMotor.setControl(hoodControl.withPosition(rotations));
    }

    public void zeroHood(){
        hoodMotor.setPosition(0);
    }



    public void moveHood(double speed){
            hoodMotor.setControl(hoodControlV.withVelocity((speed)));
    }

    /** D-pad: steps the hood setpoint by ±5° and holds position via Motion Magic. */
    public void stepHood(double deltaDegrees) {
        hoodSetpointDegrees = MathUtil.clamp(
            hoodSetpointDegrees + deltaDegrees,
            Constants.HoodedShooterConstants.hoodMinDegrees,
            Constants.HoodedShooterConstants.hoodMaxDegrees
        );
        moveHoodToSetpoint(hoodSetpointDegrees);
    }

    /** SOTM: commands hood to LUT-looked-up base angle + driver trim offset. */
    public void moveHoodToAngleWithOffset(double baseAngleDegrees) {
        double target = MathUtil.clamp(
            baseAngleDegrees + hoodSetpointDegrees,
            Constants.HoodedShooterConstants.hoodMinDegrees,
            Constants.HoodedShooterConstants.hoodMaxDegrees
        );
        moveHoodToSetpoint(target);
    }

    @Override
    public void periodic(){
        // Convert motor rotations back to hood degrees using gear ratio
        double actualHoodDegrees = hoodMotor.getPosition().getValueAsDouble()
                / Constants.HoodedShooterConstants.motorRotationsPerHoodRotation * 360.0;
        SmartDashboard.putNumber("HoodedShooter/HoodAngle", actualHoodDegrees);
        SmartDashboard.putNumber("HoodedShooter/HoodSetpoint", hoodSetpointDegrees);
        SmartDashboard.putNumber("HoodedShooter/HoodTarget", (hoodControl.getPositionMeasure().in(Degrees)));
        SmartDashboard.putNumber("HoodedShooter/StatorCurrent", hoodMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("HoodedShooter/MotorRotations", hoodMotor.getPosition().getValueAsDouble());
    }
}
