package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants.IntakeConstants;



public class HoodedShooter extends SubsystemBase{
    private TalonFX hoodMotor;
    private final MotionMagicVoltage hoodControl = new MotionMagicVoltage(0);

    public HoodedShooter(){
        hoodMotor = new TalonFX(15);
        TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();
        hoodMotorConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        hoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        hoodMotorConfig.Voltage.PeakReverseVoltage = -12.0;
        
        Slot0Configs slot0 = hoodMotorConfig.Slot0;
        slot0.kP = IntakeConstants.pivotP;
        slot0.kI = IntakeConstants.pivotI;
        slot0.kD = IntakeConstants.pivotD;
        hoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.cruiseVelocityRps;
        hoodMotorConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.accelRps2;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 10;
        hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodMotor.getConfigurator().apply(hoodMotorConfig);

        zeroHood();
    }

    //TODO: Make the degrees negative if hood moves in the wrong direction
    public void moveHoodToSetpoint(double angleInDegrees){
        //Converts to motor rotations
        hoodMotor.setControl(hoodControl.withPosition(angleInDegrees/360));
    }

    public void zeroHood(){
        hoodMotor.setPosition(0);
    }

    //Use this method to set HoodedShooter angle based on these
    public double calculateDesiredAngle(double distanceToHub, double speed){
        //TODO: Change y (height between shooter and hub) 
        return Math.toDegrees(getLowAngle(distanceToHub, 8, speed));
    }

    /*
     * Calculates the angle (helper method, dont use anywhere else)
     * 
     * distance to hub & height (x&y): meters
     * speed (v): meters/second
     * g (gravity constant): metrs per second squared
     * 
     * returns in radian
     */
    private static double getLowAngle(double x, double y, double v) {
        double inside = Math.pow(v,4) - 9.81*(9.81*Math.pow(x,2) + 2*y*Math.pow(v,2));
        if (inside < 0) return 0;
        double sqrt = Math.sqrt(inside);
        return Math.atan((Math.pow(v,2) - sqrt) / (9.81*x));
    }

    // public static double getHighAngle(double x, double y, double v) {
    //     double inside = Math.pow(v,4) - 9.81*(9.81*Math.pow(x,2) + 2*y*Math.pow(v,2));
    //     if (inside < 0) return 0;
    //     double sqrt = Math.sqrt(inside);
    //     return Math.atan((Math.pow(v,2) + sqrt) / (9.81*x));
    // }

    public void moveHood(double speed){
        hoodMotor.set(speed);
    }

    public void periodic(){
        SmartDashboard.putNumber("HoodedShooter/HoodAngle", (hoodMotor.getPosition().getValueAsDouble())/360.0);
    }
}
