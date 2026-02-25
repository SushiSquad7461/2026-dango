package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.Constants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakeState;


public class IntakeReal implements IntakeIO {
    
    
    private final TalonFX leftPivotMotor = new TalonFX(IntakeConstants.leftPivotMotorId);
    private final TalonFX rightPivotMotor = new TalonFX(IntakeConstants.rightPivotMotorId);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId);
    private IntakeState state = IntakeState.IDLE;


    private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
    private final DutyCycleOut rollerControl = new DutyCycleOut(0);

    // Current pivot setpoint in degrees (converted to motor rotations when commanded).
    private double pivotTargetDeg = Constants.IntakeConstants.stowedAngleDeg;


    // Configure motors and start in IDLE.
    public IntakeReal() {
       // zeroPivot();
        configurePivot();
        configureRoller();
        setState(IntakeState.IDLE);
    }

    // Apply Motion Magic + PID + current limit + brake mode for the pivot.
    @Override
    public void configurePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = IntakeConstants.pivotP;
        slot0.kI = IntakeConstants.pivotI;
        slot0.kD = IntakeConstants.pivotD;

        cfg.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.cruiseVelocityRps;
        cfg.MotionMagic.MotionMagicAcceleration = IntakeConstants.accelRps2;

        cfg.CurrentLimits.SupplyCurrentLimit = 5;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration tempCFG = cfg;
        tempCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightPivotMotor.getConfigurator().apply(cfg);
        leftPivotMotor.getConfigurator().apply(tempCFG);
        //rightPivotMotor.setControl(new Follower(leftPivotMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        // TODO: pivot zeroing
    }

    // Apply current limit + coast mode for the roller.
    @Override
    public void configureRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotor.getConfigurator().apply(cfg);
    }

    // Set high-level state; updates pivot setpoint and roller behavior.
    public void setState(IntakeState newState) {
        this.state = newState;
        pivotTargetDeg = this.state.intakeExtended ? IntakeConstants.intakeAngleDeg : IntakeConstants.stowedAngleDeg;
        leftPivotMotor.setControl(pivotControl.withPosition(degreesToMotorRotations(pivotTargetDeg)));
        rollerMotor.setControl(rollerControl.withOutput(newState.rollerSpeed));
    }

    @Override
    public void getMotorPos(){
        // Replace motor.getPosition() with your specific motor encoder method
        SmartDashboard.putNumber("Arm Position", leftPivotMotor.getPosition().getValueAsDouble());
    }

    public IntakeState getState() {
        return this.state;
    }

    // Pivot angle in degrees (from motor rotations via gear ratio).
    public double getPivotAngle() {
        final double motorRot = leftPivotMotor.getPosition().getValueAsDouble();
        final double armRot = motorRot / IntakeConstants.motorRotationsPerArmRotation;
        return armRot * 360.0;
    }
    public double getPivotTargetAngle(){
        return pivotTargetDeg;
    }

    // True when pivot is within tolerance of current target.
    public boolean isPivotAtTarget() {
        return Math.abs(getPivotAngle() - pivotTargetDeg) <= IntakeConstants.angleToleranceDeg;
    }



    public void runRollers(){
        rollerMotor.set(IntakeConstants.rollerSpeed);
    }
    public void stopRollers(){
        rollerMotor.set(0.0);
    }

    // Convert degrees to motor rotations (arm rotations scaled by gear ratio).
    private static double degreesToMotorRotations(double degrees) {
        final double armRot = degrees / 360.0;
        return armRot * IntakeConstants.motorRotationsPerArmRotation;
    }


    @Override
    public void zeroPivot() {
        leftPivotMotor.setPosition(0.0);
        pivotTargetDeg = IntakeConstants.stowedAngleDeg;    
    }
}