package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.Constants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakeState;


public class Intake{
    public enum IntakeState {
            IDLE(false, 0,IntakeConstants.stowedAngleDeg),
            DEPLOYED(true,Constants.IntakeConstants.rollerSpeed,IntakeConstants.intakeAngleDeg),
            WIGGLING(true, 0,80);

            public final boolean intakeExtended;
            public final double rollerSpeed;
            public final double pivotAngle;

            private IntakeState(boolean extended, double rollerSpeed, double pivotAngle) {
                this.intakeExtended = extended;
                this.rollerSpeed = rollerSpeed;
                this.pivotAngle = pivotAngle;
            }
    }
    
    private final TalonFX leftPivotMotor = new TalonFX(IntakeConstants.leftPivotMotorId);
    private final TalonFX rightPivotMotor = new TalonFX(IntakeConstants.rightPivotMotorId);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId);
    private IntakeState state = IntakeState.IDLE;


    private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
    //private final DutyCycleOut rollerControl = new DutyCycleOut(0);

    // Current pivot setpoint in degrees (converted to motor rotations when commanded).
    private double pivotTargetDeg = Constants.IntakeConstants.stowedAngleDeg;


    // Configure motors and start in IDLE.
    public Intake() {
       // zeroPivot();
        
        configurePivot();
        configureRoller();
    }

    // Apply Motion Magic + PID + current limit + brake mode for the pivot.
    public void configurePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = IntakeConstants.pivotP;
        slot0.kI = IntakeConstants.pivotI;
        slot0.kD = IntakeConstants.pivotD;

        cfg.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.cruiseVelocityRps;
        cfg.MotionMagic.MotionMagicAcceleration = IntakeConstants.accelRps2;
        cfg.CurrentLimits.SupplyCurrentLimit = 10;
        
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration tempCFG = cfg;
        tempCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightPivotMotor.getConfigurator().apply(cfg);
        leftPivotMotor.getConfigurator().apply(tempCFG);
        rightPivotMotor.setControl(new Follower(leftPivotMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // TODO: pivot zeroing
    }

    // Apply current limit + coast mode for the roller.
    public void configureRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotor.getConfigurator().apply(cfg);
    }


    public Command pivot(double pos) {
        return Commands.runOnce(() -> {
              leftPivotMotor.setControl(pivotControl.withPosition(degreesToMotorRotations(pos)));
        });
    }


    public Command setStateRollers(double rollerSpeed) {
        return Commands.runOnce(() -> {
             rollerMotor.set(rollerSpeed);
        });
    }


    public Command getMotorPos() {
        return Commands.runOnce(() -> {
             SmartDashboard.putNumber("Arm Position", leftPivotMotor.getPosition().getValueAsDouble());
        });
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

    public boolean isPivotAtSetpoint(double targetDeg) {
        return Math.abs(getPivotAngle() - targetDeg) <= IntakeConstants.angleToleranceDeg;
    }

    public Command runRollers() {
        return Commands.runOnce(() -> {
            rollerMotor.set(IntakeConstants.rollerSpeed);
        });
    }

    public Command stopRollers() {
        return Commands.runOnce(() -> {
            rollerMotor.set(0.0);
        });
    }

    // Convert degrees to motor rotations (arm rotations scaled by gear ratio).
    private static double degreesToMotorRotations(double degrees) {
        final double armRot = degrees / 360.0;
        return armRot * IntakeConstants.motorRotationsPerArmRotation;
    }


        public Command zeroPivot() {
            return Commands.runOnce(() -> {
                leftPivotMotor.setPosition(0.0);
                pivotTargetDeg = IntakeConstants.stowedAngleDeg;
            });
        }
}