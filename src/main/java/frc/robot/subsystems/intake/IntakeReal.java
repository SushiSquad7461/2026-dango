package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.Constants;
import frc.robot.generated.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakeState;


public class IntakeReal implements IntakeIO {
    
    
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.pivotMotorId);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId);
    private IntakeState state = IntakeState.IDLE;


    private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
    private final DutyCycleOut rollerControl = new DutyCycleOut(0);

    // Current pivot setpoint in degrees (converted to motor rotations when commanded).
    private double pivotTargetDeg = Constants.IntakeConstants.stowedAngleDeg;

    // Wiggle edge detector: flip target once per arrival at setpoint.
    private boolean wiggleTargetHigh = false;
    private boolean wiggleReady = true;

    // Configure motors and start in IDLE.
    public IntakeReal() {
        zeroPivot();
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

        cfg.CurrentLimits.SupplyCurrentLimit = 40;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotMotor.getConfigurator().apply(cfg);
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

        if (this.state == IntakeState.WIGGLING) {
            wiggleTargetHigh = false;
            wiggleReady = true;
            pivotTargetDeg = IntakeConstants.wiggleLowDeg;
        } else {
            pivotTargetDeg = this.state.intakeExtended ? IntakeConstants.intakeAngleDeg : IntakeConstants.stowedAngleDeg;
        }

        updateRollers();
    }

    public IntakeState getState() {
        return this.state;
    }

    // Pivot angle in degrees (from motor rotations via gear ratio).
    public double getPivotAngle() {
        final double motorRot = pivotMotor.getPosition().getValueAsDouble();
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

    // Command pivot Motion Magic to current target.
    @Override
    public void runPivotToTarget() {
        double targetRot = degreesToMotorRotations(pivotTargetDeg);
        pivotMotor.setControl(pivotControl.withPosition(targetRot));
    }

    public void runRollers(){
        rollerMotor.set(IntakeConstants.rollerSpeed);
    }
    public void stopRollers(){
        rollerMotor.set(0.0);
    }

    // Handle state transitions (deploy/stow completion and wiggle target flips).
   public void changeIfWiggle(boolean atTarget) {
        if (state == IntakeState.WIGGLING) {
            if (atTarget && wiggleReady) {
                wiggleTargetHigh = !wiggleTargetHigh;
                pivotTargetDeg = wiggleTargetHigh ? IntakeConstants.wiggleHighDeg : IntakeConstants.wiggleLowDeg;
                wiggleReady = false;
            }
            if (!atTarget) {
                wiggleReady = true;
            }
            return;
        } else{

        }
    }

    // Update rollers; only runs when pivot is at target.
    public void updateRollers() {
        boolean atTarget = isPivotAtTarget();
        double out = 0.0;
        if (atTarget) {
            switch (state.direction) {
                case FORWARD -> out = +IntakeConstants.rollerSpeed;
                case REVERSE -> out = -IntakeConstants.rollerSpeed;
                case OFF -> out = 0.0;
            }
        }
        out = MathUtil.clamp(out, -1.0, 1.0);
        rollerMotor.setControl(rollerControl.withOutput(out));
    }

    // Periodic loop: command pivot and update rollers and log telemetry.
    public void periodic() {

        // SmartDashboard.putString("Intake/State", state.name());
        // SmartDashboard.putNumber("Intake/PivotDeg", getPivotAngle());
        // SmartDashboard.putNumber("Intake/PivotTargetDeg", pivotTargetDeg);
        // SmartDashboard.putBoolean("Intake/PivotAtTarget", isPivotAtTarget());
    }

    // Convert degrees to motor rotations (arm rotations scaled by gear ratio).
    private static double degreesToMotorRotations(double degrees) {
        final double armRot = degrees / 360.0;
        return armRot * IntakeConstants.motorRotationsPerArmRotation;
    }


    @Override
    public void zeroPivot() {
        pivotMotor.setPosition(0.0);
        pivotTargetDeg = IntakeConstants.stowedAngleDeg;    
    }
}