package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public static final class IntakeConstants {
        public static final int pivotMotorId = 10;
        public static final int rollerMotorId = 11;

        public static final double intakeAngleDeg = 125.0;
        public static final double angleToleranceDeg = 5.0;
        public static final double stowedAngleDeg = 0.0;

        public static final double motorRotationsPerArmRotation = 72.0;

        public static final double cruiseVelocityRps = 6.0;
        public static final double accelRps2 = 12.0;

        public static final double pivotP = 40.0;
        public static final double pivotI = 0.0;
        public static final double pivotD = 0.5;

        public static final double rollerPercent = 0.70;

        public static final double wiggleLowDeg = 90.0;
        public static final double wiggleHighDeg = 110.0;
    }

    private IntakeState state = IntakeState.IDLE;

    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.pivotMotorId);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId);

    private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
    private final DutyCycleOut rollerControl = new DutyCycleOut(0);

    // Current pivot setpoint in degrees (converted to motor rotations when commanded).
    private double pivotTargetDeg = IntakeConstants.stowedAngleDeg;

    // Wiggle edge detector: flip target once per arrival at setpoint.
    private boolean wiggleTargetHigh = false;
    private boolean wiggleReady = true;

    // Configure motors and start in IDLE.
    public Intake() {
        configurePivot();
        configureRoller();
        setState(IntakeState.IDLE);
    }

    // Apply Motion Magic + PID + current limit + brake mode for the pivot.
    private void configurePivot() {
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
    private void configureRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotor.getConfigurator().apply(cfg);
    }

    // Set high-level state; updates pivot setpoint and roller behavior.
    public void setState(IntakeState newState) {
        state = newState;

        if (state == IntakeState.WIGGLING) {
            wiggleTargetHigh = false;
            wiggleReady = true;
            pivotTargetDeg = IntakeConstants.wiggleLowDeg;
        } else {
            pivotTargetDeg = state.intakeExtended ? IntakeConstants.intakeAngleDeg : IntakeConstants.stowedAngleDeg;
        }

        updateRollers();
    }

    public IntakeState getState() {
        return state;
    }

    // Pivot angle in degrees (from motor rotations via gear ratio).
    public double getPivotAngle() {
        final double motorRot = pivotMotor.getPosition().getValueAsDouble();
        final double armRot = motorRot / IntakeConstants.motorRotationsPerArmRotation;
        return armRot * 360.0;
    }

    // True when pivot is within tolerance of current target.
    private boolean isPivotAtTarget() {
        return Math.abs(getPivotAngle() - pivotTargetDeg) <= IntakeConstants.angleToleranceDeg;
    }

    // Command pivot Motion Magic to current target.
    private void runPivotToTarget() {
        double targetRot = degreesToMotorRotations(pivotTargetDeg);
        pivotMotor.setControl(pivotControl.withPosition(targetRot));
    }

    // Handle state transitions (deploy/stow completion and wiggle target flips).
    private void updateState(boolean at) {
        if (state == IntakeState.WIGGLING) {
            if (at && wiggleReady) {
                wiggleTargetHigh = !wiggleTargetHigh;
                pivotTargetDeg = wiggleTargetHigh ? IntakeConstants.wiggleHighDeg : IntakeConstants.wiggleLowDeg;
                wiggleReady = false;
            }
            if (!at) {
                wiggleReady = true;
            }
            return;
        }
        if (state == IntakeState.DEPLOYING && at) {
            state = IntakeState.DEPLOYED;
        }
        if (state == IntakeState.STOWING && at) {
            state = IntakeState.STOWED;
        }
    }

    // Update rollers; only runs when pivot is at target.
    private void updateRollers() {
        boolean at = isPivotAtTarget();
        updateState(at);
        double out = 0.0;
        if (at) {
            switch (state.direction) {
                case FORWARD -> out = +IntakeConstants.rollerPercent;
                case REVERSE -> out = -IntakeConstants.rollerPercent;
                case OFF -> out = 0.0;
            }
        }
        out = MathUtil.clamp(out, -1.0, 1.0);
        rollerMotor.setControl(rollerControl.withOutput(out));
    }

    // Periodic loop: command pivot and update rollers and log telemetry.
    @Override
    public void periodic() {
        runPivotToTarget();
        updateRollers();

        SmartDashboard.putString("Intake/State", state.name());
        SmartDashboard.putNumber("Intake/PivotDeg", getPivotAngle());
        SmartDashboard.putNumber("Intake/PivotTargetDeg", pivotTargetDeg);
        SmartDashboard.putBoolean("Intake/PivotAtTarget", isPivotAtTarget());
    }

    // Convert degrees to motor rotations (arm rotations scaled by gear ratio).
    private static double degreesToMotorRotations(double degrees) {
        double armRot = degrees / 360.0;
        return armRot * IntakeConstants.motorRotationsPerArmRotation;
    }
}