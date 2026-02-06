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

import frc.robot.Direction;

public class Intake extends SubsystemBase {

    public static final class IntakeConstants {
        public static final int pivotMotorId = 10;
        public static final int rollerMotorId = 11;

        public static final double intakeAngleDeg = 125.0;
        public static final double angleToleranceDeg = 5.0;
        public static final double stowedAngleDeg = 0.0;

        public static final double motorRotationsPerArmRotation = 100.0;

        public static final double cruiseVelocityRps = 6.0;
        public static final double accelRps2 = 12.0;

        public static final double pivotP = 40.0;
        public static final double pivotI = 0.0;
        public static final double pivotD = 0.5;

        public static final double rollerPercent = 0.70; // base roller percent
    }

    private IntakeState state = IntakeState.IDLE;

    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.pivotMotorId);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.rollerMotorId);

    private final MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
    private final DutyCycleOut rollerControl = new DutyCycleOut(0);

    private double pivotTargetDeg = IntakeConstants.stowedAngleDeg;

    public Intake() {
        configurePivot();
        configureRoller();
        setState(IntakeState.IDLE);
    }

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
    }

    private void configureRoller() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = 30;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerMotor.getConfigurator().apply(cfg);
    }

    public void setState(IntakeState newState) {
        state = newState;

        // Set pivot target
        pivotTargetDeg = state.intakeExtended ? IntakeConstants.intakeAngleDeg : IntakeConstants.stowedAngleDeg;

        // Set rollers
        switch(state.direction) {
            case FORWARD -> rollerMotor.setControl(rollerControl.withOutput(IntakeConstants.rollerPercent));
            case REVERSE -> rollerMotor.setControl(rollerControl.withOutput(-IntakeConstants.rollerPercent));
            default -> rollerMotor.setControl(rollerControl.withOutput(0));
        }
    }

    public IntakeState getState() {
        return state;
    }

    public double getPivotAngle() {
        double motorRot = pivotMotor.getPosition().getValueAsDouble();
        double armRot = motorRot / IntakeConstants.motorRotationsPerArmRotation;
        return armRot * 360.0;
    }

    private void runPivotToTarget() {
        double targetRot = degreesToMotorRotations(pivotTargetDeg);
        pivotMotor.setControl(pivotControl.withPosition(targetRot));
    }

    @Override
    public void periodic() {
        runPivotToTarget();

        SmartDashboard.putString("Intake/State", state.name());
        SmartDashboard.putNumber("Intake/PivotDeg", getPivotAngle());
    }

    private static double degreesToMotorRotations(double degrees) {
        double armRot = degrees / 360.0;
        return armRot * IntakeConstants.motorRotationsPerArmRotation;
    }
}
