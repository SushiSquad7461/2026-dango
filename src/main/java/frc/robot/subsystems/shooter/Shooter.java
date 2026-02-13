/* Shooter Subsystem: initializes shooter motors, controls speed and hood position using PID, etc.
 * TODO: State machine to control shooter behavior between IDLE, PRESHOOT, and SHOOT
 * TODO: Add shooter control methods (set target rpm, target hood position)
 * TODO: Tune PID
*/
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    // TODO: tune shooter PID
    public final double SHOOTER_KS = 0.1;
    public final double SHOOTER_KV = 0.12;
    public final double SHOOTER_KP = 0.11;
    public final double SHOOTER_KI = 0;
    public final double SHOOTER_KD = 0;
    // TODO: tune shooter intake PID
    public final double SHOOTER_INTAKE_KS = 0.1;
    public final double SHOOTER_INTAKE_KV = 0.12;
    public final double SHOOTER_INTAKE_KP = 0.11;
    public final double SHOOTER_INTAKE_KI = 0;
    public final double SHOOTER_INTAKE_KD = 0;
    // TODO: tune hood PID
    public final double HOOD_KP = 2.4;
    public final double HOOD_KI = 0;
    public final double HOOD_KD = 0.1;

    // initialize shooter, shooter intake, and hood motors
    private final TalonFX krakenShooterLeft = new TalonFX(0, "rio");
    private final TalonFX krakenShooterRight = new TalonFX(1, "rio");
    private final TalonFX krakenShooterIntake = new TalonFX(2, "rio");
    private final TalonFX krakenShooterHood = new TalonFX(3, "rio");

    public Shooter() {
        // shooter flywheel motor config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.CurrentLimits.StatorCurrentLimit = 120.0;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // adapts for different battery voltages
        shooterConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterConfig.Voltage.PeakReverseVoltage = -12.0;
        krakenShooterLeft.getConfigurator().apply(shooterConfig);
        krakenShooterRight.getConfigurator().apply(shooterConfig);
        // sets second shooter motor w same config, opposite direction
        krakenShooterRight.setControl(new Follower(krakenShooterLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        // setup PID
        Slot0Configs shooterPID = new Slot0Configs();
        shooterPID.kS = SHOOTER_KS;
        shooterPID.kV = SHOOTER_KV; 
        shooterPID.kP = SHOOTER_KP;
        shooterPID.kI = SHOOTER_KI; 
        shooterPID.kD = SHOOTER_KD;
        krakenShooterLeft.getConfigurator().apply(shooterPID);

        // shooter intake wheel motor config
        TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterIntakeConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterIntakeConfig.Voltage.PeakReverseVoltage = -12.0;
        krakenShooterIntake.getConfigurator().apply(shooterIntakeConfig);
        // setup PID
        Slot0Configs shooterIntakePID = new Slot0Configs();
        shooterIntakePID.kS = SHOOTER_INTAKE_KS;
        shooterIntakePID.kV = SHOOTER_INTAKE_KV; 
        shooterIntakePID.kP = SHOOTER_INTAKE_KP;
        shooterIntakePID.kI = SHOOTER_INTAKE_KI; 
        shooterIntakePID.kD = SHOOTER_INTAKE_KD;
        krakenShooterIntake.getConfigurator().apply(shooterIntakePID);

        // shooter adjustable hood motor config
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // note the brake behavior because we need to stop the hood in place
        krakenShooterHood.getConfigurator().apply(hoodConfig);
        // setup PID
        Slot0Configs hoodPID = new Slot0Configs();
        hoodPID.kP = HOOD_KP;
        hoodPID.kI = HOOD_KI; 
        hoodPID.kD = HOOD_KD;
        krakenShooterHood.getConfigurator().apply(hoodPID);
    }
}
