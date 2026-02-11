package frc.robot.subsystems.Climb.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.Climb.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbIO;

/**
 * Real hardware implementation for the Climb subsystem using Kraken X60 motors.
 * 
 * Motor Configuration:
 * - 2x Kraken X60 FOC motors (CAN FD via Phoenix 6)
 * - Total gearing: 36:1 (5:1 * 4:1 * 1.8:1)
 * - 22-tooth sprocket, 1/4" pitch
 * - Integrated encoder: 2048 counts per revolution
 */
public class ClimbIOReal implements ClimbIO {
    
    // Kraken X60 motors (TalonFX controllers)
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    // Control requests (Phoenix 6 pattern)
    private final VoltageOut voltageRequest = new VoltageOut(0);
    
    // Data container
    private final ClimbDataAutoLogged data;
    
    /**
     * Position conversion factor calculation:
     * 
     * Motor shaft rotations → Linear distance
     * 
     * 1. One motor rotation = (2 * PI * drum_radius) / gearing meters
     * 2. Drum circumference = 2 * PI * radius
     * 3. Total = (2 * PI * radius) / gearing
     * 
     * For our setup:
     * - Drum radius = 0.0221 meters (22 teeth * 0.25 inch pitch / (2*PI))
     * - Gearing = 36:1
     * - Factor = (2 * PI * 0.0221) / 36 = 0.00386 meters per motor rotation
     */
    private static final double POSITION_CONVERSION_FACTOR = 
        (2.0 * Math.PI * ClimbConstants.DRUM_RADIUS_METERS) / ClimbConstants.GEARING;
    
    public ClimbIOReal(ClimbDataAutoLogged data) {
        this.data = data;
        
        // Initialize motors
        leftMotor = new TalonFX(ClimbConstants.LEFT_MOTOR_ID, "rio"); // Update CAN bus name
        rightMotor = new TalonFX(ClimbConstants.RIGHT_MOTOR_ID, "rio");
        
        // Configure motors
        configureMotor(leftMotor, ClimbConstants.LEFT_MOTOR_INVERTED);
        configureMotor(rightMotor, ClimbConstants.RIGHT_MOTOR_INVERTED);
        
        // Set brake mode
        setBrakeMode(true);
        
        // Reset encoders to zero
        resetEncoder();
    }
    
    /**
     * Configures a single Kraken motor with optimal settings for elevator/climb
     */
    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output settings
        config.MotorOutput.Inverted = inverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits (tune based on testing)
        // Kraken can handle 366A stall, but limit for safety and battery life
        config.CurrentLimits.SupplyCurrentLimit = 40.0; // Continuous limit
        config.CurrentLimits.SupplyCurrentThreshold = 60.0; // Peak limit
        config.CurrentLimits.SupplyTimeThreshold = 0.5; // Time at peak before limiting
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        config.CurrentLimits.StatorCurrentLimit = 80.0; // Stator current limit
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Voltage compensation
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        
        // Soft limits (safety - prevents hardware damage)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
            ClimbConstants.MAX_HEIGHT_METERS / POSITION_CONVERSION_FACTOR; // Convert to rotations
        
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
            ClimbConstants.MIN_HEIGHT_METERS / POSITION_CONVERSION_FACTOR;
        
        // Apply configuration
        motor.getConfigurator().apply(config);
    }
    
    @Override
    public void updateData() {
        // Read left motor
        data.leftPositionMeters = leftMotor.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR;
        data.leftVelocityMetersPerSec = leftMotor.getVelocity().getValueAsDouble() * POSITION_CONVERSION_FACTOR;
        data.leftAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        data.leftCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        data.leftTempCelsius = leftMotor.getDeviceTemp().getValueAsDouble();
        
        // Read right motor
        data.rightPositionMeters = rightMotor.getPosition().getValueAsDouble() * POSITION_CONVERSION_FACTOR;
        data.rightVelocityMetersPerSec = rightMotor.getVelocity().getValueAsDouble() * POSITION_CONVERSION_FACTOR;
        data.rightAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        data.rightCurrentAmps = rightMotor.getSupplyCurrent().getValueAsDouble();
        data.rightTempCelsius = rightMotor.getDeviceTemp().getValueAsDouble();
        
        // Calculate averages
        data.avgPositionMeters = (data.leftPositionMeters + data.rightPositionMeters) / 2.0;
        data.avgVelocityMetersPerSec = (data.leftVelocityMetersPerSec + data.rightVelocityMetersPerSec) / 2.0;
        data.avgCurrentAmps = (data.leftCurrentAmps + data.rightCurrentAmps) / 2.0;
        
        // Calculate synchronization delta
        data.positionDeltaMeters = data.leftPositionMeters - data.rightPositionMeters;
        
        // Update primary data fields (for compatibility)
        data.height = edu.wpi.first.units.Units.Meters.of(data.avgPositionMeters);
        data.velocity = edu.wpi.first.units.Units.MetersPerSecond.of(data.avgVelocityMetersPerSec);
        
        // Acceleration can be calculated via differentiation if needed
        // For now, set to 0 (would need to track previous velocity and dt)
        data.accel = edu.wpi.first.units.Units.MetersPerSecondPerSecond.of(0);
    }
    
    @Override
    public void setVoltage(double volts) {
        // Clamp voltage to safe range
        volts = Math.max(-12.0, Math.min(12.0, volts));
        
        // Send voltage command to both motors
        leftMotor.setControl(voltageRequest.withOutput(volts));
        rightMotor.setControl(voltageRequest.withOutput(volts));
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = mode;
        
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
    }
    
    @Override
    public void resetEncoder() {
        leftMotor.setPosition(0.0);
        rightMotor.setPosition(0.0);
    }
    
    @Override
    public void setEncoderPosition(double positionMeters) {
        double rotations = positionMeters / POSITION_CONVERSION_FACTOR;
        leftMotor.setPosition(rotations);
        rightMotor.setPosition(rotations);
    }
    
    @Override
    public void configureCurrentLimit(double currentLimit, double triggerThreshold, double triggerThresholdTime) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = triggerThreshold;
        config.CurrentLimits.SupplyTimeThreshold = triggerThresholdTime;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
    }
    
    @Override
    public ClimbData getData() {
        return data;
    }
}