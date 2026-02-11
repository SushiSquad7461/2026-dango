package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Hardware abstraction layer for the Climb subsystem.
 * 
 * Hardware Configuration:
 * - 2x Kraken X60 motors (FOC-capable brushless motors)
 * - Gearing: 5:1 cartridge * 4:1 cartridge * 18:10 chain = 36:1 total
 * - Drum: 22-tooth sprocket, 1/4" pitch
 * - Encoder: Integrated Kraken encoder (2048 CPR)
 * 
 * This interface allows switching between real hardware and simulation
 * through AdvantageKit's IO layer pattern.
 */
public interface ClimbIO {
    
    /**
     * Container for all logged climb data.
     * AdvantageKit will automatically generate getters/setters and logging code.
     */
    @AutoLog
    public static class ClimbData {
        // Position, velocity, and acceleration
        public Distance height = Meters.of(0);
        public LinearVelocity velocity = MetersPerSecond.of(0);
        public LinearAcceleration accel = MetersPerSecondPerSecond.of(0);
        
        // Left motor (Kraken X60)
        public double leftPositionMeters = 0.0;
        public double leftVelocityMetersPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;
        
        // Right motor (Kraken X60)
        public double rightPositionMeters = 0.0;
        public double rightVelocityMetersPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;
        
        // Average values (computed)
        public double avgPositionMeters = 0.0;
        public double avgVelocityMetersPerSec = 0.0;
        public double avgCurrentAmps = 0.0;
        
        // Motor synchronization tracking
        public double positionDeltaMeters = 0.0; // Difference between left and right
    }

    /**
     * Updates all sensor readings and motor states.
     * This should be called every loop cycle (20ms default) to refresh data.
     */
    public void updateData();

    /**
     * Commands both motors to run at the specified voltage.
     * Voltage is clamped to [-12, 12] volts automatically by motor controllers.
     * 
     * @param volts Desired output voltage (-12 to 12)
     */
    public void setVoltage(double volts);

    /**
     * Enable or disable brake mode on both motors.
     * 
     * Brake mode (true): Motors actively resist motion when not powered (high holding force)
     * Coast mode (false): Motors spin freely when not powered (no holding force)
     * 
     * For climb mechanisms, brake mode is typically preferred for safety.
     * 
     * @param enable true for brake mode, false for coast mode
     */
    public void setBrakeMode(boolean enable);

    /**
     * Stops both motors immediately by setting voltage to 0.
     * Note: This does not disable brake mode - motors will hold position if brake is enabled.
     */
    public default void stop() {
        setVoltage(0.0);
    }

    /**
     * Resets the encoder position to zero.
     * Should be called when the climb is at a known position (e.g., fully retracted).
     */
    public default void resetEncoder() {
        // Default implementation does nothing - override in implementation classes
    }

    /**
     * Sets the encoder position to a specific value.
     * Useful for calibration or when starting from a known position.
     * 
     * @param positionMeters The position to set in meters
     */
    public default void setEncoderPosition(double positionMeters) {
        // Default implementation does nothing - override in implementation classes
    }

    /**
     * Configures current limits for the motors to prevent brownouts and damage.
     * 
     * @param currentLimit Maximum continuous current in amps
     * @param triggerThreshold Current threshold to start limiting in amps
     * @param triggerThresholdTime Time above threshold before limiting in seconds
     */
    public default void configureCurrentLimit(double currentLimit, 
                                             double triggerThreshold, 
                                             double triggerThresholdTime) {
        // Default implementation does nothing - override in implementation classes
    }

    /**
     * Sets the conversion factor for encoder readings to meters.
     * This is calculated based on gearing and drum radius.
     * 
     * @param factor Conversion factor (motor rotations to meters)
     */
    public default void setPositionConversionFactor(double factor) {
        // Default implementation does nothing - override in implementation classes
    }

    /**
     * Checks if both motors are within synchronization tolerance.
     * 
     * @param toleranceMeters Maximum allowed position difference in meters
     * @return true if motors are synchronized within tolerance
     */
    public default boolean areMotorsSynchronized(double toleranceMeters) {
        return Math.abs(getData().positionDeltaMeters) <= toleranceMeters;
    }

    /**
     * Gets the current climb data.
     * This is mainly for internal use - subsystem should track its own data object.
     * 
     * @return Current ClimbData object
     */
    public default ClimbData getData() {
        return new ClimbData();
    }
}