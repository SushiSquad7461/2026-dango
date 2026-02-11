package frc.robot.subsystems.Climb;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;

/**
 * Hardware constants for the AndyMark Climber in a Box (2-Stage)
 * Product: am-4668
 * 
 * Physical System:
 * - 2-stage telescoping aluminum tubes (2" → 1.5" → 1")
 * - Spring-out, winch-back mechanism
 * - 3.5 lb constant force springs for extension
 * - 3mm Dyneema rope for retraction
 * - 2x Kraken X60 motors with custom gearing
 * 
 * Gearing Configuration:
 * - Stage 1: 5:1 cartridge
 * - Stage 2: 4:1 cartridge
 * - Chain reduction: 18:10 (1.8:1)
 * - Total: 36:1
 * 
 * @see https://andymark.com/products/climber-in-a-box
 */
public class ClimbConstants {
    
    // ==================== MOTOR CONFIGURATION ====================
    
    public static final int LEFT_MOTOR_ID = 20;  // Update with actual CAN ID
    public static final int RIGHT_MOTOR_ID = 21; // Update with actual CAN ID
    public static final boolean LEFT_MOTOR_INVERTED = false;
    public static final boolean RIGHT_MOTOR_INVERTED = true; // Typically opposite side
    public static final int NUM_MOTORS = 2; // Dual Kraken X60 motors
    
    // ==================== GEARING ====================
    
    /**
     * Gearing Breakdown:
     * - 5:1 cartridge on one stage
     * - 4:1 cartridge on another stage
     * - 18:10 chain reduction (1.8:1)
     * Total: 5 * 4 * (18/10) = 36:1
     */
    public static final double GEARING = 5.0 * 4.0 * (18.0 / 10.0); // 36:1 total reduction
    
    // ==================== KRAKEN X60 SPECIFICATIONS ====================
    
    public static final double KRAKEN_FREE_SPEED_RPM = 6000.0; // Kraken free speed
    public static final double KRAKEN_STALL_TORQUE_NM = 7.09; // Kraken stall torque (N⋅m)
    public static final double KRAKEN_STALL_CURRENT_AMPS = 366.0; // Kraken stall current
    public static final double KRAKEN_FREE_CURRENT_AMPS = 19.0; // Kraken free current
    
    // ==================== PHYSICAL SPECIFICATIONS ====================
    
    /**
     * AndyMark Climber in a Box Physical Properties:
     * - 2 Stage configuration weighs 6 lbs (without motor/gearbox)
     * - Uses 2", 1.5", and 1" square aluminum tubing
     * - Each stage has 24.5" of travel
     * - Total extended height: ~49" (2 stages × 24.5")
     * - Carriage mass includes robot + climber mechanism
     */
    
    // Mass of robot being lifted (estimate - update based on actual robot)
    public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(120); // Typical FRC robot weight
    
    /**
     * Winch Configuration:
     * The Climber in a Box uses a winch drum to spool rope.
     * The winch kit comes with a specific drum diameter that we need to measure.
     * 
     * Standard winch drum is typically 1.5-2" diameter for this application.
     * UPDATE THIS VALUE after measuring the actual winch drum!
     */
    public static final double WINCH_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0); // MEASURE AND UPDATE!
    
    // ==================== TRAVEL LIMITS ====================
    
    /**
     * Height Limits:
     * - Minimum: Fully retracted (springs compressed)
     * - Maximum: Fully extended (both stages at max travel)
     * - Each stage: 24.5" travel per AndyMark specs
     */
    public static final double MIN_HEIGHT_METERS = 0.0; // Fully retracted
    public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(49.0); // 2 stages × 24.5"
    public static final double SINGLE_STAGE_TRAVEL_METERS = Units.inchesToMeters(24.5);
    
    /**
     * Starting position:
     * Springs push the climber out to extended position by default.
     * We'll start with it retracted (winched in) for stowed position.
     */
    public static final double STARTING_HEIGHT_METERS = 0.0; // Start retracted
    
    /**
     * Base height:
     * Height of the climber mounting point above ground when robot is on the ground.
     * UPDATE THIS based on your robot's design!
     */
    public static final double BASE_HEIGHT_METERS = Units.inchesToMeters(36.0); // Estimate - update!
    
    // ==================== SPRING SPECIFICATIONS ====================
    
    /**
     * Constant Force Spring:
     * - 3.5 lb force per spring (from AndyMark description)
     * - Springs push the stages outward
     * - Motor must overcome spring force to retract
     */
    public static final double SPRING_FORCE_NEWTONS = Units.lbsToKilograms(3.5) * 9.81; // ~15.4 N per spring
    
    // ==================== SIMULATION ====================
    
    public static final boolean SIMULATE_GRAVITY = true;
    
    // ==================== TOLERANCES ====================
    
    // Position tolerance for "at goal" detection (0.5 inches)
    public static final double STATE_MARGIN_OF_ERROR = Units.inchesToMeters(0.5);
    
    // Maximum allowed position difference between left and right motors
    public static final double MAX_SYNC_ERROR_METERS = Units.inchesToMeters(2.0); // 2 inches
    
    // ==================== ENCODER CONFIGURATION ====================
    
    public static final double ENCODER_ZERO_OFFSET = 0.0; // Calibrate on robot
    
    // ==================== CONTROL GAINS ====================
    
    /**
     * Control gains and constraints for the climb mechanism.
     * 
     * TUNING NOTES:
     * - kG must overcome spring force AND gravity
     * - Springs provide ~15.4N upward force
     * - Gravity provides ~1177N downward force (120kg robot)
     * - Net force = gravity - spring = 1177 - 15.4 = 1162N downward
     */
    public static class ClimbControl {
        
        // ========== PID GAINS ==========
        
        // Proportional: How aggressively to correct position error
        public static LoggedTunableNumber kP = new LoggedTunableNumber(
            "/Tuning/Climb/kP", 
            60.0 // Higher than typical due to heavy load
        );
        
        // Integral: Accumulates error over time (usually not needed for climber)
        public static LoggedTunableNumber kI = new LoggedTunableNumber(
            "/Tuning/Climb/kI", 
            0.0
        );
        
        // Derivative: Damps oscillation and overshoot
        public static LoggedTunableNumber kD = new LoggedTunableNumber(
            "/Tuning/Climb/kD", 
            1.0
        );
        
        // ========== FEEDFORWARD GAINS ==========
        
        /**
         * kG: Gravity compensation (volts to hold position)
         * 
         * Calculation:
         * Net downward force = (robot_mass * g) - spring_force
         *                    = (120kg * 9.81) - 15.4N = 1162N
         * 
         * Torque needed = force * radius = 1162 * 0.0254m = 29.5 N⋅m
         * Motor torque available = stall_torque * gearing * num_motors * efficiency
         *                        = 7.09 * 36 * 2 * 0.8 = 410 N⋅m (way more than needed!)
         * 
         * Voltage ratio = torque_needed / torque_available * 12V
         *               = 29.5 / 410 * 12 = 0.86V
         * 
         * However, this is a rough estimate. Actual kG depends on:
         * - Actual winch drum radius (NEEDS MEASUREMENT!)
         * - Spring mounting and geometry
         * - Friction in the system
         * 
         * Start with 0.5V and tune up/down based on drift
         */
        public static LoggedTunableNumber kG = new LoggedTunableNumber(
            "/Tuning/Climb/kG", 
            0.5 // START HERE, tune based on drift
        );
        
        // kS: Static friction (volts to overcome friction at zero velocity)
        public static LoggedTunableNumber kS = new LoggedTunableNumber(
            "/Tuning/Climb/kS", 
            0.25 // Moderate friction in telescoping tubes
        );
        
        /**
         * kV: Velocity feedforward (volts per m/s)
         * 
         * Calculation:
         * Max motor speed = 6000 RPM = 100 rev/s = 628 rad/s
         * Linear velocity = (motor_speed / gearing) * drum_circumference
         *                 = (628 / 36) * (2 * π * 0.0254)
         *                 = 2.78 m/s theoretical max
         * 
         * kV = 12V / max_velocity = 12 / 2.78 = 4.3 V/(m/s)
         * 
         * But this assumes we measured the drum correctly!
         */
        public static LoggedTunableNumber kV = new LoggedTunableNumber(
            "/Tuning/Climb/kV", 
            4.3 // Based on 1" drum radius - UPDATE after measurement!
        );
        
        // kA: Acceleration feedforward (volts per m/s²)
        public static LoggedTunableNumber kA = new LoggedTunableNumber(
            "/Tuning/Climb/kA", 
            0.4
        );
        
        // ========== MOTION CONSTRAINTS ==========
        
        /**
         * Maximum velocity:
         * Conservative setting for safety and control.
         * Can be increased after testing.
         */
        public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber(
            "/Tuning/Climb/maxVelocity", 
            1.0 // m/s - conservative start, can go up to ~2.5 m/s theoretically
        );
        
        /**
         * Maximum acceleration:
         * Limit jerk and motor current during motion.
         */
        public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
            "/Tuning/Climb/maxAcceleration",
            3.0 // m/s² - smooth acceleration
        );
    }
    
    // ==================== CLIMB STATES ====================
    
    /**
     * Predefined climb states with target heights.
     * 
     * Heights are relative to the fully retracted position (0").
     * Adjust these based on your game requirements and robot design.
     */
    public enum ClimbStates {
        STOW(Units.inchesToMeters(0)),        // Fully retracted - stored position
        HOOK_DEPLOY(Units.inchesToMeters(36)), // Extend to hook onto bar
        CLIMB_START(Units.inchesToMeters(24)), // Partial retraction to start climb
        CLIMB_FINISH(Units.inchesToMeters(2)), // Nearly fully retracted - elevated
        MAX_EXTEND(MAX_HEIGHT_METERS);         // Maximum extension
        
        public final double heightMeters;
        
        private ClimbStates(double heightMeters) {
            this.heightMeters = heightMeters;
        }
        
        /**
         * Get the target height as a Distance unit
         */
        public edu.wpi.first.units.measure.Distance getHeight() {
            return edu.wpi.first.units.Units.Meters.of(heightMeters);
        }
    }
    
    // ==================== THEORETICAL CALCULATIONS ====================
    
    /**
     * Calculate theoretical max velocity based on motor specs and winch drum.
     * 
     * WARNING: This depends on accurate WINCH_DRUM_RADIUS measurement!
     * 
     * @return Maximum velocity in m/s
     */
    public static double calculateTheoreticalMaxVelocity() {
        double motorRPM = KRAKEN_FREE_SPEED_RPM;
        double radsPerSec = (motorRPM / 60.0) * 2.0 * Math.PI;
        double outputRadsPerSec = radsPerSec / GEARING;
        double linearVelocity = outputRadsPerSec * WINCH_DRUM_RADIUS_METERS;
        return linearVelocity;
    }
    
    /**
     * Calculate theoretical max force the climber can exert.
     * 
     * This is the force available to lift the robot (minus spring assistance).
     * 
     * @return Maximum force in Newtons
     */
    public static double calculateTheoreticalMaxForce() {
        double motorTorque = KRAKEN_STALL_TORQUE_NM * NUM_MOTORS;
        double outputTorque = motorTorque * GEARING * 0.8; // 80% efficiency estimate
        double force = outputTorque / WINCH_DRUM_RADIUS_METERS;
        return force; // This is total force - subtract spring force for net downward pull
    }
    
    /**
     * Calculate net force available for lifting robot (accounting for springs).
     * 
     * @return Net lifting force in Newtons
     */
    public static double calculateNetLiftingForce() {
        // Springs help by pushing up, reducing the force needed from motors
        return calculateTheoreticalMaxForce() - SPRING_FORCE_NEWTONS;
    }
    
    /**
     * Calculate weight capacity (max robot weight that can be lifted).
     * 
     * @return Maximum liftable weight in kg
     */
    public static double calculateWeightCapacity() {
        double netForce = calculateNetLiftingForce();
        return netForce / 9.81; // Convert force to mass
    }
}