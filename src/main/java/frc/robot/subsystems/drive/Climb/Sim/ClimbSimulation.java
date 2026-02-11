package frc.robot.subsystems.Climb.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.Climb.ClimbConstants;
import frc.robot.subsystems.Climb.ClimbIO;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Simulation implementation for the AndyMark Climber in a Box.
 * 
 * Key Differences from Standard Elevator:
 * - Spring-assisted extension (3.5 lb constant force springs)
 * - Winch-based drive (rope spooled on drum)
 * - Lighter mechanism weight (6 lbs) + robot weight when climbing
 * 
 * Physics Model:
 * - Gravity pulls robot down: F_gravity = m * g
 * - Springs push climber up: F_spring = constant
 * - Net force = F_gravity - F_spring (when loaded with robot)
 * - Motor provides additional force through winch drum
 * 
 * Uses WPILib's ElevatorSim with adjusted parameters for spring assistance.
 */
public class ClimbSimulation implements ClimbIO {
    
    // Simulation model
    private final ElevatorSim sim;
    
    // Data container
    private final ClimbDataAutoLogged data;
    
    // Commanded voltage
    private double appliedVolts = 0.0;
    
    // Simulation state tracking
    private double lastVelocity = 0.0;
    private double lastUpdateTime = 0.0;
    
    // Kraken X60 motor model (custom since not in WPILib yet)
    private static final DCMotor KRAKEN_X60 = new DCMotor(
        12.0,                                      // Nominal voltage (V)
        ClimbConstants.KRAKEN_STALL_TORQUE_NM,    // Stall torque (N⋅m) 
        ClimbConstants.KRAKEN_STALL_CURRENT_AMPS, // Stall current (A)
        ClimbConstants.KRAKEN_FREE_CURRENT_AMPS,  // Free current (A)
        ClimbConstants.KRAKEN_FREE_SPEED_RPM * (2 * Math.PI / 60.0),  // Free speed (rad/s)
        ClimbConstants.NUM_MOTORS                 // Number of motors
    );
    
    /**
     * Creates a new AndyMark Climber in a Box simulation.
     * 
     * NOTE: The ElevatorSim doesn't natively support spring assistance,
     * so we simulate it by reducing the effective mass that gravity acts on.
     * 
     * Effective mass reduction:
     * Spring force = 3.5 lbs = 15.6 N upward
     * Equivalent mass reduction = F_spring / g = 15.6 / 9.81 = 1.59 kg
     * 
     * So instead of simulating 120 kg robot with 15.6 N spring,
     * we simulate (120 - 1.59) = 118.4 kg robot with no spring.
     * 
     * @param data The data container to populate with simulated values
     */
    public ClimbSimulation(ClimbDataAutoLogged data) {
        this.data = data;
        
        // Calculate effective mass (accounting for spring assistance)
        double springForce = ClimbConstants.SPRING_FORCE_NEWTONS;
        double equivalentMassReduction = springForce / 9.81; // F = ma, so m = F/a
        double effectiveMass = ClimbConstants.CARRIAGE_MASS_KG - equivalentMassReduction;
        
        // Create elevator simulation with spring-adjusted mass
        // Parameters: motor, gearing, carriage mass, drum radius, min height, max height, simulate gravity, measurement std dev
        this.sim = new ElevatorSim(
            KRAKEN_X60,
            ClimbConstants.GEARING,
            effectiveMass, // Reduced mass to account for spring assistance
            ClimbConstants.WINCH_DRUM_RADIUS_METERS,
            ClimbConstants.MIN_HEIGHT_METERS,
            ClimbConstants.MAX_HEIGHT_METERS,
            ClimbConstants.SIMULATE_GRAVITY,
            0.002  // 2mm measurement noise (rope stretch, encoder quantization)
        );
        
        // Set initial position (fully retracted, springs compressed)
        sim.setState(ClimbConstants.STARTING_HEIGHT_METERS, 0.0);
        
        System.out.println("=== Climber Simulation Initialized ===");
        System.out.println("Effective mass (with spring assistance): " + effectiveMass + " kg");
        System.out.println("Winch drum radius: " + ClimbConstants.WINCH_DRUM_RADIUS_METERS + " m");
        System.out.println("Theoretical max velocity: " + 
            ClimbConstants.calculateTheoreticalMaxVelocity() + " m/s");
        System.out.println("Theoretical max force: " + 
            ClimbConstants.calculateTheoreticalMaxForce() + " N");
        System.out.println("======================================");
    }
    
    @Override
    public void updateData() {
        // Update simulation (20ms default robot loop time)
        double currentTime = System.currentTimeMillis() / 1000.0;
        double dt = currentTime - lastUpdateTime;
        if (dt < 0.01) dt = 0.020; // Default to 20ms if first call
        
        sim.update(dt);
        lastUpdateTime = currentTime;
        
        // Get simulated position and velocity
        double position = sim.getPositionMeters();
        double velocity = sim.getVelocityMetersPerSecond();
        
        // Calculate acceleration via finite difference
        double acceleration = (velocity - lastVelocity) / dt;
        lastVelocity = velocity;
        
        // Simulate current draw
        double currentDraw = sim.getCurrentDrawAmps();
        
        // Split current between motors (should be equal in ideal case)
        double currentPerMotor = currentDraw / ClimbConstants.NUM_MOTORS;
        
        // Simulate individual motor readings with realistic variation
        // Real motors won't be perfectly synchronized - add small noise
        double leftPositionNoise = (Math.random() - 0.5) * 0.0005; // 0.5mm noise
        double rightPositionNoise = (Math.random() - 0.5) * 0.0005;
        
        data.leftPositionMeters = position + leftPositionNoise;
        data.rightPositionMeters = position + rightPositionNoise;
        
        data.leftVelocityMetersPerSec = velocity + (Math.random() - 0.5) * 0.01;
        data.rightVelocityMetersPerSec = velocity + (Math.random() - 0.5) * 0.01;
        
        data.leftAppliedVolts = appliedVolts;
        data.rightAppliedVolts = appliedVolts;
        
        // Add realistic current variation (motors not perfectly balanced)
        data.leftCurrentAmps = currentPerMotor * (0.95 + Math.random() * 0.1); // ±5% variation
        data.rightCurrentAmps = currentPerMotor * (0.95 + Math.random() * 0.1);
        
        // Simulate realistic motor temperature
        data.leftTempCelsius = simulateTemperature(data.leftCurrentAmps, data.leftTempCelsius);
        data.rightTempCelsius = simulateTemperature(data.rightCurrentAmps, data.rightTempCelsius);
        
        // Calculate averages
        data.avgPositionMeters = position;
        data.avgVelocityMetersPerSec = velocity;
        data.avgCurrentAmps = currentDraw / ClimbConstants.NUM_MOTORS;
        
        // Calculate synchronization delta (should be very small in sim)
        data.positionDeltaMeters = data.leftPositionMeters - data.rightPositionMeters;
        
        // Update primary data fields (for compatibility with units)
        data.height = edu.wpi.first.units.Units.Meters.of(position);
        data.velocity = edu.wpi.first.units.Units.MetersPerSecond.of(velocity);
        data.accel = edu.wpi.first.units.Units.MetersPerSecondPerSecond.of(acceleration);
    }
    
    @Override
    public void setVoltage(double volts) {
        // Clamp voltage to realistic battery range (accounting for voltage sag)
        double batteryVoltage = RobotController.getBatteryVoltage();
        volts = Math.max(-batteryVoltage, Math.min(batteryVoltage, volts));
        
        this.appliedVolts = volts;
        sim.setInputVoltage(volts);
    }
    
    @Override
    public void setBrakeMode(boolean enable) {
        // Simulation doesn't need to track brake mode
        // In real hardware this affects behavior when voltage = 0
        // We could simulate it by adding friction, but not critical for now
    }
    
    @Override
    public void stop() {
        setVoltage(0.0);
    }
    
    @Override
    public void resetEncoder() {
        sim.setState(0.0, sim.getVelocityMetersPerSecond());
    }
    
    @Override
    public void setEncoderPosition(double positionMeters) {
        sim.setState(positionMeters, sim.getVelocityMetersPerSecond());
    }
    
    @Override
    public ClimbData getData() {
        return data;
    }
    
    /**
     * Simulates motor temperature based on current draw.
     * 
     * Temperature model:
     * - Heating: Proportional to I²R (current squared)
     * - Cooling: Proportional to (T - T_ambient)
     * 
     * @param currentAmps Current draw in amps
     * @param currentTemp Current temperature in Celsius
     * @return New temperature in Celsius
     */
    private double simulateTemperature(double currentAmps, double currentTemp) {
        final double AMBIENT_TEMP = 25.0; // Celsius (room temperature)
        final double HEATING_COEFFICIENT = 0.0008; // Degrees per amp² per cycle
        final double COOLING_COEFFICIENT = 0.015; // Fraction of temp difference per cycle
        final double MAX_TEMP = 95.0; // Celsius (realistic max before thermal throttling)
        
        // Heat generation: I²R losses
        // Higher current = more heat (quadratic relationship)
        double heating = currentAmps * currentAmps * HEATING_COEFFICIENT;
        
        // Cooling: Newton's law of cooling
        // Rate proportional to temperature difference
        double cooling = (currentTemp - AMBIENT_TEMP) * COOLING_COEFFICIENT;
        
        // Update temperature
        double newTemp = currentTemp + heating - cooling;
        
        // Clamp to realistic range
        return Math.max(AMBIENT_TEMP, Math.min(MAX_TEMP, newTemp));
    }
    
    /**
     * Gets the underlying WPILib simulation object.
     * Useful for advanced testing or visualization.
     * 
     * @return The ElevatorSim object
     */
    public ElevatorSim getSimulation() {
        return sim;
    }
    
    /**
     * Simulates the effect of the robot loading onto the climber.
     * Call this when the robot hooks onto the bar and starts to climb.
     * 
     * This doesn't change the simulation parameters (mass is already set),
     * but it's here as a placeholder if you want to add load-detection logic.
     */
    public void simulateRobotLoad(boolean loaded) {
        // In a more advanced simulation, you could:
        // 1. Change the effective mass based on whether robot is loaded
        // 2. Simulate rope stretch under load
        // 3. Add extra friction when loaded
        // 
        // For now, we assume the robot is always loaded during climb
    }
}