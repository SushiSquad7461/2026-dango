// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbConstants.ClimbControl;
import frc.robot.subsystems.Climb.ClimbConstants.ClimbStates;
import frc.robot.subsystems.Climb.sim.ClimbSimulation;
// import frc.robot.subsystems.Climb.real.ClimbIOReal; // Uncomment for real robot
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Subsystem for controlling the Robot Climb mechanism.
 * 
 * Hardware:
 * - 2x Kraken X60 motors
 * - 36:1 total gearing (5:1 * 4:1 * 1.8:1)
 * - 22-tooth sprocket, 1/4" pitch
 * 
 * Control Strategy:
 * - ProfiledPIDController for smooth, trapezoidal motion profiles
 * - ElevatorFeedforward for gravity and friction compensation
 * - Dual motor synchronization
 * 
 * Uses AdvantageKit for logging and simulation-ready IO abstraction.
 */
public class Climb extends SubsystemBase {

    // Hardware Interface
    private ClimbIO climbIO;
    private final ClimbDataAutoLogged data = new ClimbDataAutoLogged();

    // Subsystem State
    private ClimbStates state = ClimbStates.STOW;

    // Controllers
    private ProfiledPIDController profile;
    private ElevatorFeedforward feedforward;

    // AdvantageKit Mechanism2d Visualization (for AdvantageScope)
    private final LoggedMechanism2d mech = new LoggedMechanism2d(2.0, 3.0); // 2m wide, 3m tall canvas
    private final LoggedMechanismRoot2d root = mech.getRoot("ClimbBase", 1.0, 0.2);
    
    // Main climb arm - changes length based on extension
    private final LoggedMechanismLigament2d climbArm = root.append(
        new LoggedMechanismLigament2d(
            "ClimbArm", 
            ClimbConstants.STARTING_HEIGHT_METERS, 
            90.0,  // Vertical (90 degrees)
            3.0,   // Line width
            new Color8Bit(Color.kOrange)
        )
    );
    
    // Carriage at the top of the arm
    private final LoggedMechanismLigament2d carriage = climbArm.append(
        new LoggedMechanismLigament2d(
            "Carriage",
            0.15,  // 15cm wide carriage
            0.0,   // Horizontal
            8.0,   // Thicker line
            new Color8Bit(Color.kYellow)
        )
    );
    
    // Goal indicator - shows where we're trying to go
    private final LoggedMechanismRoot2d goalRoot = mech.getRoot("GoalIndicator", 0.2, 0.2);
    private final LoggedMechanismLigament2d goalLine = goalRoot.append(
        new LoggedMechanismLigament2d(
            "Goal",
            0.0,  // Will be updated
            90.0,
            2.0,
            new Color8Bit(Color.kGreen)
        )
    );
    
    // Reference lines for min/max positions
    private final LoggedMechanismRoot2d minLineRoot = mech.getRoot("MinHeight", 1.5, 0.2);
    private final LoggedMechanismLigament2d minLine = minLineRoot.append(
        new LoggedMechanismLigament2d(
            "Min",
            0.3,  // 30cm horizontal line
            0.0,
            1.0,
            new Color8Bit(Color.kRed)
        )
    );
    
    private final LoggedMechanismRoot2d maxLineRoot = mech.getRoot("MaxHeight", 1.5, 0.2 + ClimbConstants.MAX_HEIGHT_METERS);
    private final LoggedMechanismLigament2d maxLine = maxLineRoot.append(
        new LoggedMechanismLigament2d(
            "Max",
            0.3,  // 30cm horizontal line
            0.0,
            1.0,
            new Color8Bit(Color.kRed)
        )
    );

    // Static tracking for global height reference
    public static double heightMeters = 0.0;
    
    // Debug tracking
    private double appliedVoltage = 0.0;
    private double lastProfileVelocity = 0.0;

    /**
     * Initializes the Climb subsystem.
     * Switches between Real and Simulation IO based on robot state.
     */
    public Climb() {
        // Initialize IO layer
        if (RobotBase.isReal()) {
            // climbIO = new ClimbIOReal(data); // Uncomment for real robot
            climbIO = new ClimbSimulation(data); // Temporary - using sim for now
        } else {
            climbIO = new ClimbSimulation(data);
        }

        // Initialize controllers with tunable values
        updateControllers();
        
        // Sync the profile with current height to prevent jumps on startup
        profile.reset(data.height.in(Meters));
        
        // Set brake mode for safety
        climbIO.setBrakeMode(true);
    }

    /**
     * Creates new controller instances with current tunable values.
     * Called during initialization and when tunables are updated.
     */
    private void updateControllers() {
        // Create ProfiledPIDController with motion constraints
        profile = new ProfiledPIDController(
            ClimbControl.kP.get(), 
            ClimbControl.kI.get(), 
            ClimbControl.kD.get(),
            new Constraints(
                ClimbControl.maxVelocity.get(),
                ClimbControl.maxAcceleration.get()
            )
        );
        
        // Set tolerance for "at goal" detection (1 inch)
        profile.setTolerance(ClimbConstants.STATE_MARGIN_OF_ERROR);

        // Create ElevatorFeedforward for physics compensation
        feedforward = new ElevatorFeedforward(
            ClimbControl.kS.get(),  // Static friction
            ClimbControl.kG.get(),  // Gravity
            ClimbControl.kV.get(),  // Velocity
            ClimbControl.kA.get()   // Acceleration
        );
    }

    /**
     * Updates controller gains from NetworkTables without recreating objects.
     * More efficient than recreating controllers every loop.
     */
    private void refreshTunables() {
        // Update PID gains
        if (ClimbControl.kP.hasChanged() || 
            ClimbControl.kI.hasChanged() || 
            ClimbControl.kD.hasChanged()) {
            profile.setP(ClimbControl.kP.get());
            profile.setI(ClimbControl.kI.get());
            profile.setD(ClimbControl.kD.get());
        }

        // Update motion constraints if changed
        if (ClimbControl.maxVelocity.hasChanged() || 
            ClimbControl.maxAcceleration.hasChanged()) {
            profile.setConstraints(new Constraints(
                ClimbControl.maxVelocity.get(),
                ClimbControl.maxAcceleration.get()
            ));
        }

        // Feedforward gains (Note: WPILib's ElevatorFeedforward doesn't have setters in older versions)
        // If your version doesn't support these, recreate the feedforward object instead
        // For now, we'll check if values changed and recreate if needed
        if (ClimbControl.kS.hasChanged() || 
            ClimbControl.kG.hasChanged() || 
            ClimbControl.kV.hasChanged() || 
            ClimbControl.kA.hasChanged()) {
            feedforward = new ElevatorFeedforward(
                ClimbControl.kS.get(),
                ClimbControl.kG.get(),
                ClimbControl.kV.get(),
                ClimbControl.kA.get()
            );
        }
    }

    /**
     * Calculates the necessary voltage to reach the target height.
     * Combines PID feedback for error correction and Feedforward for physics compensation.
     */
    private void moveToGoal() {
        double goalHeight = state.heightMeters;
        double currentHeight = data.height.in(Meters);
        
        // Set the goal for the profiled controller
        profile.setGoal(goalHeight);
        
        // Calculate PID output (feedback control)
        double pidOutput = profile.calculate(currentHeight, goalHeight);
        
        // Get the profiled setpoint (what the profile says we should be doing right now)
        var setpoint = profile.getSetpoint();
        
        // Calculate feedforward (physics-based control)
        double ffOutput = feedforward.calculate(setpoint.velocity);
        
        // Combine feedback and feedforward
        appliedVoltage = pidOutput + ffOutput;
        
        // Track profile velocity for logging
        lastProfileVelocity = setpoint.velocity;
        
        // Send voltage to motors
        setVoltage(appliedVoltage);
    }

    /**
     * Logs subsystem data to AdvantageKit/AdvantageScope.
     * Data is organized into logical groups for easy visualization.
     */
    private void logData() {
        // === SUBSYSTEM STATE ===
        Logger.recordOutput("Climb/State/Current", state.name());
        Logger.recordOutput("Climb/State/GoalHeight", state.heightMeters);
        Logger.recordOutput("Climb/State/AtGoal", profile.atGoal());
        Logger.recordOutput("Climb/State/CurrentCommand",
            this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        
        // === POSITION TRACKING ===
        Logger.recordOutput("Climb/Position/Current", data.height);
        Logger.recordOutput("Climb/Position/CurrentMeters", data.height.in(Meters));
        Logger.recordOutput("Climb/Position/Error", profile.getPositionError());
        Logger.recordOutput("Climb/Position/ProfileSetpoint", profile.getSetpoint().position);
        
        // For AdvantageScope line graph (array format for easy plotting)
        Logger.recordOutput("Climb/Position/Graph", new double[] {
            data.height.in(Meters),           // Current position
            state.heightMeters,                // Goal position
            profile.getSetpoint().position     // Profile setpoint
        });
        
        // === VELOCITY TRACKING ===
        Logger.recordOutput("Climb/Velocity/Current", data.velocity);
        Logger.recordOutput("Climb/Velocity/CurrentMetersPerSec", data.velocity.in(MetersPerSecond));
        Logger.recordOutput("Climb/Velocity/Error", profile.getVelocityError());
        Logger.recordOutput("Climb/Velocity/ProfileSetpoint", lastProfileVelocity);
        Logger.recordOutput("Climb/Velocity/MaxAllowed", ClimbControl.maxVelocity.get());
        
        // For AdvantageScope line graph
        Logger.recordOutput("Climb/Velocity/Graph", new double[] {
            data.velocity.in(MetersPerSecond),  // Current velocity
            lastProfileVelocity,                 // Profile setpoint velocity
            ClimbControl.maxVelocity.get()      // Max velocity limit
        });
        
        // === ACCELERATION ===
        Logger.recordOutput("Climb/Acceleration/Current", data.accel);
        Logger.recordOutput("Climb/Acceleration/MaxAllowed", ClimbControl.maxAcceleration.get());
        
        // === MOTOR OUTPUTS (Left Motor) ===
        Logger.recordOutput("Climb/Motors/Left/Voltage", data.leftAppliedVolts);
        Logger.recordOutput("Climb/Motors/Left/Current", data.leftCurrentAmps);
        Logger.recordOutput("Climb/Motors/Left/Position", data.leftPositionMeters);
        Logger.recordOutput("Climb/Motors/Left/Velocity", data.leftVelocityMetersPerSec);
        Logger.recordOutput("Climb/Motors/Left/Temperature", data.leftTempCelsius);
        
        // === MOTOR OUTPUTS (Right Motor) ===
        Logger.recordOutput("Climb/Motors/Right/Voltage", data.rightAppliedVolts);
        Logger.recordOutput("Climb/Motors/Right/Current", data.rightCurrentAmps);
        Logger.recordOutput("Climb/Motors/Right/Position", data.rightPositionMeters);
        Logger.recordOutput("Climb/Motors/Right/Velocity", data.rightVelocityMetersPerSec);
        Logger.recordOutput("Climb/Motors/Right/Temperature", data.rightTempCelsius);
        
        // === MOTOR AVERAGES & COMPARISON ===
        Logger.recordOutput("Climb/Motors/Avg/Voltage", (data.leftAppliedVolts + data.rightAppliedVolts) / 2.0);
        Logger.recordOutput("Climb/Motors/Avg/Current", data.avgCurrentAmps);
        Logger.recordOutput("Climb/Motors/Avg/Position", data.avgPositionMeters);
        Logger.recordOutput("Climb/Motors/Avg/Velocity", data.avgVelocityMetersPerSec);
        
        // Motor comparison graph (for detecting synchronization issues)
        Logger.recordOutput("Climb/Motors/PositionComparison", new double[] {
            data.leftPositionMeters,
            data.rightPositionMeters
        });
        
        Logger.recordOutput("Climb/Motors/CurrentComparison", new double[] {
            data.leftCurrentAmps,
            data.rightCurrentAmps
        });
        
        // === SYNCHRONIZATION ===
        Logger.recordOutput("Climb/Sync/PositionDelta", data.positionDeltaMeters);
        Logger.recordOutput("Climb/Sync/Synchronized", 
            Math.abs(data.positionDeltaMeters) < ClimbConstants.STATE_MARGIN_OF_ERROR);
        Logger.recordOutput("Climb/Sync/MaxDeltaAllowed", ClimbConstants.STATE_MARGIN_OF_ERROR);
        
        // === CONTROL SIGNALS ===
        Logger.recordOutput("Climb/Control/CommandedVoltage", appliedVoltage);
        Logger.recordOutput("Climb/Control/PID/P", ClimbControl.kP.get());
        Logger.recordOutput("Climb/Control/PID/I", ClimbControl.kI.get());
        Logger.recordOutput("Climb/Control/PID/D", ClimbControl.kD.get());
        
        Logger.recordOutput("Climb/Control/Feedforward/kS", ClimbControl.kS.get());
        Logger.recordOutput("Climb/Control/Feedforward/kG", ClimbControl.kG.get());
        Logger.recordOutput("Climb/Control/Feedforward/kV", ClimbControl.kV.get());
        Logger.recordOutput("Climb/Control/Feedforward/kA", ClimbControl.kA.get());
        
        // === THEORETICAL CALCULATIONS (for tuning reference) ===
        Logger.recordOutput("Climb/Theoretical/MaxVelocity", 
            ClimbConstants.calculateTheoreticalMaxVelocity());
        Logger.recordOutput("Climb/Theoretical/MaxForce", 
            ClimbConstants.calculateTheoreticalMaxForce());
        Logger.recordOutput("Climb/Theoretical/VelocityUtilization",
            data.velocity.in(MetersPerSecond) / ClimbConstants.calculateTheoreticalMaxVelocity());
        
        // === HARDWARE SPECS (logged once for reference) ===
        Logger.recordOutput("Climb/Hardware/Gearing", ClimbConstants.GEARING);
        Logger.recordOutput("Climb/Hardware/DrumRadius", ClimbConstants.DRUM_RADIUS_METERS);
        Logger.recordOutput("Climb/Hardware/CarriageMass", ClimbConstants.CARRIAGE_MASS_KG);
        Logger.recordOutput("Climb/Hardware/NumMotors", ClimbConstants.NUM_MOTORS);
    }

    /**
     * Updates the 2D visual representation of the climber for AdvantageScope.
     * Shows current position, goal position, and safety limits.
     */
    public void updateMechanism() {
        double currentHeight = getHeight().in(Meters);
        double goalHeight = state.heightMeters;
        
        // Update main climb arm length
        climbArm.setLength(currentHeight);
        
        // Update goal indicator
        goalLine.setLength(goalHeight);
        
        // Color code based on state
        if (profile.atGoal()) {
            climbArm.setColor(new Color8Bit(Color.kGreen)); // At goal - green
        } else if (Math.abs(currentHeight - goalHeight) > 0.5) {
            climbArm.setColor(new Color8Bit(Color.kOrange)); // Far from goal - orange
        } else {
            climbArm.setColor(new Color8Bit(Color.kYellow)); // Near goal - yellow
        }
        
        // Update carriage position (at top of arm)
        // The carriage ligament is appended to the arm, so it automatically moves with it
        
        // Log the complete mechanism
        Logger.recordOutput("Climb/Mechanism", mech);
    }

    @Override
    public void periodic() {
        // 1. Update tunable parameters
        refreshTunables();
        
        // 2. Pull latest hardware data
        climbIO.updateData();

        // 3. Process data for logging (AdvantageKit)
        Logger.processInputs("Climb", data);
        
        // 4. Run control loop to reach target setpoint
        moveToGoal();
        
        // 5. Log values and update visualization
        logData();
        updateMechanism();
    }

    @Override
    public void simulationPeriodic() {
        // Simulation-specific logic can be added here if needed
    }

    // ========== Public Getters ==========

    /**
     * Gets the current height of the climber.
     */
    public Distance getHeight() {
        return data.height;
    }

    /**
     * Gets the current target state (STOW, CLIMB, etc.).
     */
    public ClimbStates getState() {
        return state;
    }

    /**
     * Gets the current height in meters (numeric).
     */
    public double getPosition() {
        return data.height.in(Meters);
    }

    /**
     * Checks if the climber is at the goal position.
     */
    public boolean atGoal() {
        return profile.atGoal();
    }

    /**
     * Gets the position error in meters.
     */
    public double getPositionError() {
        return profile.getPositionError();
    }

    // ========== Public Setters ==========

    /**
     * Directly sets the motor voltage through the IO layer.
     * Use with caution - prefer setState() for normal operation.
     */
    public void setVoltage(double volts) {
        climbIO.setVoltage(volts);
    }

    /**
     * Updates the target state of the climber.
     * The control loop will automatically move to the new target.
     */
    public void setState(ClimbStates newState) {
        if (newState != state) {
            Logger.recordOutput("Climb/StateChange", 
                String.format("%s -> %s", state.name(), newState.name()));
            state = newState;
        }
    }

    /**
     * Static setter for the height reference.
     * (Used for external tracking - consider if this is needed)
     */
    public static void setHeight(double height) {
        heightMeters = height;
    }

    /**
     * Commands the climber to stop moving immediately.
     * Sets voltage to 0 - motors will hold position if in brake mode.
     */
    public void stop() {
        climbIO.setVoltage(0);
    }

    /**
     * Resets the encoder to zero position.
     * Should be called when climber is at a known position (e.g., fully retracted).
     */
    public void resetEncoder() {
        climbIO.resetEncoder();
    }

    // ========== Command Factories ==========

    /**
     * Command to move to a specific climb state.
     */
    public Command moveToState(ClimbStates targetState) {
        return runOnce(() -> setState(targetState))
            .andThen(run(() -> {})) // Keep running periodic
            .until(this::atGoal)
            .withName("MoveToState_" + targetState.name());
    }

    /**
     * Command to stow the climber.
     */
    public Command stow() {
        return moveToState(ClimbStates.STOW);
    }

    /**
     * Command to extend climber for climbing.
     */
    public Command extend() {
        return moveToState(ClimbStates.CLIMB);
    }

    /**
     * Command to fully retract climber.
     */
    public Command retract() {
        return moveToState(ClimbStates.STOP);
    }

    /**
     * Command to manually control climb with voltage.
     * For testing/emergency use only.
     */
    public Command manualControl(double voltage) {
        return run(() -> setVoltage(voltage))
            .finallyDo(() -> stop())
            .withName("ManualControl_" + voltage + "V");
    }
}