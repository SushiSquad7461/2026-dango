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
import frc.robot.config.ClimbConfig.ClimbControl;
import frc.robot.config.ClimbConfig.ClimbSpecs;
import frc.robot.config.ClimbConfig.ClimbStates;
// import frc.robot.subsystems.Climb.real.ClimbReal; // Added Import
import frc.robot.subsystems.Climb.sim.ClimbSimulation; // Added Import
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.MiscUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Subsystem for controlling the Robot Climb mechanism.
 * Uses AdvantageKit for logging and simulation-ready IO abstraction.
 */
public class Climb extends SubsystemBase {

 
  // Hardware Interface
  private ClimbIO climbIO;
  private final ClimbDataAutoLogged data = new ClimbDataAutoLogged();

  // Subsystem State
  private ClimbStates state = ClimbStates.STOW;

  // Control Constants from Config
  private static final MiscUtils.ControlConfig config = ClimbControl.CONTROL_CONFIG.get();

  /** * Profiled PID Controller: Handles smooth motion by following a trapezoidal velocity profile.
   * This prevents the climber from jerking by limiting max velocity and acceleration.
   * 
   */
    
  // Tunables
  // public static LoggedNetworkNumber kG;
  // public static LoggedNetworkNumber kP;
  // public static LoggedNetworkNumber kI;
  // public static LoggedNetworkNumber kD;
  // public static LoggedNetworkNumber kS;
  // public static LoggedNetworkNumber kV;
  // public static LoggedNetworkNumber kA;
  // public static LoggedNetworkNumber maxVelocity;
  // public static LoggedNetworkNumber maxAcceleration;

  public static LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/Climb/kG", config.kG);
  public static LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Climb/kP", config.kP);
  public static LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Climb/kI", config.kI);
  public static LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Climb/kD", config.kD);
  public static LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Climb/kS", config.kS);
  public static LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Climb/kV", config.kV);
  public static LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Climb/kA", config.kA); // 1.72
  public static LoggedNetworkNumber maxVelocity = new LoggedNetworkNumber("/Tuning/Climb/max velocity", 1.415);
  public static LoggedNetworkNumber maxAcceleration = new LoggedNetworkNumber( "/Climb/max acceleration", 4.1);

  public Climb() {

    // kG = new LoggedNetworkNumber("/Tuning/Climb/kG", config.kG);
    // kP = new LoggedNetworkNumber("/Tuning/Climb/kP", 12);
    // kI = new LoggedNetworkNumber("/Tuning/Climb/kI", 0);
    // kD = new LoggedNetworkNumber("/Tuning/Climb/kD", 0);
    // kS = new LoggedNetworkNumber("/Tuning/Climb/kS", 0.16);
    // kV = new LoggedNetworkNumber("/Tuning/Climb/kV", 7.77);
    // kA = new LoggedNetworkNumber("/Tuning/Climb/kA", 0.27); // 1.72
    // maxVelocity = new LoggedNetworkNumber("/Tuning/Climb/max velocity",
    //         1.415);
    // maxAcceleration = new LoggedNetworkNumber(
    //         "/Climb/max acceleration",
    //         4.1);
    // Conditional initialization based on environment
    // if (RobotBase.isReal()) {
    //     climbIO = new ClimbReal(data);
    // } else {
    //     climbIO = new ClimbSimulation(data);
    // }

    climbIO = new ClimbSimulation(data);
    // Sync the profile with the current height to prevent jump on startup
    profile.reset(data.height.in(Meters));
  }
  


  private final ProfiledPIDController profile = new ProfiledPIDController(
      kP.get(), kI.get(), kD.get(),
      new Constraints(
          ClimbControl.MAX_VELOCITY.in(MetersPerSecond),
          ClimbControl.MAX_ACCEL.in(MetersPerSecondPerSecond)));

  /**
   * Feedforward Controller: Calculates the voltage needed to overcome gravity (kG)
   * and friction/inertia (kS, kV, kA) to keep the elevator at a setpoint.
   */
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      kS.get(), kG.get(), kV.get(), kA.get());

  // AdvantageKit Mechanism2d Visualization
  private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d root = mech.getRoot("ClimbRoot", 1, 0);
  private final LoggedMechanismLigament2d ligament = root.append(
      new LoggedMechanismLigament2d("Climb", ClimbSpecs.STARTING_HEIGHT.in(Meters), 90));

  // Static tracking for global height reference
  public static double heightMeters = 0.0;
  
  // Debug tracking
  private double appliedVoltage;

  /** Initializes the Climb subsystem, switching between Real and Sim IO. */
  

  /** Gets the current height of the climber. */
  public Distance getHeight() {
    return data.height;
  }

  /** Gets the current target state (STOW, EXTEND, etc.). */
  public ClimbStates getState() {
    return state;
  }

  /** Gets the current height in meters (numeric). */
  public double getPosition() {
    return data.height.in(Meters);
  }

  /** Directly sets the motor voltage through the IO layer. */
  public void setVoltage(double volts) {
    climbIO.setVoltage(volts);
  }

  /** Updates the target state of the climber. */
  public void setState(ClimbStates newState) {
    state = newState;
  }

  /** Static setter for the height reference. */
  public static void setHeight(double height) {
    heightMeters = height;
  }

  /** Commands the climber to stop moving. */
  public void stop() {
    climbIO.setVoltage(0);
  }

  public void refreshTuneables(){
    profile.setP(kP.get());
    profile.setI(kI.get());
    profile.setD(kD.get());

    feedforward.setKa(kA.get());
    feedforward.setKg(kG.get());
    feedforward.setKs(kS.get());
    feedforward.setKv(kV.get());

  }

  /**
   * Calculates the necessary voltage to reach the target height defined by the current state.
   * Combines PID feedback for error correction and Feedforward for physical physics compensation.
   */
  private void moveToGoal() {
    profile.setGoal(state.height.in(Meters));
    
    // Calculate PID feedback based on current height
    double pidOutput = profile.calculate(data.height.in(Meters), state.height.in(Meters));
    
    // Calculate Feedforward based on the profiled setpoint (velocity and position)
    double ffOutput = feedforward.calculate(
        profile.getSetpoint().position,
        profile.getSetpoint().velocity);
    
    appliedVoltage = pidOutput + ffOutput;
    setVoltage(appliedVoltage);
  }
  
  /** Logs subsystem data to AdvantageKit/AdvantageScope. */
  private void logData() {
    
    Logger.recordOutput("Climb/HeightGoal", state.height.in(Meters));
    Logger.recordOutput("Climb/inputvoltage", appliedVoltage);
    Logger.recordOutput("Climb/currentCommand",
        this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());

    Logger.recordOutput("Climb/state", state.name());
    Logger.recordOutput("Climb/position", data.height);
    Logger.recordOutput("Climb/velocity", data.velocity);
    Logger.recordOutput("Climb/acceleration", data.accel);

    // Logs the average voltage between the two motors
    Logger.recordOutput("Climb/appliedVolts", ((data.leftAppliedVolts + data.rightAppliedVolts) / 2.0));
    Logger.recordOutput("Climb/leftAppliedVolts", data.leftAppliedVolts);
    Logger.recordOutput("Climb/rightAppliedVolts", data.rightAppliedVolts);
    Logger.recordOutput("Climb/leftCurrentAmps", data.leftCurrentAmps);
    Logger.recordOutput("Climb/rightCurrentAmps", data.rightCurrentAmps);
  }

  /** Updates the 2D visual representation of the climber. */
  public void updateMechanism() {
    ligament.setLength(getHeight().in(Meters));
    Logger.recordOutput("Climb/mechanism", mech);
  }

  @Override
  public void periodic() {

    refreshTuneables();
    // 1. Pull latest hardware data

    
    climbIO.updateData();

    // 2. Process data for logging (AdvantageKit)
    Logger.processInputs("Climb", data);
    
    // 3. Log values and update visualization
    logData();
    updateMechanism();

    //climbIO.setVoltage(kG.get());

    // 4. Run control loop to reach target setpoint
    moveToGoal();
  }

  /** Example command factory. */
  public Command exampleMethodCommand() {
    return runOnce(() -> { /* one-time action */ });
  }

  @Override
  public void simulationPeriodic() {
    // Logic specifically for simulation can be added here
  }
}
