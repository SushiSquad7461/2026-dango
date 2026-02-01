// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.config.ClimbConfig;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Climb.ClimbDataAutoLogged;
import frc.robot.subsystems.Climb.ClimbIO;
import frc.robot.utils.MiscUtils;

/**
 * Physics simulation implementation for the Climb subsystem.
 * Uses WPILib's ElevatorSim to model movement and current draw.
 */
public class ClimbSimulation implements ClimbIO {

  private ElevatorSim climbSim = new ElevatorSim(
      DCMotor.getNEO(2),
      ClimbConfig.ClimbSpecs.GEARING,
      ClimbConfig.ClimbSpecs.CARRIAGE_MASS.in(Kilograms),
      ClimbConfig.ClimbSpecs.DRUM_RADIUS.in(Meters),
      ClimbConfig.ClimbSpecs.MIN_HEIGHT.in(Meters),
      ClimbConfig.ClimbSpecs.MAX_HEIGHT.in(Meters),
      true,
      ClimbConfig.ClimbSpecs.STARTING_HEIGHT.in(Meters)
  );

  private final ClimbDataAutoLogged data;

  /**
   * @param climbdata The Logged Data object to be updated by this simulation.
   */
  public ClimbSimulation(ClimbDataAutoLogged climbdata) {
    this.data = climbdata;
  }

  @Override
  public void setVoltage(double volts) {
    double clampedVolts = MiscUtils.voltageClamp(volts);

    // Update the data object immediately with the applied voltage
    data.leftAppliedVolts = clampedVolts;
    data.rightAppliedVolts = clampedVolts;

    climbSim.setInputVoltage(clampedVolts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Brake mode is typically ignored in simple elevator simulations
  }

  @Override
  public void updateData() {
    // Step the physics simulation forward
    climbSim.update(RobotConfig.General.NOMINAL_LOOP_TIME_S);

    // Update height with the mount offset (physical position on the robot)
    data.height = Meters.of(climbSim.getPositionMeters() + 
                  ClimbConfig.ClimbSpecs.MOUNT_OFFSET.getY());
    
    data.velocity = MetersPerSecond.of(climbSim.getVelocityMetersPerSecond());

    // Simulation current draw (useful for checking battery strain/PDH limits)
    data.leftCurrentAmps = climbSim.getCurrentDrawAmps();
    data.rightCurrentAmps = climbSim.getCurrentDrawAmps();

    // Acceleration isn't natively provided by ElevatorSim without manual calculation
    data.accel = MetersPerSecondPerSecond.of(0);
  }
}