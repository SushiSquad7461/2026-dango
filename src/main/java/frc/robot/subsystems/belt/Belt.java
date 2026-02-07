package frc.robot.subsystems.belt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Belt extends SubsystemBase {

    BeltIO io;
  public Belt() {
    if(Robot.isReal()) {
      io = new BeltReal();
    } else {
      io = new BeltSim();
    }
  }

  public Command switchState(BeltState newState) {
    return runOnce(()->{

    });
  }
  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}