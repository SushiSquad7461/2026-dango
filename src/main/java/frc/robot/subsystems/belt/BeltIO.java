package frc.robot.subsystems.belt;

import org.littletonrobotics.junction.AutoLog;


public interface BeltIO{
  @AutoLog
  public class BeltData {
    //TODO: add data fields    
    public double beltVelocity = 0.0;
  }
  public void setBeltVelocity(double velocity);

}