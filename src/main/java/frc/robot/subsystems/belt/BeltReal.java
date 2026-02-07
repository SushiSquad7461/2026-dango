package frc.robot.subsystems.belt;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltReal implements BeltIO {
  /** Creates a new ExampleSubsystem. */
  private TalonFX beltTalon = new TalonFX(1);
  public BeltReal() {

  }

  public boolean exampleCondition() {
    return false;
  }
@Override
public void setBeltVelocity(double velocity) {
    // TODO Auto-generated method stub
    }
}
