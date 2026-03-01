package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO{
  @Override
  public void setFlywheelRPM(double rpm) {
    // open-loop "good enough" for first iteration
  }

  @Override
  public void stopFlywheel() {
  }

  @Override
  public void runFeeder() { // open-loop "good enough" for first iteration
  }

  @Override
  public void stopFeeder() {
  }

  @Override
  public double getFlywheelRPM() {
    // TODO: implement checking flywheel RPM
    return 0;
  }

@Override
public double getHoodPos() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getHoodPos'");
}

@Override
public boolean isShooterReady() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'isShooterReady'");
}
}
