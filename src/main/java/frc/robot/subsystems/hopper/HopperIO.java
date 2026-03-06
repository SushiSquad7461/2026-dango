package frc.robot.subsystems.hopper;

import frc.robot.subsystems.hopper.Hopper.HopperState;

public interface HopperIO {
    void changeState(HopperState newState);
    void setSpeed(double speed);

}
