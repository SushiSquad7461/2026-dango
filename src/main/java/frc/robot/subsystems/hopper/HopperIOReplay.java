package frc.robot.subsystems.hopper;

import frc.robot.subsystems.hopper.Hopper.HopperState;

public class HopperIOReplay implements HopperIO {
    private double speed = 0.0;

    @Override
    public void changeState(HopperState newState) {
        speed = newState.speed;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
