package frc.robot.subsystems.hopper;

import frc.robot.subsystems.hopper.Hopper.HopperState;

public class HopperIOSim implements HopperIO{
    public class HopperData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    private double appliedOutput = 0.0;

    @Override
    public void changeState(HopperState newState) {
        appliedOutput = newState.speed;
    }

    @Override
    public void setSpeed(double speed) {
        appliedOutput = speed;
    }
}