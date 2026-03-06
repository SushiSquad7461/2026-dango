package frc.robot.subsystems.hopper;

import frc.robot.subsystems.hopper.Hopper.HopperState;

public class HopperIOSim implements HopperIO{
    public class HopperData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    @Override
    public void changeState(HopperState newState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'changeState'");
    }
    @Override
    public void setSpeed(double speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
    }
    
}
