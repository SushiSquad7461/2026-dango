package frc.robot.subsystems.hopper;

import frc.robot.subsystems.hopper.Hopper.HopperState;

public class HopperIOSim implements HopperIO{
    public class HopperData{
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    // Matches HopperIOReal supply current limit.
    private static final double HOPPER_SUPPLY_CURRENT_LIMIT_AMPS = 70.0;

    public final HopperData data = new HopperData();
    @Override
    public void changeState(HopperState newState) {
        setSpeed(newState.speed);
    }
    @Override
    public void setSpeed(double speed) {
        data.appliedVolts = speed * 12.0;
        data.currentAmps = Math.abs(speed) * HOPPER_SUPPLY_CURRENT_LIMIT_AMPS;
    }
    
}
