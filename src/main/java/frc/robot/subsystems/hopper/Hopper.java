package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.Constants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase{
    public enum HopperState{
        IDLE(0.0),
        RUNNING(-0.75);

        public double speed; 

        private HopperState(double speed){
            this.speed = speed;
        }
    }
    private HopperIO io;
    private double commandedSpeed = 0.0;
    public Hopper(HopperIO io){
        this.io = io;
    }
    public boolean isRunning() {
        // TODO Auto-generated method stub
        return true;
    }
    public Command changeState(HopperState newState){
        return runOnce(()->{
            commandedSpeed = newState.speed;
            io.changeState(newState);
        });
    }
    public Command runHopper(){
        return runOnce(()->{
            commandedSpeed = -0.75;
            io.setSpeed(-0.75);
        });
    }
    public Command runHopperBack(){
        return runOnce(()->{
            commandedSpeed = 0.75;
            io.setSpeed(0.75);
        });
    }
    public Command stopHopper(){
        return runOnce(()->{
            commandedSpeed = 0.0;
            io.setSpeed(0);
        });
    }

    public double getSimulatedCurrentDrawAmps() {
        if (io instanceof HopperIOSim simIo) {
            return simIo.data.currentAmps;
        }
        return 0.0;
    }
    @Override
    public void periodic() {
        if (Constants.currentMode != Constants.Mode.REAL && io instanceof HopperIOSim simIo) {
            Logger.recordOutput("Hopper/CommandedSpeed", commandedSpeed);
            Logger.recordOutput("Hopper/AppliedVolts", simIo.data.appliedVolts);
            Logger.recordOutput("Hopper/CurrentAmps", simIo.data.currentAmps);
        }
    }
}
