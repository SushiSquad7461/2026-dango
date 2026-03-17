package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    public Hopper(HopperIO io){
        this.io = io;
    }
    public boolean isRunning() {
        // TODO Auto-generated method stub
        return true;
    }
    public Command changeState(HopperState newState){
        return runOnce(()->{io.changeState(newState);});
    }
    public Command runHopper(){
        return runOnce(()->{io.setSpeed(-0.75);});
    }
    public Command runHopperBack(){
        return runOnce(()->{io.setSpeed(0.75);});
    }
    public Command stopHopper(){
        return runOnce(()->{io.setSpeed(0);});
    }
}
