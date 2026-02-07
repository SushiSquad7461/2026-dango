package frc.robot.subsystems.belt;

public enum BeltState {
    //TODO: Set the speed for running
    RUNNING(0.0),
    STOPPED(0.0);

    private double speed;

    private BeltState(double speed){
        this.speed = speed;
    }
}
