package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;


public class StateMachine extends SubsystemBase {
    public enum RobotState {


        IDLE(ShooterState.IDLE,HopperState.IDLE),
        SHOOT_ONLY(ShooterState.SHOOT,HopperState.RUNNING),
        //WIGGLING(IntakeState.WIGGLING,ShooterState.IDLE,HopperState.IDLE),
        //INTAKE_DOWN_SHOOT(IntakeState.DEPLOYED, ShooterState.SHOOT,HopperState.RUNNING),
        //INTAKE_ROLL_IN(IntakeState.ROLLERS_IN,ShooterState.IDLE,HopperState.IDLE),
        INTAKE_DOWN_AND_SHOOT(ShooterState.SHOOT,HopperState.RUNNING);
        //INTAKE_ROLL_OUT(IntakeState.ROLLERS_OUT,ShooterState.IDLE,HopperState.IDLE);
        //INTAKE_ROLL_OUT_AND_SHOOT(IntakeState.ROLLERS_OUT,ShooterState.SHOOT,HopperState.RUNNING),
        //INTAKE_WIGGLE_AND_SHOOT(IntakeState.WIGGLING,ShooterState.SHOOT,HopperState.RUNNING);
        //AUTO_ALIGN()
    
        public final ShooterState shooterState;
        public final HopperState hopperState;

        private RobotState( ShooterState shooterState,HopperState hopperState) {
            this.shooterState = shooterState;
            this.hopperState = hopperState;


        }
    }

    private RobotState state;
    private final ShooterSubsystem shooter;
    private final Hopper hopper;
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;

    /**
     * Constructs the State Machine
     */
    public StateMachine(ShooterSubsystem shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.state = RobotState.IDLE;

       
        this.stateTable = NetworkTableInstance.getDefault().getTable("StateMachine");
         this.currentStatePub = stateTable.getStringTopic("CurrentState").publish();
    }

    /**
     * Publishes the state of each subsystem
     */
    @Override
    public void periodic() {
        publishStates();
    }

    public void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    public RobotState getState(){
        return this.state;
    }

    //TODO: Combine
    public Command changeState(RobotState newState) {
        
        return Commands.sequence(
            Commands.runOnce(() -> 
            state = newState
            ),
            Commands.parallel(
                shooter.changeState(newState.shooterState))
                .andThen(Commands.waitSeconds(1))
                .andThen(hopper.changeState(newState.hopperState))
        );
    }

    public RobotState getCurrentState() {
        return state;
    }

      private void publishStates() {
         currentStatePub.set(state.toString());
      }
 }