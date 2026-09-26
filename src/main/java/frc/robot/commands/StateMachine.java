package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;


public class StateMachine extends SubsystemBase {
    public enum RobotState {


        IDLE(HopperState.IDLE,IntakeState.DEPLOYED),
        SHOOT_ONLY(HopperState.RUNNING, IntakeState.DEPLOYED),//IntakeState.DEPLOYED
        INTAKE_DOWN(HopperState.IDLE,IntakeState.DEPLOYED),
        //WIGGLING(IntakeState.WIGGLING,ShooterState.IDLE,HopperState.IDLE),
        //INTAKE_DOWN_SHOOT(IntakeState.DEPLOYED, ShooterState.SHOOT,HopperState.RUNNING),
        //INTAKE_ROLL_IN(IntakeState.ROLLERS_IN,ShooterState.IDLE,HopperState.IDLE),
        INTAKE_DOWN_AND_SHOOT(HopperState.RUNNING, IntakeState.DEPLOYED),
        PASSING(HopperState.RUNNING, IntakeState.DEPLOYED);
        //INTAKE_ROLL_OUT(IntakeState.ROLLERS_OUT,ShooterState.IDLE,HopperState.IDLE);
        //INTAKE_ROLL_OUT_AND_SHOOT(IntakeState.ROLLERS_OUT,ShooterState.SHOOT,HopperState.RUNNING),
        //INTAKE_WIGGLE_AND_SHOOT(IntakeState.WIGGLING,ShooterState.SHOOT,HopperState.RUNNING);


        public final HopperState hopperState;
        public final IntakeState intakeState;

        private RobotState(HopperState hopperState,IntakeState intakeState) {
            this.hopperState = hopperState;
            this.intakeState = intakeState;
        }
    }

    private RobotState state;
    private final Hopper hopper;
    private final Intake intake;
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;
    private final Shooter shooter;

    /**
     * Constructs the State Machine
     */
    public StateMachine(Hopper hopper, Intake intake, Shooter shooter) {
        this.hopper = hopper;
        this.intake = intake;
        this.state = RobotState.IDLE;
        this.shooter = shooter;


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
        boolean shoot = newState == RobotState.SHOOT_ONLY;
        if(shoot) {
            return Commands.sequence(
            Commands.runOnce(() ->
            state = newState
            ),
            Commands.parallel(
                hopper.changeState(newState.hopperState),
                Commands.runOnce(() -> intake.setWantedState(newState.intakeState))),
                Commands.run(() -> shooter.shoot(3000, 25))
            );
        }
        return Commands.sequence(
            Commands.runOnce(() ->
            state = newState
            ),
            Commands.parallel(
                hopper.changeState(newState.hopperState),
                Commands.runOnce(() -> intake.setWantedState(newState.intakeState))),
                Commands.runOnce(() -> shooter.stop())
        );
    }

    public RobotState getCurrentState() {
        return state;
    }

    private void publishStates() {
        currentStatePub.set(state.toString());
    }
 }
