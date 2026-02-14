package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class StateMachine extends SubsystemBase {
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;

    public enum RobotState {
        IDLE(IntakeState.IDLE),

        public final IntakeState intakeState;
        public final ShooterState shooterState;


        private RobotState(IntakeState intakeState, ShooterState shooterState) {
            this.intakeState = intakeState;
            this.shooterState = shooterState;
        }
    }

    private RobotState state;
    private final Intake intake;
    private final Shooter shooter;

    public StateMachine(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;

        // stateTable = NetworkTableInstance.getDefault().getTable("StateMachine");
        // currentStatePub = stateTable.getStringTopic("CurrentState").publish();
        // intakeStatePub = stateTable.getStringTopic("SubsystemStates/Intake").publish();
        // manipulatorStatePub = stateTable.getStringTopic("SubsystemStates/Manipulator").publish();
        // elevatorStatePub = stateTable.getStringTopic("SubsystemStates/Elevator").publish();
    }

    @Override
    public void periodic() {
        publishStates();
    }

    public void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    //TODO: Combine
    public Command changeState(RobotState newState) {
        return Commands.parallel(
                this.state = newState,
                shooter.changeState(newState.shooterState),
                intake.changeState(newState.intakeState)
            );

    }

    public RobotState getCurrentState() {
        return state;
    }

    private void publishStates() {
        currentStatePub.set(state.toString());
    }
}