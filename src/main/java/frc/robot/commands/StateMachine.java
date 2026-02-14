package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;


public class StateMachine extends SubsystemBase {
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;

    public enum RobotState {


        IDLE(IntakeState.IDLE,ShooterState.IDLE);

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

    /**
     * Constructs the State Machine
     */
    public StateMachine(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        this.currentStatePub = null;
        this.stateTable = null;
    }

    /**
     * Publishes the state of each subsystem
     */
    @Override
    public void periodic() {
       // publishStates();
    }

    public void scheduleNewState(RobotState newState) {
        changeState(newState).schedule();
    }

    //TODO: Combine
    public Command changeState(RobotState newState) {
        return Commands.sequence(
        Commands.runOnce(()->{
            this.state = newState;
        }),
        Commands.parallel(
                shooter.changeState(newState.shooterState),
                intake.changeState(newState.intakeState)
            )
        );
    }

    public RobotState getCurrentState() {
        return state;
    }

    // private void publishStates() {
    //     currentStatePub.set(state.toString());
    // }
}