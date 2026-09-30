package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;


public class StateMachine extends SubsystemBase {
    public enum RobotState {


        IDLE(IntakeState.IDLE),
        SHOOT_ONLY(IntakeState.DEPLOYED),
        INTAKE_DOWN(IntakeState.DEPLOYED),
        INTAKE_DOWN_AND_SHOOT(IntakeState.DEPLOYED),
        PASSING(IntakeState.DEPLOYED);

        public final IntakeState intakeState;

        private RobotState(IntakeState intakeState) {
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

    public Command changeState(RobotState newState) {
        boolean shoot = newState == RobotState.SHOOT_ONLY;

        if (shoot) {
            return Commands.sequence(
                Commands.runOnce(() -> state = newState),
                Commands.parallel(
                    intake.pivot(newState.intakeState.pivotAngle),
                    intake.setStateRollers(newState.intakeState.rollerSpeed),
                    shooter.shoot(3000, 25)
                )
            );
        }

        return Commands.sequence(
            Commands.runOnce(() -> state = newState),
            Commands.parallel(
                intake.pivot(newState.intakeState.pivotAngle),
                intake.setStateRollers(newState.intakeState.rollerSpeed)
            ),
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
