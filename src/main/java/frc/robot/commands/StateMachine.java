package frc.robot.commands;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;


public class StateMachine extends SubsystemBase {
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;

    public enum RobotState {


        IDLE(IntakeState.IDLE,ShooterState.IDLE),
        SHOOT_ONLY(IntakeState.IDLE,ShooterState.SHOOT),
        INTAKE_DOWN(IntakeState.DEPLOYED, ShooterState.IDLE),
        INTAKE_DOWN_SHOOT(IntakeState.DEPLOYED, ShooterState.SHOOT),
        INTAKE_ROLL_IN(IntakeState.ROLLERS_IN,ShooterState.IDLE),
        INTAKE_ROLL_IN_AND_SHOOT(IntakeState.ROLLERS_IN,ShooterState.SHOOT),
        INTAKE_ROLL_OUT(IntakeState.ROLLERS_OUT,ShooterState.IDLE),
        INTAKE_ROLL_OUT_AND_SHOOT(IntakeState.ROLLERS_OUT,ShooterState.SHOOT),
        INTAKE_WIGGLE(IntakeState.WIGGLING,ShooterState.IDLE),
        INTAKE_WIGGLE_AND_SHOOT(IntakeState.WIGGLING,ShooterState.SHOOT);


        public final IntakeState intakeState;
        public final ShooterState shooterState;

        private RobotState(IntakeState intakeState, ShooterState shooterState) {
            this.intakeState = intakeState;
            this.shooterState = shooterState;
        }
    }

    private RobotState state;
    private final Intake intake;
    private final ShooterSubsystem shooter;

    /**
     * Constructs the State Machine
     */
    public StateMachine(Intake intake, ShooterSubsystem shooter) {
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
        if(intake.intakeAtTargetPos()){
            return Commands.sequence(
                Commands.runOnce(()->{
                    this.state = newState;
                }),
                Commands.parallel(
                        shooter.changeState(newState.shooterState),
                        intake.changeState(newState.intakeState)
                    )
            );
        } else{
            return Commands.sequence(
                Commands.runOnce(()->{
                    this.state = newState;
                }),
                Commands.parallel(
                        shooter.changeState(newState.shooterState)
                    )
            );
        }
    }

    public RobotState getCurrentState() {
        return state;
    }

    // private void publishStates() {
    //     currentStatePub.set(state.toString());
    // }
}