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
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;


public class StateMachine extends SubsystemBase {
    public enum RobotState {


        IDLE(IntakeState.IDLE,ShooterState.IDLE,HopperState.IDLE),
        SHOOT_ONLY(IntakeState.IDLE,ShooterState.PRESHOOT,HopperState.RUNNING),
        WIGGLING(IntakeState.WIGGLING,ShooterState.IDLE,HopperState.IDLE),
        INTAKE_DOWN(IntakeState.DEPLOYED, ShooterState.IDLE,HopperState.IDLE),
        //INTAKE_DOWN_SHOOT(IntakeState.DEPLOYED, ShooterState.SHOOT,HopperState.RUNNING),
        //INTAKE_ROLL_IN(IntakeState.ROLLERS_IN,ShooterState.IDLE,HopperState.IDLE),
        INTAKE_DOWN_AND_SHOOT(IntakeState.DEPLOYED,ShooterState.PRESHOOT,HopperState.RUNNING);
        //INTAKE_ROLL_OUT(IntakeState.ROLLERS_OUT,ShooterState.IDLE,HopperState.IDLE);
        //INTAKE_ROLL_OUT_AND_SHOOT(IntakeState.ROLLERS_OUT,ShooterState.SHOOT,HopperState.RUNNING),
        //INTAKE_WIGGLE_AND_SHOOT(IntakeState.WIGGLING,ShooterState.SHOOT,HopperState.RUNNING);

    
        public final IntakeState intakeState;
        public final ShooterState shooterState;
        public final HopperState hopperState;

        private RobotState(IntakeState intakeState, ShooterState shooterState,HopperState hopperState) {
            this.intakeState = intakeState;
            this.shooterState = shooterState;
            this.hopperState = hopperState;
        }
    }

    private RobotState state;
    private final Intake intake;
    private final ShooterSubsystem shooter;
    private final Hopper hopper;
    private final NetworkTable stateTable;
    private final StringPublisher currentStatePub;

    /**
     * Constructs the State Machine
     */
    public StateMachine(Intake intake, ShooterSubsystem shooter, Hopper hopper) {
        this.intake = intake;
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
                Commands.sequence(
                    //If the new state's change is the "wiggle" state
                    newState.intakeState == IntakeState.WIGGLING ?
                    //Depoly the intake
                    intake.changeState(IntakeState.DEPLOYED)
                        //Wait until the intake is in position
                        .andThen(Commands.waitUntil(intake::intakeAtTargetPos))
                        //Only then chagne the sate to wiggling
                        .andThen(intake.changeState(IntakeState.WIGGLING))
                        //Waits until the intake is at the wiggling height
                        .andThen(Commands.waitUntil(intake::intakeAtTargetPos))
                        //Slams the intake back down
                        .andThen(intake.changeState(IntakeState.DEPLOYED))
                        //ensures that the intake is in its deployed position before another command is scheduled
                        .andThen(Commands.waitUntil(intake::intakeAtTargetPos))
                    //If the new state isn't wiggle, act normally
                    : intake.changeState(newState.intakeState)
                ),
                shooter.changeState(newState.shooterState))
                .andThen(newState.shooterState != ShooterState.IDLE
                    ? Commands.waitUntil(() -> shooter.isShooterReady())
                    : Commands.none())
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