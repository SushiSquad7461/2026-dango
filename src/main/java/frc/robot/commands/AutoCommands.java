package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoCommands {
    private final SendableChooser<Command> autoChooser=new SendableChooser<Command>();;
    private final StringPublisher selectedAuto;
    private final NetworkTable autoNetworkTable;

    public AutoCommands(StateMachine stateMachine, Intake intake, ShooterSubsystem shooter, HoodedShooter hoodedShooter, Swerve swerve, Vision vision){
            autoNetworkTable = NetworkTableInstance.getDefault().getTable("Auto");
            selectedAuto = autoNetworkTable.getStringTopic("selectedAuto").publish();
            selectedAuto.set("Nothing");

            NamedCommands.registerCommand("AutoAlign",
                new AutoAlign(swerve, vision, shooter, hoodedShooter,
                    () -> 0.0, () -> 0.0, () -> 0.0));

             NamedCommands.registerCommand("Shoot",
            new InstantCommand(() -> stateMachine.scheduleNewState(RobotState.SHOOT_ONLY)));
        NamedCommands.registerCommand("Intake",
            new InstantCommand(() -> stateMachine.scheduleNewState(RobotState.INTAKE_DOWN_AND_SHOOT)));
        NamedCommands.registerCommand("Down",
            new InstantCommand(() -> stateMachine.scheduleNewState(RobotState.INTAKE_DOWN)));
        NamedCommands.registerCommand("Idle",
            new InstantCommand(() -> stateMachine.scheduleNewState(RobotState.IDLE)));
    
            autoChooser.setDefaultOption("Nothing", new InstantCommand());
            autoChooser.addOption("Test_Auto", new PathPlannerAuto("Test_Auto"));
            autoChooser.addOption("N_Shoot", new PathPlannerAuto("N_Shoot"));
            autoChooser.addOption("Ideal_N_Shoot", new PathPlannerAuto("Ideal_N_Shoot"));
            autoChooser.addOption("TestIdeal_N_Shoot", new PathPlannerAuto("TestIdeal_N_Shoot"));
            autoChooser.addOption("Shoot_Still", new PathPlannerAuto("Shoot_Still"));
            autoChooser.addOption("AutoAlign_Shoot", new PathPlannerAuto("AutoAlign_Shoot"));
            autoChooser.addOption("Shoot_N_Shoot", new PathPlannerAuto("Shoot_N_Shoot"));
            autoChooser.addOption("Shoot_Neutral_Trench", new PathPlannerAuto("Shoot_Neutral_Trench"));
            autoChooser.addOption("BestIdeal_N_Shoot_Extra", new PathPlannerAuto("BestIdeal_N_Shoot_Extra"));
            SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }
}
