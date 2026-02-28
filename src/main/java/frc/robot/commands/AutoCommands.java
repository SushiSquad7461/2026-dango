package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoCommands {
    private final SendableChooser<Command> autoChooser;
    private final StringPublisher selectedAuto;
    private final NetworkTable autoNetworkTable;

    public AutoCommands(StateMachine stateMachine, Intake intake, ShooterSubsystem shooter){
            autoChooser= new SendableChooser<Command>();
            autoNetworkTable = NetworkTableInstance.getDefault().getTable("Auto");
            selectedAuto = autoNetworkTable.getStringTopic("selectedAuto").publish();
            selectedAuto.set("Nothing");

            NamedCommands.registerCommand("Move_Ahead", stateMachine.changeState(null) );
    
            autoChooser.setDefaultOption("Nothing", new InstantCommand());
            autoChooser.addOption("a", new PathPlannerAuto("a"));

    }

    public Command getAuto() {
        return autoChooser.getSelected();
    }
}
