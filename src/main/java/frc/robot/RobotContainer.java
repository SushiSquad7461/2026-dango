// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TeleopSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Swerve swerve = new Swerve();
        private final Intake intake;
        private final Hopper hopper;
        private final Shooter shooter = new Shooter();
        private final StateMachine stateMachine;
        private final AutoCommands autos;


        // private boolean wiggleOn;

        // Controller
        private final CommandXboxController driverController = new CommandXboxController(0);

        // Dashboard inputs
        // private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                if (Robot.isReal()) {
                        intake = new Intake();
                        hopper = new Hopper();
                        // swerve.resetGyro();

                } else {
                        intake = new Intake();
                        hopper = new Hopper();
                }
                this.stateMachine = new StateMachine(hopper,intake, shooter, swerve);

                this.autos = new AutoCommands(stateMachine, intake, swerve);

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
       private void configureButtonBindings() {
                swerve.setDefaultCommand(new TeleopSwerve(
                        swerve,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> driverController.back().getAsBoolean()));

                driverController.y().onTrue(Commands.runOnce(swerve::resetGyro));

                driverController.rightTrigger()
                        .onTrue(Commands.parallel(
                                stateMachine.changeState(RobotState.SHOOT_ONLY),
                                new AutoAlign(
                                        swerve,
                                        () -> -driverController.getLeftY(),
                                        () -> -driverController.getLeftX())))
                        .onFalse(stateMachine.changeState(RobotState.IDLE));

                driverController.rightBumper().onTrue(
                        Commands.either(
                        stateMachine.changeState(RobotState.IDLE),
                        stateMachine.changeState(RobotState.INTAKE_DOWN),
                        () -> stateMachine.getCurrentState() == RobotState.INTAKE_DOWN));

                driverController.leftBumper().onFalse(
                        Commands.either(
                        Commands.parallel(hopper.runHopper()),
                        Commands.parallel(hopper.stopHopper()),
                        () -> stateMachine.getCurrentState() == RobotState.SHOOT_ONLY ||
                                stateMachine.getCurrentState() == RobotState.INTAKE_DOWN_AND_SHOOT));

                driverController.povUp().onTrue(shooter.stepHood(5));
                driverController.povDown().onTrue(shooter.stepHood(-5));
        }

        public Command getAutonomousCommand() {
                return autos.getAuto();
        }

        public void resetModulesToAbsolute() {
                swerve.resetModulesToAbsolute();
        }

}
