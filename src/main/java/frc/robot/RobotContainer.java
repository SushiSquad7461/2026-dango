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
import frc.robot.generated.Constants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.StateMachine;
import frc.robot.commands.StateMachine.RobotState;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.HoodedShooter;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
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
        private final ShooterSubsystem shooter;
        private final Hopper hopper;
        private final StateMachine stateMachine;
        private final HoodedShooter hoodedShooter;
        private final AutoCommands autos;
        //@SuppressWarnings("unused")
        private final Vision vision;


        // private boolean wiggleOn;

        // Controller
        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController operatorController = new CommandXboxController(1);

        // Dashboard inputs
        // private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                if (Robot.isReal()) {
                        shooter = new ShooterSubsystem(new ShooterIOKraken());
                        intake = new Intake(new IntakeReal());
                        hopper = new Hopper(new HopperIOReal());
                        // swerve.resetGyro();

                } else {
                        shooter = new ShooterSubsystem(new ShooterIOSim());
                        intake = new Intake(new IntakeSim());
                        hopper = new Hopper(new HopperIOSim());
                }
                hoodedShooter = new HoodedShooter();
                vision = new Vision(swerve);
                this.stateMachine = new StateMachine(shooter, hopper,intake);

                this.autos = new AutoCommands(stateMachine, intake, shooter, hoodedShooter, swerve, vision);

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
                                () -> driverController.back().getAsBoolean())); // allows you to drive as robot relative
                                                                                // only while holding down the button

                driverController.y().onTrue(Commands.runOnce(() -> {
                        // Compute target heading once — both resetGyro and seedIMU must use
                        // the same value.
                        double yaw = frc.robot.util.AllianceUtil.isRedAlliance() ? 180.0 : 0.0;
                        swerve.resetGyro();
                        vision.seedIMU(yaw);
                        vision.resetOffset();
                        vision.resetWarmStart();
                        // Trigger MT1 re-bootstrap so the next cycle hard-sets the full
                        // pose (X/Y + heading) from multi-tag geometry, instead of slowly
                        // converging through the Kalman filter.
                        vision.requestRebootstrap();
                }));

                driverController.rightTrigger().onTrue(stateMachine.changeState(RobotState.SHOOT_ONLY)).onFalse(stateMachine.changeState(RobotState.IDLE));
                driverController.rightBumper().onTrue(
                        Commands.either(
                                Commands.runOnce(()->intake.setWantedState(IntakeState.IDLE)),
                                Commands.runOnce(()->intake.setWantedState(IntakeState.DEPLOYED)),
                                () -> intake.getState() == IntakeState.DEPLOYED));

                driverController.leftBumper().onTrue(
                        Commands.parallel(
                        Commands.runOnce(() -> intake.setWantedState(IntakeState.WIGGLING)))
                ).onFalse(
                        Commands.either(
                                Commands.parallel(shooter.runFeeder(), hopper.runHopper()),
                                Commands.parallel(shooter.stopFeeder(), hopper.stopHopper()),
                                () -> stateMachine.getCurrentState() == RobotState.SHOOT_ONLY ||
                                      stateMachine.getCurrentState() == RobotState.INTAKE_DOWN_AND_SHOOT));

                // D-pad: step hood ±5° using MotionMagic position hold
                driverController.povDown().onTrue(Commands.runOnce(() -> hoodedShooter.stepHood(-Constants.HoodedShooterConstants.hoodStepDegrees), hoodedShooter));
                driverController.povUp().onTrue(  Commands.runOnce(() -> hoodedShooter.stepHood( Constants.HoodedShooterConstants.hoodStepDegrees), hoodedShooter));

                driverController.leftTrigger().whileTrue(new AutoAlign(
                        swerve, vision, shooter, hoodedShooter,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()
                ));

                // bind to copilot D-pad
                operatorController.povUp().onTrue(Commands.runOnce(() -> vision.adjustOffset(100.0)));
                operatorController.povDown().onTrue(Commands.runOnce(() -> vision.adjustOffset(-100.0)));
        }

        public Command getAutonomousCommand() {
                return autos.getAuto();
        }

        public void resetModulesToAbsolute() {
                swerve.resetModulesToAbsolute();
        }

        /** Re-bootstrap vision pose at auto→teleop transition. */
        public void onTeleopStart() {
                vision.requestRebootstrap();
                vision.resetWarmStart();
        }
}
