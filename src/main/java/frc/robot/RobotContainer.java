// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TeleopSwerve;
import frc.robot.subsystems.vision.limelight_vision.Vision;

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
                vision = new Vision(swerve);

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
                this.stateMachine = new StateMachine(shooter, hopper,intake);

                this.autos = new AutoCommands(stateMachine, intake, shooter, swerve, vision);

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
                // // Default command, normal field-relative drive
                // swerve.setDefaultCommand(
                // DriveCommands.joystickDrive(
                // swerve,
                // () -> -driverController.getLeftY(),
                // () -> -driverController.getLeftX(),
                // () -> -driverController.getRightX()
                // )
                // );

                // // Lock to 0° when A button is held
                // driverController
                // .a()
                // .whileTrue(
                // DriveCommands.joystickDriveAtAngle(
                // swerve,
                // () -> -driverController.getLeftY(),
                // () -> -driverController.getLeftX(),
                // () -> Rotation2d.kZero
                // )
                // );

                // // Switch to X pattern when X button is pressed
                // driverController.x().onTrue(Commands.runOnce(swerve::stopWithX, swerve));

                // // Reset gyro to 0° when Y button is pressed
                // driverController.y().onTrue(Commands.runOnce(() ->swerve.setPose(
                // new Pose2d(swerve.getPose().getTranslation(),
                // Rotation2d.kZero)),swerve).ignoringDisable(true));

                swerve.setDefaultCommand(new TeleopSwerve(
                                swerve,
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX(),
                                () -> -driverController.getRightX(),
                                () -> driverController.back().getAsBoolean())); // allows you to drive as robot relative
                                                                                // only while holding down the button

                driverController.y().onTrue(Commands.runOnce(() -> swerve.resetGyro()));

                // Intake & Shooter
                // driverController.rightTrigger().and(driverController.rightBumper()).onTrue(
                //                 stateMachine.changeState(RobotState.SHOOT_ONLY));
                // driverController.rightTrigger().negate().and(driverController.rightBumper()).onTrue(
                //                 stateMachine.changeState(RobotState.IDLE));
                // driverController.rightBumper().negate().and(driverController.rightTrigger()).onTrue(
                //                 stateMachine.changeState(RobotState.SHOOT_ONLY));
                // driverController.rightBumper().negate().and(driverController.rightTrigger().negate()).onTrue(
                //                 stateMachine.changeState(RobotState.IDLE));

                driverController.rightTrigger().onTrue(stateMachine.changeState(RobotState.SHOOT_ONLY)).onFalse(stateMachine.changeState(RobotState.INTAKE_DOWN));
                driverController.rightBumper().onTrue(
                        Commands.either(
                                intake.changeState(IntakeState.IDLE),
                                intake.changeState(IntakeState.DEPLOYED),
                                () -> intake.getState() == IntakeState.DEPLOYED));

                 // Intake & Shooter
                // driverController.rightTrigger().and(driverController.rightBumper()).onTrue(
                //                 intakeDown?stateMachine.changeState(RobotState.SHOOT_ONLY)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)):
                //                 stateMachine.changeState(RobotState.INTAKE_DOWN_AND_SHOOT)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)));

                // driverController.rightTrigger().negate().and(driverController.rightBumper()).onTrue(
                //                  intakeDown?stateMachine.changeState(RobotState.IDLE)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)):
                //                 stateMachine.changeState(RobotState.INTAKE_DOWN)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)));

                // driverController.rightBumper().negate().and(driverController.rightTrigger()).onTrue(
                //                  intakeDown?stateMachine.changeState(RobotState.INTAKE_DOWN_AND_SHOOT)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)):
                //                 stateMachine.changeState(RobotState.SHOOT_ONLY)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)));

                // driverController.rightBumper().negate().and(driverController.rightTrigger().negate()).onTrue(
                //                  intakeDown?stateMachine.changeState(RobotState.INTAKE_DOWN)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)):
                //                 stateMachine.changeState(RobotState.IDLE)
                //                 .andThen(Commands.runOnce(()->intakeDown=!intakeDown)));

                driverController.leftBumper().onTrue(
                        Commands.parallel(shooter.runFeederBack(), hopper.runHopperBack())
                ).onFalse(
                        Commands.either(
                                Commands.parallel(shooter.runFeeder(), hopper.runHopper()),
                                Commands.parallel(shooter.stopFeeder(), hopper.stopHopper()),
                                () -> stateMachine.getCurrentState() == RobotState.SHOOT_ONLY ||
                                      stateMachine.getCurrentState() == RobotState.INTAKE_DOWN_AND_SHOOT));
                
                driverController.povDown().onTrue(Commands.runOnce(() -> {
                        hoodedShooter.moveHood(-0.05);
                })).onFalse(Commands.runOnce(() -> {
                        hoodedShooter.moveHood(0);}));
                driverController.povUp().onTrue(Commands.runOnce(() -> {
                        hoodedShooter.moveHood(0.05);
                }))
                                .onFalse(Commands.runOnce(() -> {
                                        hoodedShooter.moveHood(0);
                                }));
                ;

                driverController.leftTrigger().whileTrue(new AutoAlign(
                    swerve,
                    vision,
                    () -> DriverStation.getAlliance().isPresent() &&
                        DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ));
                
                // operatorController.a().onTrue(Commands.runOnce(() -> shooter.setTargetRPM("hub"), shooter));
                // operatorController.b().onTrue(Commands.runOnce(() -> shooter.setTargetRPM("default"), shooter));
                // operatorController.x().onTrue(Commands.runOnce(() -> shooter.setTargetRPM("outpost"), shooter));
                // operatorController.y().onTrue(Commands.runOnce(() -> shooter.setTargetRPM("trench"), shooter));

                // operatorController.a().or(operatorController.b()).or(operatorController.x()).or(operatorController.y())
                //         .onFalse(Commands.runOnce(() -> shooter.setTargetRPM("default"), shooter));
        }

        public Command getAutonomousCommand() {
                return autos.getAuto();
        }

        public void resetModulesToAbsolute() {
                swerve.resetModulesToAbsolute();
        }
}
