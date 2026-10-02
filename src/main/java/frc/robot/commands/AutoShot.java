package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotLUT;
import frc.robot.subsystems.shooter.ShotTable;

public class AutoShot extends Command {

    // Hub centers in blue-origin field coordinates (meters), precomputed from 2026-rebuilt-andymark.json
    // as the midpoint of each hub's tag bounding box (blue tags 18-21, 24-27; red tags 2-5, 8-11).
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6115097, 4.0213534);
    private static final Translation2d RED_HUB_CENTER = new Translation2d(11.9015002, 4.0213534);

    private final Shooter shooter;
    
    private final ShotLUT lut = ShotTable.buildLUT();
    private final Swerve swerve;

    public AutoShot(Shooter shooter, Swerve swerve) {
        this.swerve = swerve;
        this.shooter = shooter;
        addRequirements(shooter);

    }

    // Looked up every loop, not at construction: the alliance is often unknown when robot code boots.
    private static Translation2d getHubCenter() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose(); // get robot pose
        Translation2d toHub = getHubCenter().minus(currentPose.getTranslation());
        double distance = toHub.getNorm();
        ShotLUT.ShotParameters shot = lut.get(distance);
        shooter.shoot(shot.rpm(), shot.angle());

        SmartDashboard.putNumber("AutoShot/DistanceM", distance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
