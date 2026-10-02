package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotLUT;
import frc.robot.subsystems.shooter.ShotTable;

public class AutoShot extends WrapperCommand {

    // Hub centers in blue-origin field coordinates (meters), precomputed from 2026-rebuilt-andymark.json
    // as the midpoint of each hub's tag bounding box (blue tags 18-21, 24-27; red tags 2-5, 8-11).
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6115097, 4.0213534);
    private static final Translation2d RED_HUB_CENTER = new Translation2d(11.9015002, 4.0213534);

    private static final ShotLUT lut = ShotTable.buildLUT();

    public AutoShot(Shooter shooter, Swerve swerve) {
        super(shooter.shoot(() -> {
            double distance = getHubCenter().getDistance(swerve.getPose().getTranslation());
            SmartDashboard.putNumber("AutoShot/DistanceM", distance);
            return lut.get(distance);
        }));
    }

    // Looked up every loop, not at construction: the alliance is often unknown when robot code boots.
    private static Translation2d getHubCenter() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    }
}
