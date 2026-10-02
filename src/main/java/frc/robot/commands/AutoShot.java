package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotLUT;
import frc.robot.subsystems.shooter.ShotTable;
import frc.robot.util.AllianceUtil;

public class AutoShot extends WrapperCommand {

    private static final ShotLUT lut = ShotTable.buildLUT();

    public AutoShot(Shooter shooter, Swerve swerve) {
        super(shooter.shoot(() -> aim(swerve).rpm(), () -> aim(swerve).angle()));
    }

    // LUT shot for the current distance to our hub.
    private static ShotLUT.ShotParameters aim(Swerve swerve) {
        double distance = AllianceUtil.getHubCenter().getDistance(swerve.getPose().getTranslation());
        SmartDashboard.putNumber("AutoShot/DistanceM", distance);
        return lut.get(distance);
    }
}
