package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtil {
    private AllianceUtil() {}
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }
}