package frc.robot.util;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public final class AllianceUtil {
    // Hub centers in blue-origin field coordinates (meters), precomputed from 2026-rebuilt-andymark.json
    // as the midpoint of each hub's tag bounding box (blue tags 18-21, 24-27; red tags 2-5, 8-11).
    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6115097, 4.0213534);
    private static final Translation2d RED_HUB_CENTER = new Translation2d(11.9015002, 4.0213534);

    private AllianceUtil() {}
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    // Call every loop, not at construction: the alliance is often unknown when robot code boots.
    public static Translation2d getHubCenter() {
        return isRedAlliance() ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    }
}
