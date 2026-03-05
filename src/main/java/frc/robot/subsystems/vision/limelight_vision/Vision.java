package frc.robot.subsystems.vision.limelight_vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;

public class Vision extends SubsystemBase {

    private final NetworkTable leftTable;
    private final NetworkTable rightTable;

    private static final Set<Integer> BLUE_HUB_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    private static final Set<Integer> RED_HUB_IDS  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    public Vision() {
        leftTable  = NetworkTableInstance.getDefault().getTable("limelight-left");
        rightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
    }

    private boolean hasTarget(NetworkTable table) {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    private double getTx(NetworkTable table) {
        return table.getEntry("tx").getDouble(0);
    }

    private boolean isHubTag(NetworkTable table) {
        if (!hasTarget(table)) return false;
        int tagId = (int) table.getEntry("tid").getDouble(-1);
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        return alliance.get() == DriverStation.Alliance.Blue
            ? BLUE_HUB_IDS.contains(tagId)
            : RED_HUB_IDS.contains(tagId);
    }

    private NetworkTable getBestCamera() {
        boolean leftHas  = isHubTag(leftTable);
        boolean rightHas = isHubTag(rightTable);

        if (leftHas && rightHas) {
            return Math.abs(getTx(leftTable)) <= Math.abs(getTx(rightTable))
                ? leftTable : rightTable;
        }
        if (leftHas)  return leftTable;
        if (rightHas) return rightTable;
        return null;
    }

    private Pose3d parsePose(NetworkTable table) {
        double[] bp = table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if (bp.length < 6) return null;
        return new Pose3d(
            bp[0], bp[1], bp[2],
            new Rotation3d(
                Math.toRadians(bp[3]),
                Math.toRadians(bp[4]),
                Math.toRadians(bp[5])
            )
        );
    }

    private double getTimestamp(NetworkTable table) {
        double[] bp = table.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if (bp.length < 7) return -1;
        return Timer.getFPGATimestamp() - (bp[6] / 1000.0);
    }

    public boolean hasTarget() {
        return getBestCamera() != null;
    }

    public double getTx() {
        NetworkTable cam = getBestCamera();
        return cam != null ? getTx(cam) : 0;
    }

    public Pose3d getEstimatedGlobalPose() {
        NetworkTable cam = getBestCamera();
        return cam != null ? parsePose(cam) : null;
    }

    public double getTimestamp() {
        NetworkTable cam = getBestCamera();
        return cam != null ? getTimestamp(cam) : -1;
    }

    @Override
    public void periodic() {}
}