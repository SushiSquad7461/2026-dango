package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperVisualizer extends SubsystemBase {

    private static final Translation3d HOOD_JOINT = new Translation3d(0.0, 0.0, 0.0);
    private static final Translation3d INTAKE_JOINT = new Translation3d(0.0, 0.0, 0.0);
    private static final Translation3d HOPPER_PIVOT_JOINT = new Translation3d(0.0, 0.0, 0.0);

    private static final double HOPPER_PIVOT_REST_DEG = 0.0;
    private static final double INTAKE_CONTACT_DEG = 45.0;
    private static final double HOPPER_PIVOT_FOLLOW_RATIO = 1.0;
    private static final double HOPPER_PIVOT_MIN_DEG = -60.0;
    private static final double HOPPER_PIVOT_MAX_DEG = 0.0;

    private final DoubleSupplier hoodDegrees;
    private final DoubleSupplier intakeDegrees;

    private final StructArrayPublisher<Pose3d> publisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Components", Pose3d.struct).publish();

    public HopperVisualizer(DoubleSupplier hoodDegrees, DoubleSupplier intakeDegrees) {
        this.hoodDegrees = hoodDegrees;
        this.intakeDegrees = intakeDegrees;
    }

    private double hopperPivotDegreesFromIntake(double intakeDeg) {
        if (intakeDeg <= INTAKE_CONTACT_DEG) {
            return HOPPER_PIVOT_REST_DEG;
        }
        double pushed = HOPPER_PIVOT_REST_DEG - (intakeDeg - INTAKE_CONTACT_DEG) * HOPPER_PIVOT_FOLLOW_RATIO;
        return MathUtil.clamp(pushed, HOPPER_PIVOT_MIN_DEG, HOPPER_PIVOT_MAX_DEG);
    }

    @Override
    public void periodic() {
        double intakeDeg = intakeDegrees.getAsDouble();

        Pose3d hood = new Pose3d(
                HOOD_JOINT,
                new Rotation3d(0.0, Math.toRadians(hoodDegrees.getAsDouble()), 0.0));

        Pose3d intake = new Pose3d(
                INTAKE_JOINT,
                new Rotation3d(0.0, Math.toRadians(intakeDeg), 0.0));

        Pose3d hopperPivot = new Pose3d(
                HOPPER_PIVOT_JOINT,
                new Rotation3d(0.0, Math.toRadians(hopperPivotDegreesFromIntake(intakeDeg)), 0.0));

        publisher.set(new Pose3d[] { hood, intake, hopperPivot });
    }
}
