package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/** LUT for a shooter with adjustable flywheel velocity (RPM), hood angle (degrees), and ToF (seconds). */
public class ShotLUT {
    public record ShotParameters(double rpm, double angle, double tof)
            implements Interpolatable<ShotParameters> {
        public ShotParameters interpolate(ShotParameters endValue, double t) {
            return new ShotParameters(
                MathUtil.interpolate(rpm(), endValue.rpm(), t),
                MathUtil.interpolate(angle(),    endValue.angle(),    t),
                MathUtil.interpolate(tof(),      endValue.tof(),      t));
        }
    }

    private final InterpolatingTreeMap<Double, ShotParameters> map =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters::interpolate);

    public void put(double distance, ShotParameters params) {
        map.put(distance, params);
    }

    public ShotParameters get(double distance) {
        return map.get(distance);
    }
}
