package frc.robot.subsystems.vision;

/**
 * Hand-tuned pass lookup table. Same pattern as ShotTable, but for lob passes
 * from the central zone into the alliance zone.
 *
 * HOW TO TUNE:
 *   1. Position robot at a known distance in the central zone.
 *   2. Aim toward the alliance zone through a clear side lane.
 *   3. Adjust RPM and hood angle until balls land in the target area.
 *   4. Add or update the corresponding row in DATA below.
 *
 * DATA FORMAT — each row:
 *   { distanceMeters, flywheelRPM, hoodAngleDegrees, timeOfFlightSeconds }
 *
 * TIPS:
 *   - Pass distances are typically 4-10m (longer than scoring shots).
 *   - Prefer higher hood angles (20-35 deg) for lob arcs that clear the 0.99m trench.
 *   - All seed values below are estimates — replace with field-tuned data.
 */
public class PassTable {
    private static final double[][] DATA = {
        // { distM,  rpm,   angleDeg, tof  }
        {    2.5,  2400,     33.0,  0.60 },
        {    3.0,  2600,     32.0,  0.65 },
        {    4.0,  3000,     30.0,  0.80 },
        {    5.0,  3200,     27.0,  0.90 },
        {    6.0,  3400,     24.0,  1.00 },
        {    7.0,  3600,     22.0,  1.10 },
        {    8.0,  3800,     20.0,  1.20 },
        {    9.0,  4000,     18.0,  1.30 },
        {   10.0,  4200,     16.0,  1.40 },
    };

    /** Build a ShotLUT from the pass data points above. Called once at startup. */
    public static ShotLUT buildLUT() {
        ShotLUT lut = new ShotLUT();
        for (double[] row : DATA) {
            double dist  = row[0];
            double rpm   = row[1];
            double angle = row[2];
            double tof   = row[3];
            lut.put(dist, new ShotLUT.ShotParameters(rpm, angle, tof));
        }
        return lut;
    }
}
