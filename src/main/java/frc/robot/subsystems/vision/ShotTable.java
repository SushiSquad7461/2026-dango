package frc.robot.subsystems.vision;

/**
 * Hand-tuned shooter lookup table. Add measured data points here — the system
 * interpolates RPM, hood angle, and time-of-flight for any distance in between.
 *
 * HOW TO TUNE:
 *   1. Stand a known distance from the hub (use a tape measure or odometry).
 *   2. Manually adjust RPM and hood angle until shots score consistently.
 *   3. Add or update the corresponding row in DATA below.
 *   4. Repeat for several distances spread across your full shooting range.
 *   5. Interpolation fills in everything between measured points automatically.
 *
 * DATA FORMAT — each row:
 *   { distanceMeters, flywheelRPM, hoodAngleDegrees, timeOfFlightSeconds }
 *
 * DISTANCE  — from the launcher to the hub center, in meters.
 * RPM       — flywheel RPM (ShooterIOKraken applies the gear ratio internally).
 * ANGLE     — hood angle in degrees. Must be within [hoodMinDegrees, hoodMaxDegrees].
 * TOF       — time-of-flight in seconds. Only affects shoot-on-the-move compensation.
 *             If you don't need SOTM, a rough estimate (distance / 10) is fine.
 *             Tune by watching how much ball drift you see while driving.
 *
 * TIPS:
 *   - More data points = smoother interpolation. Aim for one every 0.5–1.0 m.
 *   - Keep distances sorted ascending so the table is easy to read.
 *   - You need at least 2 rows for interpolation to work.
 */
public class ShotTable {

    // -------------------------------------------------------------------------
    // EDIT THESE ROWS with your measured values.
    // -------------------------------------------------------------------------
    private static final double[][] DATA = {
        // { distM,  rpm,   angleDeg, tof  }
        {    1.5,  3100,     12.5,  0.30 },
        {    2.0,  3400,     13.9,  0.40 },
        {    3,  4000,     19.0,  0.50 },
        {    4,  4500,     22.5,  0.60 },
        {    5,  5800,     25.0,  0.68 },
        // {    4.0,  3700,     20.5,  0.76 },
        // {    4.5,  3800,     21,  0.84 },
        // {    5.0,  3900,     21.5,  0.92 },
    };
    // -------------------------------------------- -----------------------------

    /** Build a ShotLUT from the data points above. Called once at startup. */
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
