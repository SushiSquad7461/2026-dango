package frc.robot.subsystems.shooter;

/**
 * Hand-tuned shooter lookup table. Add measured data points here — the system
 * interpolates RPM and hood angle for any distance in between.
 *
 * HOW TO TUNE:
 *   1. Stand a known distance from the hub (use a tape measure or odometry).
 *   2. Manually adjust RPM and hood angle until shots score consistently.
 *   3. Add or update the corresponding row in DATA below.
 *   4. Repeat for several distances spread across your full shooting range.
 *   5. Interpolation fills in everything between measured points automatically.
 *
 * DATA FORMAT — each row:
 *   { distanceMeters, flywheelRPM, hoodAngleDegrees }
 *
 * DISTANCE  — from the robot center to the hub center, in meters.
 * RPM       — sent straight to the shooter motor; no gear ratio is applied.
 * ANGLE     — hood angle in degrees. Must be within [hoodMinDegrees, hoodMaxDegrees].
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
        // { distM,  rpm,   angleDeg }
        {    1.5,  3100,     12.5 },
        {    2.0,  3400,     13.9 },
        {    3,  4000,     19.0 },
        {    4,  4500,     22.5 },
        {    5,  5800,     25.0 },
    };
    // -------------------------------------------------------------------------

    /** Build a ShotLUT from the data points above. Called once at startup. */
    public static ShotLUT buildLUT() {
        ShotLUT lut = new ShotLUT();
        for (double[] row : DATA) {
            lut.put(row[0], new ShotLUT.ShotParameters(row[1], row[2]));
        }
        return lut;
    }
}
