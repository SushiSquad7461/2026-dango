package frc.robot.subsystems.Climb;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LoggedTunableNumber;


public class ClimbConstants {
    public static final double gearing = 16 * (24.0 / 22.0);
    public static final double carriageMassKg = Units.lbsToKilograms(54);
    // 22 TOOTH, 1/4 in pitch, divide by 2pi to go from circumfrence to radius
    public static final double drumRadiusMeters = (Units.inchesToMeters(22.0 / 4.0) / (2 * Math.PI));
    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = Units.feetToMeters(6); // remeasure maxV and A
    public static final boolean simulateGravity = false;
    public static final double startingHeightMeters = 0;

    public static final double baseHeight = Units.feetToMeters(3.25);
    public static final int numMotors = 1;

    public static int motorIds = 0;
    public static int motorInverted = 0;
    public static int sprocketRadiusMeters = 1;

    public static int zeroOffset = 0;

    public static final double stateMarginOfError = 0.1;

    public static class ClimbControl {
        public static LoggedTunableNumber kG = new LoggedTunableNumber("/Tuning/Climb/kG", 0.32);
        public static LoggedTunableNumber kP = new LoggedTunableNumber("/Tuning/Climb/kP", 12);
        public static LoggedTunableNumber kI = new LoggedTunableNumber("/Tuning/Climb/kI", 0);
        public static LoggedTunableNumber kD = new LoggedTunableNumber("/Tuning/Climb/kD", 0);
        public static LoggedTunableNumber kS = new LoggedTunableNumber("/Tuning/Climb/kS", 0.16);
        public static LoggedTunableNumber kV = new LoggedTunableNumber("/Tuning/Climb/kV", 7.77);
        public static LoggedTunableNumber kA = new LoggedTunableNumber("/Tuning/Climb/kA", 0.27); // 1.72
        public static LoggedTunableNumber maxVelocity = new LoggedTunableNumber("/Tuning/Climb/max velocity",
                1.415);
        public static LoggedTunableNumber maxAcceleration = new LoggedTunableNumber(
                "/Climb/max acceleration",
                4.1);
    }

    public enum ClimbStates {
        STOP(Units.inchesToMeters(0)), 
        CLIMB(Units.inchesToMeters(3)),
        MAX(Units.feetToMeters(6)),
        STOW(Units.inchesToMeters(.75));
        public double heightMeters;

        private ClimbStates(double heightMeters) {
            this.heightMeters = heightMeters;
        }

        
    }
}