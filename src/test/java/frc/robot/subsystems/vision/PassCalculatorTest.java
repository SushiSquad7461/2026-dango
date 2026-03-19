package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PassCalculatorTest {

    private PassCalculator calc;

    @BeforeEach
    void setUp() {
        calc = new PassCalculator(PassTable.buildLUT());
    }

    @Test
    void outsideCentralZone_returnsInvalid() {
        Pose2d pose = new Pose2d(2.0, 4.0, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertFalse(result.isValid(), "Should be invalid outside central zone");
    }

    @Test
    void inCentralZone_unblocked_returnsClosestPoint() {
        Pose2d pose = new Pose2d(8.0, 1.5, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertTrue(result.isValid(), "Should be valid in central zone");
        assertFalse(result.isBlocked(), "Y=1.5 is clear of hub shadow");
        assertEquals(2.5, result.targetPoint().getX(), 0.01);
        assertEquals(1.5, result.targetPoint().getY(), 0.01);
    }

    @Test
    void inCentralZone_blocked_slidesToClearY() {
        Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertTrue(result.isValid(), "Should be valid in central zone");
        assertTrue(result.isBlocked(), "Y=4.0 is in hub shadow");
        double targetY = result.targetPoint().getY();
        double hubBlockMin = 4.03 - 0.597 - 0.3;
        double hubBlockMax = 4.03 + 0.597 + 0.3;
        assertTrue(targetY <= hubBlockMin || targetY >= hubBlockMax,
            "Target Y should be outside hub shadow, got " + targetY);
    }

    @Test
    void inCentralZone_nearFieldEdge_clampsTargetY() {
        Pose2d pose = new Pose2d(8.0, 0.2, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertTrue(result.isValid());
        assertTrue(result.targetPoint().getY() >= 0.5,
            "Target Y should be clamped to min 0.5m from wall");
    }

    @Test
    void returnsPositiveRpmAndAngle() {
        Pose2d pose = new Pose2d(8.0, 2.0, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertTrue(result.isValid());
        assertTrue(result.rpm() > 0, "RPM should be positive");
        assertTrue(result.hoodAngleDeg() > 0, "Hood angle should be positive");
        assertTrue(result.tofSec() > 0, "TOF should be positive");
    }

    @Test
    void aimAngle_pointsTowardTarget() {
        Pose2d pose = new Pose2d(8.0, 2.0, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertTrue(result.isValid());
        double aimDeg = Math.toDegrees(result.aimAngleRad());
        assertEquals(180.0, Math.abs(aimDeg), 5.0,
            "Aim angle should point toward blue wall, got " + aimDeg);
    }

    @Test
    void redAlliance_targetsRedSide() {
        Pose2d pose = new Pose2d(8.0, 2.0, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Red);
        assertTrue(result.isValid());
        assertEquals(14.0, result.targetPoint().getX(), 0.01,
            "Red alliance should target X=14.0");
    }

    @Test
    void distanceMatchesGeometry() {
        Pose2d pose = new Pose2d(8.0, 2.0, new Rotation2d());
        PassCalculator.PassParameters result = calc.calculate(pose, Alliance.Blue);
        assertTrue(result.isValid());
        double expectedDist = pose.getTranslation().getDistance(result.targetPoint());
        assertEquals(expectedDist, result.distance(), 0.01);
    }
}
