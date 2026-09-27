package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {

    private ShotCalculator calc;

    @BeforeEach
    void setUp() {
        ShotLUT lut = new ShotLUT();
        lut.put(1.0, new ShotLUT.ShotParameters(2000, 30.0, 0.5));
        lut.put(2.0, new ShotLUT.ShotParameters(2500, 25.0, 0.7));
        lut.put(3.0, new ShotLUT.ShotParameters(3000, 20.0, 0.9));
        lut.put(4.0, new ShotLUT.ShotParameters(3500, 17.0, 1.1));
        lut.put(5.0, new ShotLUT.ShotParameters(4000, 15.0, 1.3));

        ShotCalculator.Config config = new ShotCalculator.Config();
        config.launcherOffsetX = 0.0;
        config.launcherOffsetY = 0.0;
        calc = new ShotCalculator(config, lut);
    }

    @Test
    void staticShot_returnsLUTValues_andHoodAngle() {
        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            new Pose2d(0, 0, new Rotation2d()),
            new ChassisSpeeds(),
            new ChassisSpeeds(),
            new Translation2d(3.0, 0),
            new Translation2d(1, 0),
            1.0
        );

        ShotCalculator.LaunchParameters result = calc.calculate(inputs);
        assertTrue(result.isValid(), "Static shot should be valid");
        assertEquals(3000, result.rpm(), 100, "RPM should match LUT at 3m");
        assertEquals(20.0, result.hoodAngleDeg(), 2.0, "Hood angle should match LUT at 3m");
    }

    @Test
    void hoodAngleDeg_interpolatesBetweenEntries() {
        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            new Pose2d(0, 0, new Rotation2d()),
            new ChassisSpeeds(),
            new ChassisSpeeds(),
            new Translation2d(2.5, 0),
            new Translation2d(1, 0),
            1.0
        );

        ShotCalculator.LaunchParameters result = calc.calculate(inputs);
        assertTrue(result.isValid());
        assertTrue(result.hoodAngleDeg() > 19 && result.hoodAngleDeg() < 26,
            "Hood angle should interpolate between 20 and 25, got " + result.hoodAngleDeg());
    }

    @Test
    void invalidShot_hasZeroHoodAngle() {
        ShotCalculator.LaunchParameters invalid = ShotCalculator.LaunchParameters.INVALID;
        assertEquals(0, invalid.hoodAngleDeg());
    }
}
