package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ProjectileSimulatorTest {

    private static final ProjectileSimulator.SimParameters PARAMS =
        new ProjectileSimulator.SimParameters(
            0.215, 0.1501, 0.47, 0.2, 1.225,
            0.43, 0.1016, 1.83, 0.6,
            12.0, 40.0, 3.0, 1.0,
            0.001, 1500, 6000, 25, 5.0);

    @Test
    void simulate_withExplicitAngle_usesThatAngle() {
        ProjectileSimulator sim = new ProjectileSimulator(PARAMS);
        var result20 = sim.simulate(3000, 2.0, 20.0);
        var result40 = sim.simulate(3000, 2.0, 40.0);
        assertTrue(result20.reachedTarget() || result40.reachedTarget(),
            "At least one angle should reach 2m at 3000 RPM");
        if (result20.reachedTarget() && result40.reachedTarget()) {
            assertNotEquals(result20.zAtTarget(), result40.zAtTarget(), 0.01,
                "Different launch angles should produce different z at target");
        }
    }

    @Test
    void generateShotLUT_returnsPopulatedLUT() {
        ProjectileSimulator sim = new ProjectileSimulator(PARAMS);
        ShotLUT lut = sim.generateShotLUT();
        assertNotNull(lut);

        ShotLUT.ShotParameters shot = lut.get(2.0);
        assertNotNull(shot);
        assertTrue(shot.rpm() >= 1500 && shot.rpm() <= 6000,
            "RPM in valid range, got " + shot.rpm());
        assertTrue(shot.angle() >= 12.0 && shot.angle() <= 40.0,
            "Angle in hood range, got " + shot.angle());
        assertTrue(shot.tof() > 0 && shot.tof() <= 3.0,
            "TOF positive and under ceiling, got " + shot.tof());
    }

    @Test
    void generateShotLUT_tightTofCeiling_requiresHigherRPM() {
        ProjectileSimulator.SimParameters tightParams = new ProjectileSimulator.SimParameters(
            0.215, 0.1501, 0.47, 0.2, 1.225,
            0.43, 0.1016, 1.83, 0.6,
            12.0, 40.0, 0.5, 1.0,
            0.001, 1500, 6000, 25, 5.0);

        ShotLUT lutTight = new ProjectileSimulator(tightParams).generateShotLUT();
        ShotLUT lutLoose = new ProjectileSimulator(PARAMS).generateShotLUT();

        ShotLUT.ShotParameters tight = lutTight.get(2.0);
        ShotLUT.ShotParameters loose = lutLoose.get(2.0);
        if (tight != null && loose != null) {
            assertTrue(tight.rpm() >= loose.rpm() - 100,
                "Tight TOF should need >= RPM. tight=" + tight.rpm() + " loose=" + loose.rpm());
        }
    }
}
