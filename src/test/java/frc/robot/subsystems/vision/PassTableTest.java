package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class PassTableTest {
    @Test
    void buildLUT_returnsNonNull() {
        ShotLUT lut = PassTable.buildLUT();
        assertNotNull(lut);
    }

    @Test
    void buildLUT_interpolatesAtKnownDistance() {
        ShotLUT lut = PassTable.buildLUT();
        ShotLUT.ShotParameters params = lut.get(6.0);
        assertNotNull(params);
        assertTrue(params.rpm() > 0, "RPM should be positive");
        assertTrue(params.angle() > 0, "Angle should be positive");
        assertTrue(params.tof() > 0, "TOF should be positive");
    }

    @Test
    void buildLUT_interpolatesBetweenEntries() {
        ShotLUT lut = PassTable.buildLUT();
        ShotLUT.ShotParameters at5 = lut.get(5.0);
        ShotLUT.ShotParameters at6 = lut.get(6.0);
        ShotLUT.ShotParameters at5_5 = lut.get(5.5);
        assertTrue(at5_5.rpm() >= at5.rpm() && at5_5.rpm() <= at6.rpm(),
            "RPM at 5.5m should interpolate between 5m and 6m values");
    }
}
