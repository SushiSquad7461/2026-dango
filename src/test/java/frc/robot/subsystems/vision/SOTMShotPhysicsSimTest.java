package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class SOTMShotPhysicsSimTest {
  private static final ProjectileSimulator.SimParameters PARAMS =
      new ProjectileSimulator.SimParameters(
          0.215,
          0.1501,
          Math.PI * Math.pow(0.1501 / 2.0, 2.0),
          0.47,
          0.0,
          1.225,
          9.81,
          0.43,
          0.1016,
          1.83,
          0.6,
          12.0,
          40.0,
          3.0,
          1.0,
          0.001,
          1500,
          6000,
          25,
          5.0);

  @Test
  void projectileAdvancesAndFallsWithoutNumericalExplosion() {
    SOTMShotPhysicsSim sim = new SOTMShotPhysicsSim(PARAMS);
    sim.spawnProjectile(
        new Pose3d(2.0, 2.0, 0.43, new Rotation3d()),
        new Translation3d(10.0, 0.0, 10.0));

    assertEquals(1, sim.getActiveCount());

    for (int i = 0; i < 50; i++) {
      sim.update(0.02);
    }

    Pose3d afterOneSecond = sim.getActivePoses()[0];
    assertTrue(Double.isFinite(afterOneSecond.getX()));
    assertTrue(Double.isFinite(afterOneSecond.getY()));
    assertTrue(Double.isFinite(afterOneSecond.getZ()));
    assertTrue(afterOneSecond.getX() < 12.0, "Drag should reduce travel below no-drag estimate");

    for (int i = 0; i < 400; i++) {
      sim.update(0.02);
    }

    assertEquals(0, sim.getActiveCount(), "Projectile should be culled after landing/timeout.");
  }
}
