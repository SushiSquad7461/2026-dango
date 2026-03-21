package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.generated.Constants;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Runtime projectile simulation for FUEL game pieces in desktop sim.
 *
 * <p>This class is intentionally separate from {@link ProjectileSimulator}: the LUT generator solves
 * a 2D shot table at startup, while this class updates live 3D in-flight projectiles during
 * periodic loops and logs them for AdvantageScope.
 */
public class SOTMShotPhysicsSim {
  public record LaunchState(Pose3d releasePose, Translation3d launchVelocity) {}

  private static final double MAX_PROJECTILE_AGE_SEC = 5.0;
  private static final int MAX_TRAIL_SAMPLES = 40;

  private static class ActiveProjectile {
    private Pose3d pose;
    private Translation3d velocity;
    private double ageSec;
    private final ArrayDeque<Pose3d> trail = new ArrayDeque<>();

    ActiveProjectile(Pose3d pose, Translation3d velocity) {
      this.pose = pose;
      this.velocity = velocity;
      this.ageSec = 0.0;
      this.trail.addLast(pose);
    }
  }

  private final ProjectileSimulator.SimParameters params;
  // k = 0.5 * rho * Cd * A / m from the standard drag equation.
  private final double dragAccelerationFactorK;
  private final double kMagnus;
  private final List<ActiveProjectile> activeProjectiles = new ArrayList<>();

  public SOTMShotPhysicsSim(ProjectileSimulator.SimParameters params) {
    this.params = params;
    this.dragAccelerationFactorK =
        (params.airDensity() * params.dragCoeff() * params.ballFrontalAreaM2())
            / (2.0 * params.ballMassKg());
    this.kMagnus =
        (params.airDensity() * params.magnusCoeff() * params.ballFrontalAreaM2())
            / (2.0 * params.ballMassKg());
  }

  public void spawnProjectile(LaunchState launchState) {
    if (launchState == null) {
      return;
    }
    spawnProjectile(launchState.releasePose(), launchState.launchVelocity());
  }

  public void spawnProjectile(Pose3d releasePose, Translation3d launchVelocity) {
    if (releasePose == null || launchVelocity == null) {
      return;
    }
    activeProjectiles.add(new ActiveProjectile(releasePose, launchVelocity));
  }

  public void clear() {
    activeProjectiles.clear();
  }

  public int getActiveCount() {
    return activeProjectiles.size();
  }

  public Pose3d[] getActivePoses() {
    Pose3d[] poses = new Pose3d[activeProjectiles.size()];
    for (int i = 0; i < activeProjectiles.size(); i++) {
      poses[i] = activeProjectiles.get(i).pose;
    }
    return poses;
  }

  public void update(double dtSeconds) {
    if (activeProjectiles.isEmpty() || dtSeconds <= 0.0) {
      return;
    }

    // Keep integration stable if loop timing spikes.
    int steps = Math.max(1, (int) Math.ceil(dtSeconds / 0.005));
    double stepDt = dtSeconds / steps;

    for (int step = 0; step < steps; step++) {
      Iterator<ActiveProjectile> iterator = activeProjectiles.iterator();
      while (iterator.hasNext()) {
        ActiveProjectile projectile = iterator.next();
        integrate(projectile, stepDt);
        projectile.ageSec += stepDt;
        if (!isProjectileAlive(projectile)) {
          iterator.remove();
        }
      }
    }
  }

  public void logOutputs() {
    Logger.recordOutput("Gamepieces/Fuel", getActivePoses());
    Logger.recordOutput("Gamepieces/SOTMTrajectory", getTrailPoses());
  }

  private Pose3d[] getTrailPoses() {
    List<Pose3d> trail = new ArrayList<>();
    for (ActiveProjectile projectile : activeProjectiles) {
      trail.addAll(projectile.trail);
    }
    return trail.toArray(Pose3d[]::new);
  }

  private void integrate(ActiveProjectile projectile, double dt) {
    double[] state = {
      projectile.pose.getX(),
      projectile.pose.getY(),
      projectile.pose.getZ(),
      projectile.velocity.getX(),
      projectile.velocity.getY(),
      projectile.velocity.getZ()
    };

    double[] k1 = derivatives(state);
    double[] k2 = derivatives(addScaled(state, k1, dt / 2.0));
    double[] k3 = derivatives(addScaled(state, k2, dt / 2.0));
    double[] k4 = derivatives(addScaled(state, k3, dt));

    double[] next = new double[6];
    for (int i = 0; i < next.length; i++) {
      next[i] = state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    projectile.pose = new Pose3d(next[0], next[1], next[2], projectile.pose.getRotation());
    projectile.velocity = new Translation3d(next[3], next[4], next[5]);
    projectile.trail.addLast(projectile.pose);
    while (projectile.trail.size() > MAX_TRAIL_SAMPLES) {
      projectile.trail.removeFirst();
    }
  }

  private double[] derivatives(double[] state) {
    double vx = state[3];
    double vy = state[4];
    double vz = state[5];
    double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);

    if (speed < 1e-9) {
      return new double[] {vx, vy, vz, 0.0, 0.0, -params.gravityMps2()};
    }

    // Quadratic drag component form:
    // a_drag_x = -k * |v| * vx
    // a_drag_y = -k * |v| * vy
    // a_drag_z = -k * |v| * vz
    double axDrag = -dragAccelerationFactorK * speed * vx;
    double ayDrag = -dragAccelerationFactorK * speed * vy;
    double azDrag = -dragAccelerationFactorK * speed * vz;

    // Magnus is kept parameterized, defaulting to zero.
    double magnusLift = kMagnus * speed * speed;

    double ax = axDrag;
    double ay = ayDrag;
    double az = -params.gravityMps2() + azDrag + magnusLift;

    return new double[] {vx, vy, vz, ax, ay, az};
  }

  private static double[] addScaled(double[] base, double[] delta, double scale) {
    double[] out = new double[base.length];
    for (int i = 0; i < base.length; i++) {
      out[i] = base[i] + delta[i] * scale;
    }
    return out;
  }

  private boolean isProjectileAlive(ActiveProjectile projectile) {
    Pose3d pose = projectile.pose;
    if (pose.getZ() <= 0.0) {
      return false;
    }
    if (projectile.ageSec > Math.min(params.maxSimTime(), MAX_PROJECTILE_AGE_SEC)) {
      return false;
    }

    double x = pose.getX();
    double y = pose.getY();
    double fieldMargin = 2.0;
    return x >= -fieldMargin
        && x <= Constants.Vision.FIELD_LENGTH_METERS + fieldMargin
        && y >= -fieldMargin
        && y <= Constants.Vision.FIELD_WIDTH_METERS + fieldMargin;
  }
}
