/*
 * ProjectileSimulator.java - RK4 projectile physics with drag and Magnus lift
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */

package frc.robot.subsystems.vision;


/**
 * Simulates a ball flying through the air with drag and Magnus lift. Uses RK4 integration
 * in the vertical plane (x, z). For each distance, binary searches RPM until the ball arrives
 * at the target height. Then generates a 91-point lookup table from 0.50m to 5.00m.
 *
 * <p>Basically, you plug in your robot's measurements from CAD, run generateLUT(), and it gives
 * you a complete shooter table. No more hand-tuning RPM values from match videos.
 *
 * <p>Usage:
 * <pre>
 *   ProjectileSimulator sim = new ProjectileSimulator(Constants.Vision.SOTM_PARAMETERS);
 *   ShotLUT lut = sim.generateLUT();
 *   // lut.get(distance) returns ShotParameters(rpm, angle, tof)
 * </pre>
 */
public class ProjectileSimulator {

  // Your robot's physical measurements, from CAD and the game manual
  public record SimParameters(
      double ballMassKg,
      double ballDiameterM,
      double dragCoeff,
      double magnusCoeff,
      double airDensity,
      double exitHeightM,
      double wheelDiameterM,
      double targetHeightM,
      double slipFactor,
      double minHoodAngleDeg,
      double maxHoodAngleDeg,
      double maxTofCeilingS,
      double angleStepDeg,
      double dt,
      double rpmMin,
      double rpmMax,
      int binarySearchIters,
      double maxSimTime) {}

  public record TrajectoryResult(
      double zAtTarget, double tof, boolean reachedTarget, double maxHeight, double apexX) {}

  // One row: distance -> RPM that lands it, TOF, reachable flag
  private record LUTEntry(double distanceM, double rpm, double tof, boolean reachable) {}

  private final SimParameters params;

  // Precomputed aero constants
  private final double kDrag;
  private final double kMagnus;

  public ProjectileSimulator(SimParameters params) {
    this.params = params;
    double area = Math.PI * (params.ballDiameterM() / 2.0) * (params.ballDiameterM() / 2.0);
    this.kDrag = (params.airDensity() * params.dragCoeff() * area) / (2.0 * params.ballMassKg());
    this.kMagnus =
        (params.airDensity() * params.magnusCoeff() * area) / (2.0 * params.ballMassKg());
  }

  /** RPM to ball exit speed (m/s). Accounts for slip between the wheel surface and ball. */
  public double exitVelocity(double rpm) {
    return params.slipFactor() * rpm * Math.PI * params.wheelDiameterM() / 60.0;
  }

  // state = [x, z, vx, vz]
  // ax = -kDrag * |v| * vx
  // az = -g - kDrag * |v| * vz + kMagnus * |v|^2 (Magnus acts as upward lift)
  private double[] derivatives(double[] state) {
    double svx = state[2];
    double svz = state[3];
    double speed = Math.hypot(svx, svz);

    double ax = -kDrag * speed * svx;
    double az = -9.81 - kDrag * speed * svz + kMagnus * speed * speed;

    return new double[] {svx, svz, ax, az};
  }

  private static double[] addScaled(double[] base, double[] delta, double scale) {
    return new double[] {
      base[0] + delta[0] * scale,
      base[1] + delta[1] * scale,
      base[2] + delta[2] * scale,
      base[3] + delta[3] * scale
    };
  }

  /** Simulate a ball launched at the given RPM and explicit angle. */
  public TrajectoryResult simulate(double rpm, double targetDistanceM, double launchAngleDeg) {
    double v0 = exitVelocity(rpm);
    double launchRad = Math.toRadians(launchAngleDeg);
    double vx = v0 * Math.cos(launchRad);
    double vz = v0 * Math.sin(launchRad);

    double x = 0;
    double z = params.exitHeightM();
    double dt = params.dt();
    double maxHeight = z;
    double apexX = 0;
    double t = 0;
    double maxTime = params.maxSimTime();

    while (t < maxTime) {
      double[] state = {x, z, vx, vz};
      double[] k1 = derivatives(state);
      double[] s2 = addScaled(state, k1, dt / 2.0);
      double[] k2 = derivatives(s2);
      double[] s3 = addScaled(state, k2, dt / 2.0);
      double[] k3 = derivatives(s3);
      double[] s4 = addScaled(state, k3, dt);
      double[] k4 = derivatives(s4);

      x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      z += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      vx += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
      vz += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
      t += dt;

      if (z > maxHeight) { maxHeight = z; apexX = x; }

      if (x >= targetDistanceM) {
        double prevX = x - vx * dt;
        double prevZ = z - vz * dt;
        double frac = (targetDistanceM - prevX) / (x - prevX);
        double zAtTarget = prevZ + frac * (z - prevZ);
        double tofAtTarget = t - dt + frac * dt;
        return new TrajectoryResult(zAtTarget, tofAtTarget, true, maxHeight, apexX);
      }

      if (z < 0) { return new TrajectoryResult(0, t, false, maxHeight, apexX); }
    }

    return new TrajectoryResult(0, maxTime, false, maxHeight, apexX);
  }

  /** Binary search for RPM at a specific launch angle that lands at target height. */
  private LUTEntry findRPMForAngle(double distanceM, double launchAngleDeg) {
    double heightTolerance = 0.02;
    double lo = params.rpmMin();
    double hi = params.rpmMax();

    TrajectoryResult maxCheck = simulate(hi, distanceM, launchAngleDeg);
    if (!maxCheck.reachedTarget()) {
      return new LUTEntry(distanceM, 0, 0, false);
    }

    double bestRpm = hi;
    double bestTof = maxCheck.tof();
    double bestError = Math.abs(maxCheck.zAtTarget() - params.targetHeightM());

    for (int i = 0; i < params.binarySearchIters(); i++) {
      double mid = (lo + hi) / 2.0;
      TrajectoryResult result = simulate(mid, distanceM, launchAngleDeg);

      if (!result.reachedTarget()) { lo = mid; continue; }

      double error = result.zAtTarget() - params.targetHeightM();
      double absError = Math.abs(error);

      if (absError < bestError) { bestRpm = mid; bestTof = result.tof(); bestError = absError; }
      if (absError < heightTolerance) { return new LUTEntry(distanceM, mid, result.tof(), true); }

      if (error > 0) { hi = mid; } else { lo = mid; }
    }

    return new LUTEntry(distanceM, bestRpm, bestTof, bestError < 0.10);
  }

  /**
   * Generate unified shot LUT: sweep hood angles per distance, find lowest RPM
   * within TOF ceiling. Takes ~5-6 seconds at startup.
   */
  public ShotLUT generateLUT() {
    ShotLUT lut = new ShotLUT();

    double minAngle = params.minHoodAngleDeg();
    double maxAngle = params.maxHoodAngleDeg();
    double angleStep = params.angleStepDeg();
    double tofCeiling = params.maxTofCeilingS();
    double rpmTiebreaker = 10.0;

    for (int i = 0; i <= 90; i++) {
      double distance = 0.50 + i * 0.05;
      distance = Math.round(distance * 100.0) / 100.0;

      double bestRpm = Double.MAX_VALUE;
      double bestAngle = minAngle;
      double bestTof = tofCeiling;

      for (double angle = minAngle; angle <= maxAngle; angle += angleStep) {
        LUTEntry entry = findRPMForAngle(distance, angle);
        if (!entry.reachable()) continue;
        if (entry.tof() > tofCeiling) continue;

        boolean betterRpm = entry.rpm() < bestRpm - rpmTiebreaker;
        boolean tiedRpm = Math.abs(entry.rpm() - bestRpm) <= rpmTiebreaker;
        boolean betterTof = entry.tof() < bestTof;

        if (betterRpm || (tiedRpm && betterTof)) {
          bestRpm = entry.rpm();
          bestAngle = angle;
          bestTof = entry.tof();
        }
      }

      if (bestRpm < Double.MAX_VALUE) {
        lut.put(distance, new ShotLUT.ShotParameters(bestRpm, bestAngle, bestTof));
      }
    }

    return lut;
  }

  // Package-private for testing
  double getKDrag() {
    return kDrag;
  }

  double getKMagnus() {
    return kMagnus;
  }
}
