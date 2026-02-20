// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Utility class for calculating Shooting On The Fly (SOTF) adjustments using iterative
 * time-of-flight recursion.
 *
 * <p>The algorithm works by repeatedly offsetting the goal position by the robot's velocity times
 * the time-of-flight to create a "virtual target." Each virtual target has its own TOF from the
 * look-up table, so we iterate until the TOF stabilizes (typically 3-5 iterations). This approach
 * implicitly encodes the physics of the problem in the look-up table, avoiding the need for
 * explicit ballistic modeling at runtime.
 */
public final class ShootingOnTheFly {

  /** Maximum number of TOF recursion iterations before giving up. */
  private static final int kMaxIterations = 10;

  /** TOF convergence tolerance in seconds. */
  private static final double kTofToleranceSeconds = 0.001;

  /** Private constructor to prevent instantiation. */
  private ShootingOnTheFly() {}

  /**
   * Result of the iterative SOTF calculation.
   *
   * @param virtualTarget The position the robot should aim at (field-relative).
   * @param aimingAngle The angle the robot should face (field-relative).
   * @param virtualDistance The distance to the virtual target, used for RPM lookup.
   * @param timeOfFlight The converged time-of-flight in seconds.
   * @param iterations The number of iterations it took to converge.
   * @param converged Whether the iteration converged within the iteration budget.
   * @param contractionFactor The contraction rate of the fixed-point iteration (0 = stable, 1 =
   *     fragile). This indicates how much platform velocity error is amplified by the shot
   *     geometry.
   * @param firstOrderMiss The estimated miss distance in meters due to velocity uncertainty (tau *
   *     |delta_v|).
   */
  public record SOTFResult(
      Translation2d virtualTarget,
      Rotation2d aimingAngle,
      double virtualDistance,
      double timeOfFlight,
      int iterations,
      boolean converged,
      double contractionFactor,
      double firstOrderMiss) {}

  /**
   * Calculates the SOTF-adjusted shot parameters using iterative TOF recursion.
   *
   * <p>The algorithm:
   *
   * <ol>
   *   <li>Compute the initial distance to the goal and look up its TOF.
   *   <li>Offset the goal by -velocity * TOF to get a "virtual target."
   *   <li>Look up the TOF for the virtual target's distance.
   *   <li>Repeat until the TOF stops changing (converges).
   *   <li>The converged virtual target is what the robot should aim at, and its distance is used
   *       for RPM lookup.
   * </ol>
   *
   * @param robotPose Current robot pose on the field.
   * @param robotSpeeds Current robot velocity (field-relative).
   * @param goalPosition The target position to shoot at.
   * @param latencyCompensation Total latency to compensate for in seconds (camera + motor lag).
   * @param timeOfFlightMap Map of distance (m) to time of flight (s) for stationary shots.
   * @param velocityUncertainty Estimated velocity uncertainty magnitude in m/s (for miss metric).
   * @return The calculated SOTF result.
   */
  public static SOTFResult calculate(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation2d goalPosition,
      double latencyCompensation,
      InterpolatingDoubleTreeMap timeOfFlightMap,
      double velocityUncertainty) {

    // Convert chassis speeds to field-relative velocity vector
    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // Project robot position forward to compensate for system latency
    Translation2d futurePosition =
        robotPose.getTranslation().plus(robotVelocity.times(latencyCompensation));

    // Vector from (latency-compensated) robot to goal — this is our base displacement
    Translation2d toGoal = goalPosition.minus(futurePosition);
    double staticDistance = toGoal.getNorm();

    // Degenerate case: very close to goal
    if (staticDistance < 0.1) {
      return new SOTFResult(
          goalPosition, toGoal.getAngle(), staticDistance, 0.0, 0, true, 0.0, 0.0);
    }

    // Step 1: Initial TOF guess from the static distance
    double tof = timeOfFlightMap.get(staticDistance);

    // Step 2-4: Iterate — offset goal by velocity * TOF, look up new TOF, repeat
    int iterations = 0;
    double secondToLastTofStep = 0.0;
    double lastTofStep = 0.0;

    for (int i = 0; i < kMaxIterations; i++) {
      // Virtual target: offset the goal opposite to the robot's motion over the TOF
      // d(tau) = (goal - robot) - v * tau
      Translation2d virtualTargetOffset = futurePosition.plus(robotVelocity.times(tof));
      Translation2d toVirtualTarget = goalPosition.minus(virtualTargetOffset);
      double virtualDistance = toVirtualTarget.getNorm();

      // Look up the TOF for this virtual distance
      double newTof = timeOfFlightMap.get(virtualDistance);
      double tofStep = Math.abs(newTof - tof);

      // Track the last two steps for contraction factor
      secondToLastTofStep = lastTofStep;
      lastTofStep = tofStep;

      tof = newTof;
      iterations = i + 1;

      // Check convergence
      if (tofStep < kTofToleranceSeconds) {
        break;
      }
    }

    // Compute the final virtual target with the converged TOF
    Translation2d finalVirtualOffset = futurePosition.plus(robotVelocity.times(tof));
    Translation2d toFinalVirtualTarget = goalPosition.minus(finalVirtualOffset);
    double finalVirtualDistance = toFinalVirtualTarget.getNorm();

    // The aiming angle is from the robot toward the virtual target
    Rotation2d aimingAngle = toFinalVirtualTarget.getAngle();

    // Contraction factor: ratio of last two TOF steps (0 = good, 1 = fragile)
    double contractionFactor = 0.0;
    if (secondToLastTofStep > kTofToleranceSeconds) {
      contractionFactor = lastTofStep / secondToLastTofStep;
    }

    // First-order platform miss: tau * |delta_v|
    double firstOrderMiss = tof * velocityUncertainty;

    boolean converged = lastTofStep < kTofToleranceSeconds;

    return new SOTFResult(
        goalPosition.minus(robotVelocity.times(tof)),
        aimingAngle,
        finalVirtualDistance,
        tof,
        iterations,
        converged,
        contractionFactor,
        firstOrderMiss);
  }

  /**
   * Simplified calculation that returns just the aiming point for the robot to face.
   *
   * <p>Uses the full iterative TOF recursion internally, then returns a point far along the aiming
   * direction so it can be used with a "look at point" drive request.
   *
   * @param robotPose Current robot pose on the field.
   * @param robotSpeeds Current robot velocity (field-relative).
   * @param goalPosition The target position to shoot at.
   * @param latencyCompensation Total latency to compensate for in seconds.
   * @param timeOfFlightMap Map of distance (m) to time of flight (s).
   * @return The point the robot should aim at (for use with lookAtPoint).
   */
  public static Translation2d calculateAimingPoint(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation2d goalPosition,
      double latencyCompensation,
      InterpolatingDoubleTreeMap timeOfFlightMap) {

    SOTFResult result =
        calculate(robotPose, robotSpeeds, goalPosition, latencyCompensation, timeOfFlightMap, 0.0);

    // Project a point far along the aiming direction so lookAtPoint gets the correct angle
    Translation2d futurePosition =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                    .times(latencyCompensation));

    Translation2d aimingDirection =
        new Translation2d(result.aimingAngle().getCos(), result.aimingAngle().getSin());
    return futurePosition.plus(aimingDirection.times(10.0));
  }
}
