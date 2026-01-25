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
 * Utility class for calculating Shooting On The Fly (SOTF) adjustments.
 *
 * <p>SOTF compensates for robot velocity when shooting. The core concept is vector subtraction:
 * V_shot = V_target - V_robot
 *
 * <p>Since this robot has no turret, the calculated shot direction becomes the required robot
 * heading for aiming.
 */
public final class ShootingOnTheFly {

  /** Private constructor to prevent instantiation. */
  private ShootingOnTheFly() {}

  /**
   * Result of SOTF calculation containing adjusted shot parameters.
   *
   * @param aimingAngle The angle the robot should face (field-relative).
   * @param requiredVelocity The required horizontal shot velocity in m/s.
   * @param effectiveDistance The effective distance to use for RPM lookup in meters.
   * @param isPossible Whether the shot is physically possible at current robot velocity.
   */
  public record SOTFResult(
      Rotation2d aimingAngle,
      double requiredVelocity,
      double effectiveDistance,
      boolean isPossible) {}

  /**
   * Calculates the SOTF-adjusted shot parameters.
   *
   * @param robotPose Current robot pose on the field.
   * @param robotSpeeds Current robot velocity (field-relative).
   * @param goalPosition The target position to shoot at.
   * @param latencyCompensation Total latency to compensate for in seconds (camera + motor +
   *     flight).
   * @param timeOfFlightMap Map of distance (m) to time of flight (s) for stationary shots.
   * @param maxHorizontalVelocity Maximum horizontal velocity the shooter can achieve in m/s.
   * @return The calculated SOTF result.
   */
  public static SOTFResult calculate(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation2d goalPosition,
      double latencyCompensation,
      InterpolatingDoubleTreeMap timeOfFlightMap,
      double maxHorizontalVelocity) {

    // Convert chassis speeds to field-relative velocity vector
    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // Project robot position forward to compensate for latency
    Translation2d futurePosition =
        robotPose.getTranslation().plus(robotVelocity.times(latencyCompensation));

    // Calculate vector from future position to goal
    Translation2d toGoal = goalPosition.minus(futurePosition);
    double distance = toGoal.getNorm();

    // Avoid division by zero for very close shots
    if (distance < 0.1) {
      return new SOTFResult(robotPose.getRotation(), 0.0, distance, true);
    }

    // Get baseline horizontal velocity from time-of-flight map
    // V_horizontal = distance / time_of_flight
    double timeOfFlight = timeOfFlightMap.get(distance);
    double baselineHorizontalVelocity = distance / timeOfFlight;

    // Calculate the target velocity vector (pointing at goal, magnitude = baseline velocity)
    Translation2d targetDirection = toGoal.div(distance);
    Translation2d targetVelocity = targetDirection.times(baselineHorizontalVelocity);

    // THE SOTF MAGIC: subtract robot velocity
    // V_shot = V_target - V_robot
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // Extract results
    Rotation2d aimingAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();

    // Check if shot is possible (velocity within shooter capability)
    boolean isPossible = requiredVelocity <= maxHorizontalVelocity;

    // Calculate effective distance for RPM lookup
    // This is the distance that would require the same horizontal velocity when stationary
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity, timeOfFlightMap);

    return new SOTFResult(aimingAngle, requiredVelocity, effectiveDistance, isPossible);
  }

  /**
   * Converts a required horizontal velocity back to an effective distance for RPM lookup.
   *
   * <p>This finds the distance that would produce the given horizontal velocity when shooting
   * stationary, allowing us to use the same RPM lookup table.
   *
   * @param velocity The required horizontal velocity in m/s.
   * @param timeOfFlightMap Map of distance (m) to time of flight (s).
   * @return The effective distance in meters.
   */
  public static double velocityToEffectiveDistance(
      double velocity, InterpolatingDoubleTreeMap timeOfFlightMap) {
    // The relationship is: velocity = distance / timeOfFlight
    // We need to find distance where this holds true
    // Since timeOfFlight generally increases with distance, we can iterate

    // Search range (adjust based on your shooter's capabilities)
    double minDist = 1.0;
    double maxDist = 6.0;
    double step = 0.1;

    double bestDist = minDist;
    double bestError = Double.MAX_VALUE;

    for (double dist = minDist; dist <= maxDist; dist += step) {
      double tof = timeOfFlightMap.get(dist);
      double vel = dist / tof;
      double error = Math.abs(vel - velocity);

      if (error < bestError) {
        bestError = error;
        bestDist = dist;
      }
    }

    return bestDist;
  }

  /**
   * Simplified SOTF calculation that returns just the aiming point for the robot to face.
   *
   * <p>This accounts for robot velocity by calculating where the robot should aim so that the ball
   * reaches the goal after accounting for the robot's motion.
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

    // Convert chassis speeds to field-relative velocity vector
    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // Project robot position forward to compensate for latency
    Translation2d futurePosition =
        robotPose.getTranslation().plus(robotVelocity.times(latencyCompensation));

    // Calculate vector from future position to goal
    Translation2d toGoal = goalPosition.minus(futurePosition);
    double distance = toGoal.getNorm();

    if (distance < 0.1) {
      return goalPosition;
    }

    // Get baseline horizontal velocity
    double timeOfFlight = timeOfFlightMap.get(distance);
    double baselineHorizontalVelocity = distance / timeOfFlight;

    // Calculate target velocity vector
    Translation2d targetDirection = toGoal.div(distance);
    Translation2d targetVelocity = targetDirection.times(baselineHorizontalVelocity);

    // SOTF: V_shot = V_target - V_robot
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

    // The aiming point is the robot's current position plus the shot direction
    // We use a large distance so lookAtPoint gets the correct angle
    Translation2d aimingDirection = shotVelocity.div(shotVelocity.getNorm());
    return futurePosition.plus(aimingDirection.times(10.0));
  }
}
