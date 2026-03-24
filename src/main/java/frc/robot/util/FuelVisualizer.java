// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Visualizes fuel projectiles in flight as Pose3d objects for AdvantageScope. Fuel follows a
 * parabolic arc from the shooter to the target based on time-of-flight.
 */
public final class FuelVisualizer {

  /** Maximum number of fuel projectiles that can be in flight simultaneously. */
  private static final int kMaxFuel = 50;

  /** Spawn rate in seconds (4 per second = 0.25s interval). */
  private static final double kSpawnIntervalSeconds = 0.25;

  /** Timestamp of last fuel spawn for rate limiting. */
  private static double m_lastSpawnTime;

  /** Number of fuel currently stored in the robot. */
  private static int m_storedFuelCount;

  /** Publisher for stored fuel count. */
  private static final IntegerPublisher m_fuelCountPub =
      NetworkTableInstance.getDefault().getIntegerTopic("Mechanism/StoredFuelCount").publish();

  /** Represents a single fuel projectile in flight. */
  private record FuelProjectile(
      Translation3d startPosition,
      Translation2d targetXY,
      double targetZ,
      double startTime,
      double timeOfFlight) {

    /**
     * Calculates the current position of the fuel based on elapsed time.
     *
     * @param currentTime The current timestamp.
     * @return The interpolated Pose3d position, or null if the fuel has reached its destination.
     */
    public Pose3d calculatePosition(double currentTime) {
      double elapsed = currentTime - startTime;
      if (elapsed < 0 || elapsed > timeOfFlight) {
        return null;
      }

      // Normalized time (0 to 1)
      double t = elapsed / timeOfFlight;

      // Linear interpolation for X and Y
      double x = startPosition.getX() + (targetXY.getX() - startPosition.getX()) * t;
      double y = startPosition.getY() + (targetXY.getY() - startPosition.getY()) * t;

      // Parabolic arc for Z
      // Peak height scales with distance (longer shots = higher arcs)
      double distance =
          Math.hypot(
              targetXY.getX() - startPosition.getX(), targetXY.getY() - startPosition.getY());
      double peakOffset = 0.25 * distance; // Peak rises 25% of horizontal distance

      // Parabola: starts at startZ, ends at targetZ, peaks in the middle
      double linearZ = startPosition.getZ() + (targetZ - startPosition.getZ()) * t;
      double parabolicOffset = 4.0 * peakOffset * t * (1.0 - t);
      double z = linearZ + parabolicOffset;

      return new Pose3d(x, y, z, new Rotation3d());
    }

    /**
     * Checks if the fuel has completed its flight.
     *
     * @param currentTime The current timestamp.
     * @return true if the fuel has reached or passed its destination.
     */
    public boolean isExpired(double currentTime) {
      return (currentTime - startTime) > timeOfFlight;
    }
  }

  private static final List<FuelProjectile> m_activeFuel = new ArrayList<>();
  private static final StructArrayPublisher<Pose3d> m_publisher;

  static {
    m_publisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Mechanism/Fuel", Pose3d.struct)
            .publish();
  }

  /** Private constructor to prevent instantiation. */
  private FuelVisualizer() {}

  /**
   * Spawns a new fuel projectile from the robot's shooter position toward the target.
   *
   * @param robotPose The current robot pose on the field.
   * @param targetXY The target position (hub center) on the field.
   * @param distanceToTarget The distance to target in meters (used for TOF lookup).
   */
  public static void spawnFuel(Pose2d robotPose, Translation2d targetXY, double distanceToTarget) {
    // Remove oldest fuel if at capacity
    if (m_activeFuel.size() >= kMaxFuel) {
      m_activeFuel.remove(0);
    }

    // Calculate shooter position in field coordinates
    // Shooter offset from robot center (robot-relative)
    Translation3d shooterOffset = Constants.FuelConstants.kShooterOffset;

    // Transform to field coordinates
    double robotHeading = robotPose.getRotation().getRadians();
    double cosHeading = Math.cos(robotHeading);
    double sinHeading = Math.sin(robotHeading);

    double fieldX =
        robotPose.getX() + shooterOffset.getX() * cosHeading - shooterOffset.getY() * sinHeading;
    double fieldY =
        robotPose.getY() + shooterOffset.getX() * sinHeading + shooterOffset.getY() * cosHeading;
    double fieldZ = shooterOffset.getZ();

    Translation3d startPosition = new Translation3d(fieldX, fieldY, fieldZ);

    // Look up time of flight from the map
    double tof = Constants.ShooterConstants.TIME_OF_FLIGHT_MAP.get(distanceToTarget);

    // Create and add the new fuel projectile
    FuelProjectile fuel =
        new FuelProjectile(
            startPosition,
            targetXY,
            Constants.FuelConstants.kTargetHeight,
            Timer.getFPGATimestamp(),
            tof);

    m_activeFuel.add(fuel);
  }

  /**
   * Updates all fuel positions and publishes to NetworkTables. Call this once per robot periodic
   * cycle.
   */
  public static void update() {
    double currentTime = Timer.getFPGATimestamp();

    // Remove expired fuel and collect current positions
    List<Pose3d> positions = new ArrayList<>();
    Iterator<FuelProjectile> iterator = m_activeFuel.iterator();

    while (iterator.hasNext()) {
      FuelProjectile fuel = iterator.next();
      if (fuel.isExpired(currentTime)) {
        iterator.remove();
      } else {
        Pose3d position = fuel.calculatePosition(currentTime);
        if (position != null) {
          positions.add(position);
        }
      }
    }

    // Publish the array of fuel positions
    m_publisher.set(positions.toArray(new Pose3d[0]));
  }

  /** Clears all active fuel projectiles. */
  public static void clear() {
    m_activeFuel.clear();
  }

  /** Increments the stored fuel count (called when fuel is intaked). */
  public static void addFuel() {
    m_storedFuelCount++;
    m_fuelCountPub.set(m_storedFuelCount);
  }

  /**
   * Gets the current stored fuel count.
   *
   * @return Number of fuel stored in the robot.
   */
  public static int getStoredFuelCount() {
    return m_storedFuelCount;
  }

  /**
   * Sets the stored fuel count.
   *
   * @param count The new fuel count.
   */
  public static void setStoredFuelCount(int count) {
    m_storedFuelCount = count;
    m_fuelCountPub.set(m_storedFuelCount);
  }

  /** Resets the stored fuel count to zero. */
  public static void resetFuelCount() {
    m_storedFuelCount = 0;
    m_fuelCountPub.set(m_storedFuelCount);
  }

  /**
   * Attempts to spawn fuel at a rate-limited interval (4 per second). Call this every cycle while
   * shooting; it will only spawn if enough time has passed since the last spawn and fuel is
   * available.
   *
   * @param robotPose The current robot pose on the field.
   * @param targetXY The target position (hub center) on the field.
   * @param distanceToTarget The distance to target in meters (used for TOF lookup).
   */
  public static void trySpawnFuel(
      Pose2d robotPose, Translation2d targetXY, double distanceToTarget) {
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - m_lastSpawnTime >= kSpawnIntervalSeconds && m_storedFuelCount > 0) {
      m_storedFuelCount--;
      m_fuelCountPub.set(m_storedFuelCount);

      spawnFuel(robotPose, targetXY, distanceToTarget);
      m_lastSpawnTime = currentTime;

      // Also spawn in physics simulation if running
      if (RobotBase.isSimulation()) {
        spawnFuelInSim(robotPose, targetXY, distanceToTarget);
      }
    }
  }

  /**
   * Spawns a fuel projectile in the FuelSim physics simulation.
   *
   * @param robotPose The current robot pose on the field.
   * @param targetXY The target position (hub center) on the field.
   * @param distanceToTarget The distance to target in meters.
   */
  private static void spawnFuelInSim(
      Pose2d robotPose, Translation2d targetXY, double distanceToTarget) {
    FuelSim fuelSim = RobotContainer.getFuelSim();
    if (fuelSim == null) {
      return;
    }

    double tof = Constants.ShooterConstants.TIME_OF_FLIGHT_MAP.get(distanceToTarget);
    double launchHeight = Constants.FuelConstants.kShooterOffset.getZ();
    double targetHeight = Constants.FuelConstants.kTargetHeight;

    // Calculate launch velocity using projectile motion equations
    // Horizontal: v_h = distance / t
    // Vertical: targetHeight = launchHeight + v_v * t - 0.5 * g * t^2
    //           v_v = (targetHeight - launchHeight + 0.5 * g * t^2) / t
    double gravity = 9.81;
    double horizontalVel = distanceToTarget / tof;
    double verticalVel = (targetHeight - launchHeight + 0.5 * gravity * tof * tof) / tof;

    double launchSpeed = Math.hypot(horizontalVel, verticalVel);
    double hoodAngleRad = Math.atan2(verticalVel, horizontalVel);

    // Calculate turret yaw (robot-relative angle to target)
    Translation2d robotToTarget = targetXY.minus(robotPose.getTranslation());
    Rotation2d angleToTarget = robotToTarget.getAngle();
    Rotation2d turretYaw = angleToTarget.minus(robotPose.getRotation());

    fuelSim.launchFuel(
        MetersPerSecond.of(launchSpeed),
        Degrees.of(Math.toDegrees(hoodAngleRad)),
        Degrees.of(turretYaw.getDegrees()),
        Meters.of(launchHeight));
  }
}
