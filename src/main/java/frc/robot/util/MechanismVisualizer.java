// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

/**
 * Utility class for publishing mechanism poses to NetworkTables for AdvantageScope 3D
 * visualization. Multiple subsystems can update their respective poses, and the combined array is
 * published each cycle.
 */
public final class MechanismVisualizer {

  /** Indices for each mechanism in the poses array. */
  public static final int INTAKE_INDEX = 0;
  public static final int HOPPER_INDEX = 1;
  public static final int CLIMBER_INDEX = 2;

  private static final int NUM_MECHANISMS = 3;

  private static final Pose3d[] m_poses = new Pose3d[NUM_MECHANISMS];
  private static final StructArrayPublisher<Pose3d> m_publisher;

  static {
    // Initialize all poses to identity
    for (int i = 0; i < NUM_MECHANISMS; i++) {
      m_poses[i] = new Pose3d();
    }

    // Create publisher at top-level Mechanism table
    m_publisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Mechanism/Poses", Pose3d.struct)
            .publish();
  }

  /** Private constructor to prevent instantiation. */
  private MechanismVisualizer() {}

  /**
   * Updates the pose for a specific mechanism.
   *
   * @param index The mechanism index (use constants like INTAKE_INDEX).
   * @param pose The new pose for this mechanism.
   */
  public static void setPose(int index, Pose3d pose) {
    if (index >= 0 && index < NUM_MECHANISMS) {
      m_poses[index] = pose;
    }
  }

  /** Publishes all mechanism poses to NetworkTables. Call this once per robot periodic cycle. */
  public static void publish() {
    m_publisher.set(m_poses);
  }
}
