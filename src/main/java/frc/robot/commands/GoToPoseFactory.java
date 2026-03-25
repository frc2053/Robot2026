package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Factory methods for building GoToPoseCommand instances with common configurations, including
 * alliance flipping and mirroring.
 *
 * <p>All poses are authored for the <b>blue alliance origin</b>. The factory methods accept a
 * {@code shouldFlip} supplier (typically {@code GoToPoseCommand::isRedAlliance}) that transforms
 * the pose to the red side at command init time using PathPlanner's {@link
 * com.pathplanner.lib.util.FlippingUtil}.
 *
 * <h2>Setup required:</h2>
 *
 * <p>Make sure PathPlanner's FlippingUtil is configured in your robot init:
 *
 * <pre>{@code
 * FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
 * FlippingUtil.fieldSizeX = 16.54;
 * FlippingUtil.fieldSizeY = 8.21;
 * }</pre>
 *
 * <p>If you use PathPlanner's AutoBuilder, this is already done for you.
 *
 * <h2>Example usage:</h2>
 *
 * <pre>{@code
 * // All poses authored for blue — auto-flips for red
 * BooleanSupplier flip = GoToPoseCommand::isRedAlliance;
 *
 * // Go to scoring pose, auto-flipped
 * Command cmd = GoToPoseFactory.goTo(drivetrain, scoringPose, flip);
 *
 * // Same pose but mirrored to the other side of the field centerline
 * Command mirrored = GoToPoseFactory.goTo(drivetrain, scoringPose, flip, true);
 *
 * // Precise placement with flip
 * Command precise = GoToPoseFactory.precisePlacement(drivetrain, scoringPose, flip);
 * }</pre>
 */
public final class GoToPoseFactory {

  private GoToPoseFactory() {} // Utility class

  // ── Typical FRC swerve robot defaults ──────────────────────────────
  // Adjust these to match YOUR robot. These are reasonable starting
  // points for a ~120lb robot with Kraken X60s on an SDS MK4i L3.

  /** Default config: moderate speed, good for general autonomous moves. */
  public static GoToPoseCommand.Config defaultConfig() {
    return new GoToPoseCommand.Config()
        .withLinearConstraints(
            SwerveConstants.kGoToPoseMaxVelocity, SwerveConstants.kGoToPoseMaxAcceleration)
        .withAngularConstraints(
            SwerveConstants.kGoToPoseMaxAngularVelocity,
            SwerveConstants.kGoToPoseMaxAngularAcceleration)
        .withMass(SwerveConstants.kRobotMassKg)
        .withMOI(SwerveConstants.kRobotMOIKgM2)
        .withLinearTolerance(SwerveConstants.kGoToPoseLinearTolerance)
        .withAngularTolerance(SwerveConstants.kGoToPoseAngularTolerance)
        .withVelocityTolerance(0.05) // m/s
        .withAngularVelocityTolerance(Math.toRadians(5.0))
        .withSettleCycles(5)
        .withTimeout(5.0);
  }

  /** Fast transit: high speed, loose tolerances, for crossing the field. */
  public static GoToPoseCommand.Config fastTransitConfig() {
    return new GoToPoseCommand.Config()
        .withLinearConstraints(4.5, 3.5)
        .withAngularConstraints(Math.PI * 2, Math.PI * 1.5)
        .withMass(SwerveConstants.kRobotMassKg)
        .withMOI(SwerveConstants.kRobotMOIKgM2)
        .withLinearTolerance(0.05) // 5 cm — looser
        .withAngularTolerance(Math.toRadians(3.0)) // 3 degrees
        .withVelocityTolerance(0.15)
        .withAngularVelocityTolerance(Math.toRadians(10.0))
        .withSettleCycles(3)
        .withTimeout(6.0);
  }

  /** Precise placement: slow, tight tolerances, for scoring. */
  public static GoToPoseCommand.Config precisePlacementConfig() {
    return new GoToPoseCommand.Config()
        .withLinearConstraints(2.0, 1.5)
        .withAngularConstraints(Math.PI, Math.PI * 0.5)
        .withMass(SwerveConstants.kRobotMassKg)
        .withMOI(SwerveConstants.kRobotMOIKgM2)
        .withLinearTolerance(0.01) // 1 cm
        .withAngularTolerance(Math.toRadians(0.5)) // 0.5 degrees
        .withVelocityTolerance(0.02)
        .withAngularVelocityTolerance(Math.toRadians(2.0))
        .withSettleCycles(8)
        .withTimeout(4.0);
  }

  // ── Factory methods — no flipping ──────────────────────────────────

  /** Go to a fixed pose with default config. No alliance flipping. */
  public static GoToPoseCommand goTo(SwerveDrivetrain<?, ?, ?> drivetrain, Pose2d goal) {
    return new GoToPoseCommand(drivetrain, defaultConfig(), goal);
  }

  /** Go to a dynamic pose with default config. No alliance flipping. */
  public static GoToPoseCommand goTo(SwerveDrivetrain<?, ?, ?> drivetrain, Supplier<Pose2d> goal) {
    return new GoToPoseCommand(drivetrain, defaultConfig(), goal);
  }

  // ── Factory methods — with alliance flip ───────────────────────────

  /**
   * Go to a fixed blue-origin pose with alliance flipping.
   *
   * @param drivetrain The drivetrain
   * @param goal Blue-origin target pose
   * @param shouldFlip Returns true to flip for red alliance
   */
  public static GoToPoseCommand goTo(
      SwerveDrivetrain<?, ?, ?> drivetrain, Pose2d goal, BooleanSupplier shouldFlip) {
    return new GoToPoseCommand(drivetrain, defaultConfig(), () -> goal, shouldFlip, false);
  }

  /**
   * Go to a fixed blue-origin pose with alliance flipping and optional mirror.
   *
   * @param drivetrain The drivetrain
   * @param goal Blue-origin target pose
   * @param shouldFlip Returns true to flip for red alliance
   * @param mirror If true, mirror across centerline before flipping
   */
  public static GoToPoseCommand goTo(
      SwerveDrivetrain<?, ?, ?> drivetrain,
      Pose2d goal,
      BooleanSupplier shouldFlip,
      boolean mirror) {
    return new GoToPoseCommand(drivetrain, defaultConfig(), () -> goal, shouldFlip, mirror);
  }

  /** Go to a dynamic blue-origin pose with alliance flipping. */
  public static GoToPoseCommand goTo(
      SwerveDrivetrain<?, ?, ?> drivetrain, Supplier<Pose2d> goal, BooleanSupplier shouldFlip) {
    return new GoToPoseCommand(drivetrain, defaultConfig(), goal, shouldFlip, false);
  }

  // ── Fast transit with flip ─────────────────────────────────────────

  /** Fast transit to a blue-origin pose with alliance flipping. */
  public static GoToPoseCommand fastTransit(
      SwerveDrivetrain<?, ?, ?> drivetrain, Pose2d goal, BooleanSupplier shouldFlip) {
    return new GoToPoseCommand(drivetrain, fastTransitConfig(), () -> goal, shouldFlip, false);
  }

  /** Fast transit with alliance flip and optional mirror. */
  public static GoToPoseCommand fastTransit(
      SwerveDrivetrain<?, ?, ?> drivetrain,
      Pose2d goal,
      BooleanSupplier shouldFlip,
      boolean mirror) {
    return new GoToPoseCommand(drivetrain, fastTransitConfig(), () -> goal, shouldFlip, mirror);
  }

  /** Fast transit to a fixed pose. No flipping. */
  public static GoToPoseCommand fastTransit(SwerveDrivetrain<?, ?, ?> drivetrain, Pose2d goal) {
    return new GoToPoseCommand(drivetrain, fastTransitConfig(), goal);
  }

  // ── Precise placement with flip ────────────────────────────────────

  /** Precise placement at a blue-origin pose with alliance flipping. */
  public static GoToPoseCommand precisePlacement(
      SwerveDrivetrain<?, ?, ?> drivetrain, Pose2d goal, BooleanSupplier shouldFlip) {
    return new GoToPoseCommand(drivetrain, precisePlacementConfig(), () -> goal, shouldFlip, false);
  }

  /** Precise placement with alliance flip and optional mirror. */
  public static GoToPoseCommand precisePlacement(
      SwerveDrivetrain<?, ?, ?> drivetrain,
      Pose2d goal,
      BooleanSupplier shouldFlip,
      boolean mirror) {
    return new GoToPoseCommand(
        drivetrain, precisePlacementConfig(), () -> goal, shouldFlip, mirror);
  }

  /** Precise placement at a dynamic pose with alliance flipping. */
  public static GoToPoseCommand precisePlacement(
      SwerveDrivetrain<?, ?, ?> drivetrain, Supplier<Pose2d> goal, BooleanSupplier shouldFlip) {
    return new GoToPoseCommand(drivetrain, precisePlacementConfig(), goal, shouldFlip, false);
  }

  /** Precise placement at a fixed pose. No flipping. */
  public static GoToPoseCommand precisePlacement(
      SwerveDrivetrain<?, ?, ?> drivetrain, Pose2d goal) {
    return new GoToPoseCommand(drivetrain, precisePlacementConfig(), goal);
  }

  /** Precise placement at a dynamic pose. No flipping. */
  public static GoToPoseCommand precisePlacement(
      SwerveDrivetrain<?, ?, ?> drivetrain, Supplier<Pose2d> goal) {
    return new GoToPoseCommand(drivetrain, precisePlacementConfig(), goal);
  }

  // ── Convenience methods ────────────────────────────────────────────

  /**
   * Go to just a translation, keeping the current heading. Heading is captured at command init
   * time.
   */
  public static GoToPoseCommand goToTranslation(
      SwerveDrivetrain<?, ?, ?> drivetrain,
      double xMeters,
      double yMeters,
      BooleanSupplier shouldFlip) {
    return new GoToPoseCommand(
        drivetrain,
        defaultConfig(),
        () -> new Pose2d(xMeters, yMeters, drivetrain.getState().Pose.getRotation()),
        shouldFlip,
        false);
  }

  /**
   * Rotate in place to a target heading. Translation is captured at command init time. No flipping
   * applied to position since it uses current pose.
   */
  public static GoToPoseCommand rotateTo(
      SwerveDrivetrain<?, ?, ?> drivetrain, Rotation2d targetHeading) {
    return new GoToPoseCommand(
        drivetrain,
        defaultConfig(),
        () -> new Pose2d(drivetrain.getState().Pose.getTranslation(), targetHeading));
  }

  // ── Full custom ────────────────────────────────────────────────────

  /**
   * Full custom command with all parameters.
   *
   * @param drivetrain The drivetrain
   * @param config Custom motion config
   * @param goal Blue-origin target pose supplier
   * @param shouldFlip Alliance flip supplier
   * @param mirror Mirror across centerline
   */
  public static GoToPoseCommand custom(
      SwerveDrivetrain<?, ?, ?> drivetrain,
      GoToPoseCommand.Config config,
      Supplier<Pose2d> goal,
      BooleanSupplier shouldFlip,
      boolean mirror) {
    return new GoToPoseCommand(drivetrain, config, goal, shouldFlip, mirror);
  }
}
