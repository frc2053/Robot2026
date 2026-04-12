package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Collections;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A resilient path-following command that pauses the trajectory timer when the robot drifts too far
 * off the planned path, drives back to the expected pose, then resumes.
 *
 * <p>Normal PathPlanner path following is purely time-based: the trajectory advances regardless of
 * where the robot actually is. If the robot gets bumped or stuck, the trajectory finishes before
 * the robot reaches the goal. This command fixes that by managing a "virtual time" that only
 * advances when the robot is close enough to the trajectory.
 *
 * <p>When the translational error exceeds {@code pauseThresholdMeters}, the virtual time freezes
 * and the PID controller works to drive the robot back to the paused target pose. Once the error
 * drops below {@code resumeThresholdMeters}, virtual time resumes advancing and normal path
 * following continues.
 *
 * <p>This is a drop-in replacement for {@code FollowPathCommand} in the AutoBuilder configuration.
 */
public class ResilientFollowPathCommand extends Command {

  // ── Configuration ────────────────────────────────────────────────

  /** Distance from the trajectory pose at which we pause time (meters). */
  private final double m_pauseThresholdMeters;

  /** Distance at which we resume time after being paused (meters). */
  private final double m_resumeThresholdMeters;

  // ── Path following fields (mirrors FollowPathCommand) ────────────

  private final PathPlannerPath m_originalPath;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Supplier<ChassisSpeeds> m_speedsSupplier;
  private final BiConsumer<ChassisSpeeds, DriveFeedforwards> m_output;
  private final PathFollowingController m_controller;
  private final RobotConfig m_robotConfig;
  private final BooleanSupplier m_shouldFlipPath;
  private final EventScheduler m_eventScheduler;

  private PathPlannerPath m_path;
  private PathPlannerTrajectory m_trajectory;

  // ── Virtual time management ──────────────────────────────────────

  private static final double kGracePeriodSeconds = 0.25;
  private static final double kPauseTimeoutSeconds = 2.0;

  private final Timer m_wallTimer = new Timer();
  private final Timer m_pauseTimer = new Timer();
  private double m_virtualTime;
  private double m_lastWallTime;
  private boolean m_isPaused;

  // ── Telemetry ────────────────────────────────────────────────────

  private final Alert m_pausedAlert =
      new Alert("Auto path paused: robot off trajectory", AlertType.kWarning);

  private final DoublePublisher m_errorPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("ResilientPath/TranslationErrorMeters")
          .publish();
  private final BooleanPublisher m_pausedPub =
      NetworkTableInstance.getDefault().getBooleanTopic("ResilientPath/IsPaused").publish();
  private final DoublePublisher m_virtualTimePub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("ResilientPath/VirtualTimeSeconds")
          .publish();
  private final DoublePublisher m_realErrorPub =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("ResilientPath/RealTrackingErrorMeters")
          .publish();
  private final StructPublisher<Pose2d> m_targetPosePub =
      NetworkTableInstance.getDefault()
          .getStructTopic("ResilientPath/TargetPose", Pose2d.struct)
          .publish();

  /**
   * Creates a resilient path-following command with custom thresholds.
   *
   * @param path The path to follow.
   * @param poseSupplier Supplier for the current field-relative robot pose.
   * @param speedsSupplier Supplier for the current robot-relative chassis speeds.
   * @param output Consumer that accepts robot-relative ChassisSpeeds and per-module feedforwards.
   * @param controller The path-following PID controller.
   * @param robotConfig The robot's physical configuration.
   * @param shouldFlipPath Whether to flip the path for the other alliance.
   * @param pauseThresholdMeters Error distance at which to pause the trajectory (meters).
   * @param resumeThresholdMeters Error distance at which to resume the trajectory (meters).
   * @param requirements Subsystems required by this command (typically the drivetrain).
   */
  public ResilientFollowPathCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      double pauseThresholdMeters,
      double resumeThresholdMeters,
      Subsystem... requirements) {
    m_originalPath = path;
    m_poseSupplier = poseSupplier;
    m_speedsSupplier = speedsSupplier;
    m_output = output;
    m_controller = controller;
    m_robotConfig = robotConfig;
    m_shouldFlipPath = shouldFlipPath;
    m_pauseThresholdMeters = pauseThresholdMeters;
    m_resumeThresholdMeters = resumeThresholdMeters;
    m_eventScheduler = new EventScheduler();

    Set<Subsystem> driveRequirements = Set.of(requirements);
    addRequirements(requirements);

    var eventReqs = EventScheduler.getSchedulerRequirements(m_originalPath);
    if (!Collections.disjoint(driveRequirements, eventReqs)) {
      throw new IllegalArgumentException(
          "Events that are triggered during path following cannot require the drive subsystem");
    }
    addRequirements(eventReqs);

    m_path = m_originalPath;
    m_originalPath.getIdealTrajectory(m_robotConfig).ifPresent(traj -> m_trajectory = traj);
  }

  /**
   * Creates a resilient path-following command with default thresholds (0.5m pause, 0.15m resume).
   *
   * @param path The path to follow.
   * @param poseSupplier Supplier for the current field-relative robot pose.
   * @param speedsSupplier Supplier for the current robot-relative chassis speeds.
   * @param output Consumer that accepts robot-relative ChassisSpeeds and per-module feedforwards.
   * @param controller The path-following PID controller.
   * @param robotConfig The robot's physical configuration.
   * @param shouldFlipPath Whether to flip the path for the other alliance.
   * @param requirements Subsystems required by this command (typically the drivetrain).
   */
  public ResilientFollowPathCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem... requirements) {
    this(
        path,
        poseSupplier,
        speedsSupplier,
        output,
        controller,
        robotConfig,
        shouldFlipPath,
        0.5,
        0.15,
        requirements);
  }

  @Override
  public void initialize() {
    if (m_shouldFlipPath.getAsBoolean() && !m_originalPath.preventFlipping) {
      m_path = m_originalPath.flipPath();
    } else {
      m_path = m_originalPath;
    }

    Pose2d currentPose = m_poseSupplier.get();
    ChassisSpeeds currentSpeeds = m_speedsSupplier.get();

    m_controller.reset(currentPose, currentSpeeds);

    double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    if (m_path.getIdealStartingState() != null) {
      boolean idealVelocity =
          Math.abs(linearVel - m_path.getIdealStartingState().velocityMPS()) <= 0.25;
      boolean idealRotation =
          !m_robotConfig.isHolonomic
              || Math.abs(
                      currentPose
                          .getRotation()
                          .minus(m_path.getIdealStartingState().rotation())
                          .getDegrees())
                  <= 30.0;
      if (idealVelocity && idealRotation) {
        m_trajectory = m_path.getIdealTrajectory(m_robotConfig).orElseThrow();
      } else {
        m_trajectory =
            m_path.generateTrajectory(currentSpeeds, currentPose.getRotation(), m_robotConfig);
      }
    } else {
      m_trajectory =
          m_path.generateTrajectory(currentSpeeds, currentPose.getRotation(), m_robotConfig);
    }

    PathPlannerLogging.logActivePath(m_path);
    PPLibTelemetry.setCurrentPath(m_path);

    m_eventScheduler.initialize(m_trajectory);

    m_virtualTime = 0.0;
    m_isPaused = false;
    m_wallTimer.restart();
    m_lastWallTime = 0.0;

    m_pausedAlert.set(false);
  }

  @Override
  public void execute() {
    double wallNow = m_wallTimer.get();
    double wallDt = wallNow - m_lastWallTime;
    m_lastWallTime = wallNow;

    // Sample trajectory at current virtual time to get the target state
    PathPlannerTrajectoryState targetState = m_trajectory.sample(m_virtualTime);

    // Current robot pose from pose estimator (fuses odometry + vision)
    Pose2d currentPose = m_poseSupplier.get();
    double translationError =
        currentPose.getTranslation().getDistance(targetState.pose.getTranslation());

    // ── Pause / resume logic (skip during grace period after path start) ──
    if (!m_isPaused
        && translationError > m_pauseThresholdMeters
        && m_wallTimer.get() > kGracePeriodSeconds) {
      // Robot has drifted too far — freeze trajectory time
      m_isPaused = true;
      m_pauseTimer.restart();
      m_controller.reset(currentPose, m_speedsSupplier.get());
    } else if (m_isPaused
        && (translationError < m_resumeThresholdMeters
            || m_pauseTimer.hasElapsed(kPauseTimeoutSeconds))) {
      // Robot has returned close enough, or we've been stuck too long — resume
      m_isPaused = false;
      m_pauseTimer.stop();
      m_controller.reset(currentPose, m_speedsSupplier.get());
    }

    // Only advance virtual time when not paused, and clamp to trajectory duration
    if (!m_isPaused) {
      m_virtualTime = Math.min(m_virtualTime + wallDt, m_trajectory.getTotalTimeSeconds());
      // Re-sample with the advanced time
      targetState = m_trajectory.sample(m_virtualTime);
    }

    m_pausedAlert.set(m_isPaused);

    // ── Drive toward target (same as normal FollowPathCommand) ───
    if (!m_controller.isHolonomic() && m_path.isReversed()) {
      targetState = targetState.reverse();
    }

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);
    PPLibTelemetry.setTargetPose(targetState.pose);
    PathPlannerLogging.logTargetPose(targetState.pose);

    ChassisSpeeds currentSpeeds = m_speedsSupplier.get();
    ChassisSpeeds targetSpeeds =
        m_controller.calculateRobotRelativeSpeeds(currentPose, targetState);
    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    PPLibTelemetry.setVelocities(
        currentVel,
        targetState.linearVelocity,
        currentSpeeds.omegaRadiansPerSecond,
        targetSpeeds.omegaRadiansPerSecond);

    m_output.accept(targetSpeeds, targetState.feedforwards);

    // Only advance events when not paused
    if (!m_isPaused) {
      m_eventScheduler.execute(m_virtualTime);
    }

    // ── Publish telemetry ────────────────────────────────────────
    double realError = currentPose.getTranslation().getDistance(targetState.pose.getTranslation());
    m_errorPub.set(translationError);
    m_realErrorPub.set(realError);
    m_pausedPub.set(m_isPaused);
    m_virtualTimePub.set(m_virtualTime);
    m_targetPosePub.set(targetState.pose);
  }

  @Override
  public boolean isFinished() {
    double totalTime = m_trajectory.getTotalTimeSeconds();
    return m_virtualTime >= totalTime || !Double.isFinite(totalTime);
  }

  @Override
  public void end(boolean interrupted) {
    m_wallTimer.stop();
    m_pausedAlert.set(false);

    if (!interrupted && m_path.getGoalEndState().velocityMPS() < 0.1) {
      m_output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(m_robotConfig.numModules));
    }

    PathPlannerLogging.logActivePath(null);
    m_eventScheduler.end();
  }
}
