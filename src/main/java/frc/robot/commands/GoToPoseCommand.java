package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A go-to-pose command for CTRE swerve drivetrains that uses pose-based termination instead of
 * time-based termination.
 *
 * <p>This command uses CTRE's {@link LinearPath} for trapezoidal motion profiling along a straight
 * line, and {@link WheelForceCalculator} to compute per-wheel force feedforwards. The command
 * terminates when the robot's actual pose converges to the goal within configurable tolerances, NOT
 * when the profile's time expires.
 *
 * <h2>Why pose-based termination?</h2>
 *
 * <p>Time-based paths assume perfect tracking. If the robot slips, collides, or otherwise falls
 * behind the profile, a time-based command will finish before the robot reaches the goal.
 * Pose-based termination guarantees the robot is actually at the target before the command ends.
 *
 * <h2>Usage:</h2>
 *
 * <pre>{@code
 * GoToPoseCommand cmd = new GoToPoseCommand(
 *     drivetrain,
 *     new GoToPoseCommand.Config()
 *         .withLinearConstraints(4.0, 3.0)    // m/s, m/s^2
 *         .withAngularConstraints(2 * Math.PI, Math.PI)  // rad/s, rad/s^2
 *         .withMass(55.0)                     // kg
 *         .withMOI(6.0)                       // kg*m^2
 *         .withLinearTolerance(0.02)           // meters
 *         .withAngularTolerance(Math.toRadians(1.0)), // radians
 *     new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90))
 * );
 * }</pre>
 */
public class GoToPoseCommand extends Command {

  // ── Configuration ──────────────────────────────────────────────────

  /**
   * Immutable-style builder for GoToPoseCommand parameters. All with* methods return {@code this}
   * for chaining.
   */
  public static class Config {
    /** Max linear velocity in m/s. */
    public double m_maxLinearVelocity = 4.0;

    /** Max linear acceleration in m/s^2. */
    public double m_maxLinearAcceleration = 3.0;

    /** Max angular velocity in rad/s. */
    public double m_maxAngularVelocity = 2 * Math.PI;

    /** Max angular acceleration in rad/s^2. */
    public double m_maxAngularAcceleration = Math.PI;

    /** Robot mass in kg (used for force feedforwards). */
    public double m_massKg = 55.0;

    /** Robot moment of inertia in kg*m^2 (used for force feedforwards). */
    public double m_moiKgM2 = 6.0;

    /** Translational pose tolerance in meters for end condition. */
    public double m_linearToleranceMeters = 0.02;

    /** Rotational pose tolerance in radians for end condition. */
    public double m_angularToleranceRadians = Math.toRadians(1.0);

    /** Velocity tolerance in m/s — robot must also be near-stopped. */
    public double m_velocityToleranceMps = 0.05;

    /** Angular velocity tolerance in rad/s. */
    public double m_angularVelocityToleranceRps = Math.toRadians(5.0);

    /**
     * Number of consecutive cycles the robot must be within tolerance before the command ends.
     * Prevents premature exit on brief passthrough.
     */
    public int m_settleCycles = 5;

    /**
     * Safety timeout in seconds. If the robot hasn't converged by this time, the command ends
     * anyway. Set to 0 to disable.
     */
    public double m_timeoutSeconds;

    public Config withLinearConstraints(double maxVel, double maxAccel) {
      m_maxLinearVelocity = maxVel;
      m_maxLinearAcceleration = maxAccel;
      return this;
    }

    public Config withAngularConstraints(double maxVel, double maxAccel) {
      m_maxAngularVelocity = maxVel;
      m_maxAngularAcceleration = maxAccel;
      return this;
    }

    public Config withMass(double massKg) {
      m_massKg = massKg;
      return this;
    }

    public Config withMOI(double moiKgM2) {
      m_moiKgM2 = moiKgM2;
      return this;
    }

    public Config withLinearTolerance(double meters) {
      m_linearToleranceMeters = meters;
      return this;
    }

    public Config withAngularTolerance(double radians) {
      m_angularToleranceRadians = radians;
      return this;
    }

    public Config withVelocityTolerance(double mps) {
      m_velocityToleranceMps = mps;
      return this;
    }

    public Config withAngularVelocityTolerance(double rps) {
      m_angularVelocityToleranceRps = rps;
      return this;
    }

    public Config withSettleCycles(int cycles) {
      m_settleCycles = cycles;
      return this;
    }

    public Config withTimeout(double seconds) {
      m_timeoutSeconds = seconds;
      return this;
    }
  }

  // ── Fields ─────────────────────────────────────────────────────────

  private final SwerveDrivetrain<?, ?, ?> m_drivetrain;
  private final Config m_config;
  private final Supplier<Pose2d> m_goalSupplier;
  private final BooleanSupplier m_shouldFlip;
  private final boolean m_mirror;

  private final LinearPath m_path;
  private final WheelForceCalculator m_forceCalc;
  private final SwerveRequest.ApplyFieldSpeeds m_request;

  private LinearPath.State m_currentState;
  private ChassisSpeeds m_previousSpeeds;
  private Pose2d m_goal;

  private final Timer m_timer = new Timer();
  private double m_previousTimestamp;
  private int m_settleCounter;

  // ── Constructors ───────────────────────────────────────────────────

  /**
   * Creates a GoToPoseCommand with a fixed goal pose. No alliance flipping or mirroring is applied.
   *
   * @param drivetrain The CTRE SwerveDrivetrain subsystem
   * @param config Motion and tolerance configuration
   * @param goal The target field-relative pose
   */
  public GoToPoseCommand(SwerveDrivetrain<?, ?, ?> drivetrain, Config config, Pose2d goal) {
    this(drivetrain, config, () -> goal, () -> false, false);
  }

  /**
   * Creates a GoToPoseCommand with a dynamic goal pose supplier. No alliance flipping or mirroring
   * is applied.
   *
   * @param drivetrain The CTRE SwerveDrivetrain subsystem
   * @param config Motion and tolerance configuration
   * @param goalSupplier Supplier that provides the target pose at init time
   */
  public GoToPoseCommand(
      SwerveDrivetrain<?, ?, ?> drivetrain, Config config, Supplier<Pose2d> goalSupplier) {
    this(drivetrain, config, goalSupplier, () -> false, false);
  }

  /**
   * Full constructor with alliance flipping and mirroring support.
   *
   * <p>Alliance flipping and mirroring use PathPlanner's {@link FlippingUtil}, which supports both
   * rotational ({@code kRotational}) and mirrored ({@code kMirrored}) field symmetry. Make sure to
   * configure {@code FlippingUtil.symmetryType} and the field size fields in your robot init —
   * PathPlanner typically does this automatically if you use AutoBuilder, but if not:
   *
   * <pre>{@code
   * FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
   * FlippingUtil.fieldSizeX = 16.54;  // meters
   * FlippingUtil.fieldSizeY = 8.21;   // meters
   * }</pre>
   *
   * <h3>Flip vs Mirror</h3>
   *
   * <ul>
   *   <li><b>Alliance flip</b> ({@code shouldFlip}): Transforms a blue-origin pose to the red side
   *       using the configured field symmetry (rotational or mirrored). Evaluated at command init
   *       time. Typical usage: {@code () -> isRedAlliance()}.
   *   <li><b>Mirror</b>: Reflects the pose across the field centerline (Y = fieldWidth/2). Applied
   *       <i>before</i> the alliance flip. Useful for selecting between symmetric positions on the
   *       same alliance side (e.g., left vs right scoring column).
   * </ul>
   *
   * <p>The transform order is: <b>mirror first, then alliance flip</b>. This means you author poses
   * for blue-left, set mirror=true to get blue-right, and the alliance flip handles red
   * automatically.
   *
   * @param drivetrain The CTRE SwerveDrivetrain subsystem
   * @param config Motion and tolerance configuration
   * @param goalSupplier Supplier that provides the target pose (blue-origin)
   * @param shouldFlip Returns true when the pose should be alliance-flipped (typically {@code () ->
   *     isRedAlliance()})
   * @param mirror If true, mirror the pose across the field centerline before applying the alliance
   *     flip
   */
  public GoToPoseCommand(
      SwerveDrivetrain<?, ?, ?> drivetrain,
      Config config,
      Supplier<Pose2d> goalSupplier,
      BooleanSupplier shouldFlip,
      boolean mirror) {
    m_drivetrain = drivetrain;
    m_config = config;
    m_goalSupplier = goalSupplier;
    m_shouldFlip = shouldFlip;
    m_mirror = mirror;

    // Build the CTRE LinearPath with trapezoidal constraints
    m_path =
        new LinearPath(
            new TrapezoidProfile.Constraints(
                config.m_maxLinearVelocity, config.m_maxLinearAcceleration),
            new TrapezoidProfile.Constraints(
                config.m_maxAngularVelocity, config.m_maxAngularAcceleration));

    // Build the wheel force calculator from drivetrain module locations
    Translation2d[] moduleLocations = getModuleLocations(drivetrain);
    m_forceCalc = new WheelForceCalculator(moduleLocations, config.m_massKg, config.m_moiKgM2);

    // Pre-allocate the swerve request
    m_request = new SwerveRequest.ApplyFieldSpeeds();

    // Add requirements if the drivetrain is a Subsystem (e.g., Swerve implements Subsystem)
    if (drivetrain instanceof Subsystem) {
      addRequirements((Subsystem) drivetrain);
    }
  }

  // ── Command lifecycle ──────────────────────────────────────────────

  @Override
  public void initialize() {
    // ── Resolve the goal pose with transforms ─────────────────────
    // Order: get raw pose → mirror (if enabled) → alliance flip (if enabled)
    Pose2d rawGoal = m_goalSupplier.get();

    if (m_mirror) {
      rawGoal = mirrorAcrossCenterline(rawGoal);
    }
    if (m_shouldFlip.getAsBoolean()) {
      rawGoal = FlippingUtil.flipFieldPose(rawGoal);
    }

    m_goal = rawGoal;

    // Seed the path state from current odometry
    Pose2d currentPose = m_drivetrain.getState().Pose;
    ChassisSpeeds currentFieldSpeeds = fieldRelativeSpeeds();

    m_currentState = new LinearPath.State(currentPose, currentFieldSpeeds);
    m_previousSpeeds = currentFieldSpeeds;

    m_timer.restart();
    m_previousTimestamp = 0.0;
    m_settleCounter = 0;
  }

  @Override
  public void execute() {
    double now = m_timer.get();
    double dt = now - m_previousTimestamp;
    m_previousTimestamp = now;

    // Guard against zero or negative dt (first cycle, timer glitch)
    if (dt <= 1e-6) {
      dt = 0.02; // default to 20ms
    }

    // ── Step 1: Advance the motion profile ────────────────────────
    // LinearPath.calculate() takes time-since-previous-update, the
    // current profiled state, and the goal. It returns the new profiled
    // state (pose + field-centric speeds).
    m_currentState = m_path.calculate(dt, m_currentState, m_goal);

    ChassisSpeeds profiledSpeeds = m_currentState.speeds;

    // ── Step 2: Compute wheel force feedforwards ──────────────────
    // Use the dt-based overload: it differentiates speeds for us.
    WheelForceCalculator.Feedforwards ff =
        m_forceCalc.calculate(dt, m_previousSpeeds, profiledSpeeds);

    m_previousSpeeds = profiledSpeeds;

    // ── Step 3: Apply to the drivetrain ───────────────────────────
    m_drivetrain.setControl(
        m_request
            .withSpeeds(profiledSpeeds)
            .withWheelForceFeedforwardsX(ff.x_newtons)
            .withWheelForceFeedforwardsY(ff.y_newtons));
  }

  @Override
  public boolean isFinished() {
    // ── Timeout check ─────────────────────────────────────────────
    if (m_config.m_timeoutSeconds > 0 && m_timer.hasElapsed(m_config.m_timeoutSeconds)) {
      return true;
    }

    // ── Pose convergence check ────────────────────────────────────
    Pose2d currentPose = m_drivetrain.getState().Pose;

    double translationError = currentPose.getTranslation().getDistance(m_goal.getTranslation());
    double rotationError =
        Math.abs(currentPose.getRotation().minus(m_goal.getRotation()).getRadians());

    ChassisSpeeds currentSpeeds = fieldRelativeSpeeds();
    double linearSpeed =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    double angularSpeed = Math.abs(currentSpeeds.omegaRadiansPerSecond);

    boolean withinTolerance =
        translationError < m_config.m_linearToleranceMeters
            && rotationError < m_config.m_angularToleranceRadians
            && linearSpeed < m_config.m_velocityToleranceMps
            && angularSpeed < m_config.m_angularVelocityToleranceRps;

    if (withinTolerance) {
      m_settleCounter++;
    } else {
      m_settleCounter = 0;
    }

    return m_settleCounter >= m_config.m_settleCycles;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.setControl(new SwerveRequest.Idle());
    m_timer.stop();
  }

  // ── Helpers ────────────────────────────────────────────────────────

  /**
   * Gets the current field-relative chassis speeds from drivetrain state. CTRE's
   * SwerveDriveState.Speeds is robot-relative, so we rotate it into the field frame using the
   * current heading.
   */
  private ChassisSpeeds fieldRelativeSpeeds() {
    var state = m_drivetrain.getState();
    return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
  }

  /**
   * Mirrors a pose across the field centerline (Y = fieldWidth / 2). X and heading-X are preserved;
   * Y is reflected and heading is negated.
   *
   * <p>Uses PathPlanner's {@link FlippingUtil#fieldSizeY} for the field width, so make sure it's
   * configured correctly.
   *
   * <pre>
   *     Original:  (x, y, θ)
   *     Mirrored:  (x, fieldWidth - y, -θ)
   * </pre>
   */
  private static Pose2d mirrorAcrossCenterline(Pose2d pose) {
    return new Pose2d(
        pose.getX(), FlippingUtil.fieldSizeY - pose.getY(), pose.getRotation().unaryMinus());
  }

  /**
   * Convenience supplier for alliance flipping. Returns true when the robot is on the red alliance.
   *
   * <p>Usage: pass {@code GoToPoseCommand::isRedAlliance} as the {@code shouldFlip} parameter.
   */
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  /**
   * Extracts module locations from the drivetrain. This uses the module positions from the
   * drivetrain's configuration.
   */
  private static Translation2d[] getModuleLocations(SwerveDrivetrain<?, ?, ?> drivetrain) {
    var modules = drivetrain.getModuleLocations();
    return modules;
  }
}
