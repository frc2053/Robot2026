// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.Supplier;

/** Helper class for tracking 2026 game state including shift phases and scoring windows. */
public class GameState {
  /** The different phases during a match. */
  public enum ShiftPhase {
    DISABLED,
    AUTO,
    TRANSITION,
    SHIFT_1,
    SHIFT_2,
    SHIFT_3,
    SHIFT_4,
    END_GAME
  }

  /** The robot's current mode. */
  public enum RobotMode {
    DISABLED,
    AUTO,
    TELEOP,
    TEST
  }

  // Shift timing constants (seconds from teleop start)
  private static final double kTransitionEnd = 10.0;
  private static final double kShift1End = 35.0;
  private static final double kShift2End = 60.0;
  private static final double kShift3End = 85.0;
  private static final double kShift4End = 110.0;
  private static final double kEndGameEnd = 140.0;

  // Grace period after hub deactivation where FUEL still counts
  private static final double kGracePeriodSeconds = 3.0;

  private final Timer m_teleopTimer = new Timer();
  private final Supplier<Pose2d> m_poseSupplier;
  private RobotMode m_robotMode = RobotMode.DISABLED;
  private ShiftPhase m_currentPhase = ShiftPhase.DISABLED;
  private ShiftPhase m_previousPhase = ShiftPhase.DISABLED;
  private boolean m_hubActiveRaw = true;
  private boolean m_isCounting = true;
  private boolean m_canScore = true;
  private double m_deactivationTimestamp = -1.0;
  private double m_timeUntilCounting;
  private double m_timeUntilNotCounting = Double.MAX_VALUE;
  private String m_gameData = "";

  // Triggers
  private final Trigger m_isCountingTrigger;
  private final Trigger m_canScoreTrigger;
  private final Trigger m_phaseChangedTrigger;
  private final Trigger m_wonAutoTrigger;

  // NetworkTables publishers
  private final NetworkTable m_table;
  private final StringPublisher m_phasePub;
  private final BooleanPublisher m_hubActivePub;
  private final BooleanPublisher m_isCountingPub;
  private final BooleanPublisher m_canScorePub;
  private final StringPublisher m_gameDataPub;
  private final BooleanPublisher m_onBluePub;
  private final DoublePublisher m_teleopTimerPub;
  private final DoublePublisher m_matchTimePub;
  private final DoublePublisher m_distanceToHubPub;
  private final DoublePublisher m_timeOfFlightPub;
  private final DoublePublisher m_timeUntilCountingPub;
  private final DoublePublisher m_timeUntilNotCountingPub;

  /**
   * Creates a new GameState instance.
   *
   * @param poseSupplier supplier for the robot's current pose
   */
  public GameState(Supplier<Pose2d> poseSupplier) {
    m_poseSupplier = poseSupplier;
    m_table = NetworkTableInstance.getDefault().getTable("GameState");
    m_phasePub = m_table.getStringTopic("Phase").publish();
    m_hubActivePub = m_table.getBooleanTopic("HubActive").publish();
    m_isCountingPub = m_table.getBooleanTopic("IsCounting").publish();
    m_canScorePub = m_table.getBooleanTopic("CanScore").publish();
    m_gameDataPub = m_table.getStringTopic("GameData").publish();
    m_onBluePub = m_table.getBooleanTopic("OnBlue").publish();
    m_teleopTimerPub = m_table.getDoubleTopic("TeleopTimer").publish();
    m_matchTimePub = m_table.getDoubleTopic("MatchTime").publish();
    m_distanceToHubPub = m_table.getDoubleTopic("DistanceToHub").publish();
    m_timeOfFlightPub = m_table.getDoubleTopic("TimeOfFlight").publish();
    m_timeUntilCountingPub = m_table.getDoubleTopic("TimeUntilCounting").publish();
    m_timeUntilNotCountingPub = m_table.getDoubleTopic("TimeUntilNotCounting").publish();

    // Initialize triggers
    m_isCountingTrigger = new Trigger(() -> m_isCounting);
    m_canScoreTrigger = new Trigger(() -> m_canScore);
    m_phaseChangedTrigger = new Trigger(() -> m_currentPhase != m_previousPhase);
    m_wonAutoTrigger = new Trigger(this::didWeWinAuto);
  }

  /** Starts the teleop timer and sets mode. Call this in teleopInit. */
  public void startTeleop() {
    m_robotMode = RobotMode.TELEOP;
    m_teleopTimer.restart();
  }

  /** Sets mode to auto. Call this in autonomousInit. */
  public void startAuto() {
    m_robotMode = RobotMode.AUTO;
    m_currentPhase = ShiftPhase.AUTO;
    m_hubActiveRaw = true;
    m_isCounting = true;
    m_deactivationTimestamp = -1.0;
  }

  /** Sets mode to disabled. Call this in disabledInit. */
  public void onDisabled() {
    m_robotMode = RobotMode.DISABLED;
    m_teleopTimer.stop();
    m_currentPhase = ShiftPhase.DISABLED;
  }

  /** Sets mode to test. Call this in testInit. */
  public void startTest() {
    m_robotMode = RobotMode.TEST;
  }

  /** Updates the game state. Call this in robotPeriodic. */
  public void periodic() {
    // Update game data
    String newGameData = DriverStation.getGameSpecificMessage();
    if (newGameData.length() > 0) {
      m_gameData = newGameData;
    }

    // Update phase based on mode and timer
    if (m_robotMode == RobotMode.TELEOP) {
      m_previousPhase = m_currentPhase;
      m_currentPhase = calculatePhase(m_teleopTimer.get());

      // Calculate hub active state
      boolean wasActive = m_hubActiveRaw;
      m_hubActiveRaw = isHubActiveInPhase(m_currentPhase);

      // Track deactivation for grace period
      if (wasActive && !m_hubActiveRaw) {
        m_deactivationTimestamp = m_teleopTimer.get();
      }

      // Hub is counting if active OR within grace period after deactivation
      if (m_hubActiveRaw) {
        m_isCounting = true;
        m_deactivationTimestamp = -1.0;
        m_timeUntilCounting = 0.0;
        m_timeUntilNotCounting = calculateTimeUntilNotCounting();
      } else if (m_deactivationTimestamp > 0) {
        double timeSinceDeactivation = m_teleopTimer.get() - m_deactivationTimestamp;
        m_isCounting = timeSinceDeactivation <= kGracePeriodSeconds;
        m_timeUntilCounting = 0.0;
        m_timeUntilNotCounting = kGracePeriodSeconds - timeSinceDeactivation;
      } else {
        m_isCounting = false;
        m_timeUntilCounting = calculateTimeUntilCounting();
        m_timeUntilNotCounting = m_timeUntilCounting + calculateNextCountingWindowDuration();
      }
    } else if (m_robotMode == RobotMode.AUTO) {
      m_currentPhase = ShiftPhase.AUTO;
      m_hubActiveRaw = true;
      m_isCounting = true;
      m_timeUntilNotCounting = Double.MAX_VALUE;
    }

    // Calculate lookup distance and time of flight for canScore
    double lookupDistance = calculateLookupDistance();
    double timeOfFlight = ShooterConstants.TIME_OF_FLIGHT_MAP.get(lookupDistance);

    // Can score if the ball will arrive during a counting window
    // This handles both: shooting while counting, AND shooting early before hub activates
    m_canScore = timeOfFlight >= m_timeUntilCounting && timeOfFlight < m_timeUntilNotCounting;

    // Publish to NetworkTables
    m_phasePub.set(m_currentPhase.name());
    m_hubActivePub.set(m_hubActiveRaw);
    m_isCountingPub.set(m_isCounting);
    m_canScorePub.set(m_canScore);
    m_gameDataPub.set(m_gameData);
    m_onBluePub.set(Constants.ifOnBlue());
    m_teleopTimerPub.set(m_teleopTimer.get());
    m_matchTimePub.set(DriverStation.getMatchTime());
    m_distanceToHubPub.set(lookupDistance);
    m_timeOfFlightPub.set(timeOfFlight);
    m_timeUntilCountingPub.set(m_timeUntilCounting);
    m_timeUntilNotCountingPub.set(m_timeUntilNotCounting);
  }

  /**
   * Calculates the current shift phase based on elapsed teleop time.
   *
   * @param elapsedSeconds seconds since teleop started
   * @return the current ShiftPhase
   */
  private ShiftPhase calculatePhase(double elapsedSeconds) {
    if (elapsedSeconds < kTransitionEnd) {
      return ShiftPhase.TRANSITION;
    } else if (elapsedSeconds < kShift1End) {
      return ShiftPhase.SHIFT_1;
    } else if (elapsedSeconds < kShift2End) {
      return ShiftPhase.SHIFT_2;
    } else if (elapsedSeconds < kShift3End) {
      return ShiftPhase.SHIFT_3;
    } else if (elapsedSeconds < kShift4End) {
      return ShiftPhase.SHIFT_4;
    } else if (elapsedSeconds < kEndGameEnd) {
      return ShiftPhase.END_GAME;
    }
    return ShiftPhase.END_GAME;
  }

  /**
   * Determines if our alliance's hub is active during the given phase.
   *
   * @param phase the shift phase to check
   * @return true if our hub is active (not counting grace period)
   */
  private boolean isHubActiveInPhase(ShiftPhase phase) {
    // Both alliances can always score during these phases
    if (phase == ShiftPhase.AUTO
        || phase == ShiftPhase.TRANSITION
        || phase == ShiftPhase.END_GAME
        || phase == ShiftPhase.DISABLED) {
      return true;
    }

    // No game data yet - assume we can score
    if (m_gameData.isEmpty()) {
      return true;
    }

    // Determine if we won auto (game data matches our alliance)
    boolean weWonAuto = didWeWinAuto();

    // If we won auto: our hub goes inactive FIRST, so we're active in SHIFT_2 and SHIFT_4
    // If we lost auto: our hub goes inactive SECOND, so we're active in SHIFT_1 and SHIFT_3
    if (weWonAuto) {
      return phase == ShiftPhase.SHIFT_2 || phase == ShiftPhase.SHIFT_4;
    } else {
      return phase == ShiftPhase.SHIFT_1 || phase == ShiftPhase.SHIFT_3;
    }
  }

  /**
   * Returns whether our alliance won the auto period (scored more FUEL).
   *
   * @return true if our alliance won auto
   */
  private boolean didWeWinAuto() {
    if (m_gameData.isEmpty()) {
      return false;
    }
    char dataChar = m_gameData.charAt(0);
    if (Constants.ifOnBlue()) {
      return dataChar == 'B';
    } else {
      return dataChar == 'R';
    }
  }

  /**
   * Calculates the lookup-table distance (front-of-bumper to front-of-hub) with alliance fudge
   * factor.
   *
   * @return distance in meters in the lookup table reference frame
   */
  private double calculateLookupDistance() {
    Translation2d robotPosition = m_poseSupplier.get().getTranslation();
    double centerToCenter = robotPosition.getDistance(Constants.FieldSpots.getHubPosition());
    double tableKey =
        centerToCenter
            - Constants.FuelConstants.kLookupTableDistanceOffset
            - Constants.FuelConstants.getAllianceLookupOffset();
    return Math.max(0.0, tableKey);
  }

  /**
   * Calculates how many seconds until the hub stops counting.
   *
   * @return seconds until counting stops, or MAX_VALUE if counting indefinitely
   */
  private double calculateTimeUntilNotCounting() {
    double currentTime = m_teleopTimer.get();
    boolean weWonAuto = didWeWinAuto();

    // Find the next phase boundary where our hub becomes inactive
    // Won auto: active in SHIFT_2, SHIFT_4 (inactive in SHIFT_1, SHIFT_3)
    // Lost auto: active in SHIFT_1, SHIFT_3 (inactive in SHIFT_2, SHIFT_4)
    double nextInactiveTime;
    if (weWonAuto) {
      // We go inactive at: SHIFT_1 start (10s), SHIFT_3 start (60s)
      if (currentTime < kTransitionEnd) {
        nextInactiveTime = kTransitionEnd;
      } else if (currentTime < kShift2End) {
        nextInactiveTime = kShift2End;
      } else if (currentTime < kShift4End) {
        nextInactiveTime = kShift4End;
      } else {
        return Double.MAX_VALUE; // END_GAME, always active
      }
    } else {
      // We go inactive at: SHIFT_2 start (35s), SHIFT_4 start (85s)
      if (currentTime < kShift1End) {
        nextInactiveTime = kShift1End;
      } else if (currentTime < kShift3End) {
        nextInactiveTime = kShift3End;
      } else {
        return Double.MAX_VALUE; // END_GAME, always active
      }
    }

    // Add grace period to the time until inactive
    return (nextInactiveTime - currentTime) + kGracePeriodSeconds;
  }

  /**
   * Calculates how many seconds until the hub starts counting (next active window). Only called
   * when hub is currently not counting.
   *
   * @return seconds until counting starts
   */
  private double calculateTimeUntilCounting() {
    double currentTime = m_teleopTimer.get();
    boolean weWonAuto = didWeWinAuto();

    // Find the next phase boundary where our hub becomes active
    // Won auto: active in SHIFT_2, SHIFT_4 (starts at 35s, 85s)
    // Lost auto: active in SHIFT_1, SHIFT_3 (starts at 10s, 60s)
    if (weWonAuto) {
      if (currentTime < kShift1End) {
        return kShift1End - currentTime; // Next active at SHIFT_2 start (35s)
      } else if (currentTime < kShift3End) {
        return kShift3End - currentTime; // Next active at SHIFT_4 start (85s)
      }
    } else {
      if (currentTime < kTransitionEnd) {
        return kTransitionEnd - currentTime; // Next active at SHIFT_1 start (10s)
      } else if (currentTime < kShift2End) {
        return kShift2End - currentTime; // Next active at SHIFT_3 start (60s)
      }
    }
    // Should not reach here during normal play
    return Double.MAX_VALUE;
  }

  /**
   * Calculates the duration of the next counting window (including grace period). Only called when
   * hub is currently not counting.
   *
   * @return duration of next counting window in seconds
   */
  private double calculateNextCountingWindowDuration() {
    double currentTime = m_teleopTimer.get();
    boolean weWonAuto = didWeWinAuto();

    // Each shift is 25 seconds, plus 3 second grace period = 28 seconds of counting
    // END_GAME is 30 seconds and always active
    double shiftDuration = 25.0;

    if (weWonAuto) {
      // Active in SHIFT_2 (35-60s), SHIFT_4 (85-110s), END_GAME (110-140s)
      if (currentTime < kShift1End) {
        return shiftDuration + kGracePeriodSeconds; // SHIFT_2 duration
      } else if (currentTime < kShift3End) {
        // SHIFT_4 leads into END_GAME, so effectively unlimited
        return Double.MAX_VALUE;
      }
    } else {
      // Active in SHIFT_1 (10-35s), SHIFT_3 (60-85s), END_GAME (110-140s)
      if (currentTime < kTransitionEnd) {
        return shiftDuration + kGracePeriodSeconds; // SHIFT_1 duration
      } else if (currentTime < kShift2End) {
        return shiftDuration + kGracePeriodSeconds; // SHIFT_3 duration
      }
    }
    return Double.MAX_VALUE;
  }

  // ==================== TRIGGERS ====================

  /**
   * Trigger that fires when hub is counting (hub active or in 3s grace period).
   *
   * @return Trigger for counting state
   */
  public Trigger isCounting() {
    return m_isCountingTrigger;
  }

  /**
   * Trigger that fires when we should shoot, accounting for ball travel time. True when the ball
   * will arrive at the hub before counting stops.
   *
   * @return Trigger for can score state
   */
  public Trigger canScore() {
    return m_canScoreTrigger;
  }

  /**
   * Trigger that fires when the phase changes.
   *
   * @return Trigger for phase change
   */
  public Trigger phaseChanged() {
    return m_phaseChangedTrigger;
  }

  /**
   * Trigger that fires when our alliance won auto.
   *
   * @return Trigger for auto win
   */
  public Trigger wonAuto() {
    return m_wonAutoTrigger;
  }

  /**
   * Trigger that fires when we are in a specific phase.
   *
   * @param phase the phase to check
   * @return Trigger for the specified phase
   */
  public Trigger inPhase(ShiftPhase phase) {
    return new Trigger(() -> m_currentPhase == phase);
  }

  // ==================== GETTERS ====================

  /**
   * Returns the current shift phase.
   *
   * @return the current ShiftPhase
   */
  public ShiftPhase getCurrentPhase() {
    return m_currentPhase;
  }

  /**
   * Returns the current game data string.
   *
   * @return the game data ('R', 'B', or empty string)
   */
  public String getGameData() {
    return m_gameData;
  }

  /**
   * Returns the elapsed teleop time in seconds.
   *
   * @return elapsed time since teleop started
   */
  public double getTeleopTime() {
    return m_teleopTimer.get();
  }

  /**
   * Returns the current robot mode.
   *
   * @return the RobotMode
   */
  public RobotMode getRobotMode() {
    return m_robotMode;
  }
}
