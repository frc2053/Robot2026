package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.ShootingDataPoint;
import java.util.ArrayList;
import java.util.List;

public final class Constants {

  /** Private constructor to prevent instantiation. */
  private Constants() {}

  /**
   * Returns true if the robot is on the blue alliance.
   *
   * @return true if on blue alliance, false otherwise.
   */
  public static boolean ifOnBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public static final class FieldSpots {
    private FieldSpots() {}

    public static final double kFieldLength = Units.inchesToMeters(651.25); // 54 ft 3.25 in
    public static final double kFieldWidth = Units.inchesToMeters(323.25); // 26 ft 11.25 in
    public static final Translation2d kBlueMiddleHub =
        new Translation2d(Units.inchesToMeters(181.907204), Units.inchesToMeters(158.843750));

    /**
     * Gets the hub position for the current alliance.
     *
     * @return the hub position, flipped for red alliance.
     */
    public static Translation2d getHubPosition() {
      if (ifOnBlue()) {
        return kBlueMiddleHub;
      }
      return com.pathplanner.lib.util.FlippingUtil.flipFieldPosition(kBlueMiddleHub);
    }
  }

  public static class SwerveConstants {
    public static final double kDrivetrainWidth = Units.inchesToMeters(26.94);
    public static final double kDrivetrainLength = Units.inchesToMeters(26.94);
    public static final double kRobotWidth = Units.inchesToMeters(33.876000);
    public static final double kRobotLength = Units.inchesToMeters(33.876000);

    // Robot mass and moment of inertia (for GoToPose feedforward calculations)
    public static final double kRobotMassKg = 54.4; // ~120 lbs with bumpers and battery
    public static final double kRobotMOIKgM2 = 6.0; // kg*m^2 (estimate, tune via SysId)

    // GoToPose motion constraints
    public static final double kGoToPoseMaxVelocity = 3.5; // m/s
    public static final double kGoToPoseMaxAcceleration = 2.5; // m/s^2
    public static final double kGoToPoseMaxAngularVelocity = Math.PI * 1.5; // rad/s
    public static final double kGoToPoseMaxAngularAcceleration = Math.PI; // rad/s^2

    // GoToPose tolerances
    public static final double kGoToPoseLinearTolerance = 0.02; // meters
    public static final double kGoToPoseAngularTolerance = Math.toRadians(1.0); // radians

    // Path following translation PID constants
    public static final double kPathTranslationP = 10.0;
    public static final double kPathTranslationI = 0.0;
    public static final double kPathTranslationD = 0.0;

    // Rotation PID constants (used for path following and FieldCentricFacingAngle)
    public static final double kRotationP = 7.0;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;

    // Deadband percentage for translation (0.1 = 10%)
    public static final double kDeadbandPercent = 0.1;

    public static final LinearVelocity translationMaxSpeed = MetersPerSecond.of(4.58);
    public static final LinearVelocity sotmMaxSpeed = MetersPerSecond.of(1.5);
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 15;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 16;
    public static final int SHOOTER_MOTOR_TOP_ROLLER_ID = 17;

    public static final int SHOOTER_SUPPLY_LIMIT = 60;
    public static final int SHOOTER_STATOR_LIMIT = 120;

    // Simulation constants
    public static final double MAIN_SHOOTER_GEAR_RATIO = 1.0;
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double MAIN_SHOOTER_MOI = 0.004; // kg*m^2
    public static final double ROLLER_MOI = 0.001; // kg*m^2

    // Main shooter PID constants (Slot 0)
    public static final double kMainShooterKS = 5.73487;
    public static final double kMainShooterKV = 0.00042955;
    public static final double kMainShooterKA = 0.0;
    public static final double kMainShooterKP = 4.3;
    public static final double kMainShooterKI = 0.0;
    public static final double kMainShooterKD = 0.0;

    // Roller PID constants (Slot 0)
    public static final double kRollerKS = 1.34;
    public static final double kRollerKV = 0.000185;
    public static final double kRollerKA = 0.0;
    public static final double kRollerKP = 1.05;
    public static final double kRollerKI = 0.0;
    public static final double kRollerKD = 0.0;

    // Velocity tolerance for "at speed" detection (rotations per second)
    public static final double kVelocityToleranceRps = 20.53;
    // .finallyDo(() -> m_drivetrain.stopModules());
    // Interpolating maps for shooter speeds based on distance to goal (meters)
    // Maps distance (m) -> speed (rotations per second)
    public static final InterpolatingDoubleTreeMap BOTTOM_SHOOTER_SPEED_MAP =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap TOP_ROLLER_SPEED_MAP =
        new InterpolatingDoubleTreeMap();

    // Time of flight map for SOTF calculations
    // Maps distance (m) -> time of flight (s)
    // TODO: Measure these values using a phone camera at 60fps
    public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP =
        new InterpolatingDoubleTreeMap();

    // Passing mode speeds (RPM)
    // Used for passing game pieces to teammates from across the field
    public static final double kPassingMainShooterRPM = 1500;
    public static final double kPassingRollerRPM = 5500;

    // SOTF (Shooting On The Fly) constants
    // Total latency compensation in seconds (camera + motor lag + ball flight through shooter)
    // TODO: Tune this value - start at 0.1s, increase if shots land behind target
    public static final double kSOTFLatencyCompensation = 0.075;

    public static final List<ShootingDataPoint> shootingDataPoints = new ArrayList<>();

    static {
      shootingDataPoints.add(new ShootingDataPoint(1.292, 2300.0, 250.0, 1.06));
      shootingDataPoints.add(new ShootingDataPoint(1.63, 2500.0, 250.0));
      shootingDataPoints.add(new ShootingDataPoint(1.77, 2500.0, 280.0));
      shootingDataPoints.add(new ShootingDataPoint(1.93, 2700.0, 250.0));
      shootingDataPoints.add(new ShootingDataPoint(2.30, 2700.0, 550.0, 1.20));
      shootingDataPoints.add(new ShootingDataPoint(2.40, 2700.0, 600.0));
      shootingDataPoints.add(new ShootingDataPoint(2.41, 1600.0, 3205.3));
      shootingDataPoints.add(new ShootingDataPoint(2.49, 1600.0, 3400.0));
      shootingDataPoints.add(new ShootingDataPoint(2.82, 1600.0, 3700.0, 1.22));
      shootingDataPoints.add(new ShootingDataPoint(3.24, 1600.0, 4100.0));
      shootingDataPoints.add(new ShootingDataPoint(3.55, 1500.0, 4300.0, 1.16));
      shootingDataPoints.add(new ShootingDataPoint(3.90, 1700.0, 4350.0));
      shootingDataPoints.add(new ShootingDataPoint(4.95, 1500.0, 5300.0, 1.58));
      shootingDataPoints.add(new ShootingDataPoint(5.40, 1700.0, 5500.0, 1.63));

      for (ShootingDataPoint point : shootingDataPoints) {
        BOTTOM_SHOOTER_SPEED_MAP.put(point.distance(), point.bottomRpm());
        TOP_ROLLER_SPEED_MAP.put(point.distance(), point.topRollerRpm());
        if (point.tof() != null) {
          TIME_OF_FLIGHT_MAP.put(point.distance(), point.tof());
        }
      }
    }
  }

  public static class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 18;

    public static final int SPINDEXER_SUPPLY_LIMIT = 40;
    public static final int SPINDEXER_STATOR_LIMIT = 60;

    public static final boolean INVERTED = true;

    // Spin voltage for feeding game pieces
    public static final double kSpinVoltage = 12.0;

    // Simulation constants
    public static final double SPINDEXER_GEAR_RATIO = 16.0;
    public static final double SPINDEXER_MOI = 0.002; // kg*m^2
  }

  public static class KickerConstants {
    public static final int KICKER_MOTOR_ID = 19;

    public static final int KICKER_SUPPLY_LIMIT = 40;
    public static final int KICKER_STATOR_LIMIT = 120;

    // Spin voltage for feeding game pieces
    public static final double kSpinVoltage = 12.0;

    // Simulation constants
    public static final double KICKER_GEAR_RATIO = 1.0;
    public static final double KICKER_MOI = 0.002; // kg*m^2
  }

  public static class IntakeConstants {
    public static final int RACK_MOTOR_ID = 20;
    public static final int ROLLER_MOTOR_ID = 21;

    public static final int RACK_SUPPLY_LIMIT = 40;
    public static final int RACK_STATOR_LIMIT = 50;
    public static final int ROLLER_SUPPLY_LIMIT = 40;
    public static final int ROLLER_STATOR_LIMIT = 80;

    // Gear ratios
    public static final double RACK_GEAR_RATIO = 5.0; // Motor to pinion gearbox ratio
    public static final double ROLLER_GEAR_RATIO = 1.0;

    // Rack and pinion parameters (10 DP, 10 tooth pinion, 53 tooth rack)
    public static final int RACK_PINION_TEETH = 10;
    public static final int RACK_TEETH = 53;
    public static final int RACK_DIAMETRAL_PITCH = 10;
    // Pinion pitch diameter = teeth / DP = 10/10 = 1 inch
    public static final double RACK_PINION_PITCH_RADIUS_METERS = Units.inchesToMeters(0.5);
    // Max rack travel = rack teeth × circular pitch = 53 × (π/10) ≈ 16.65 inches
    public static final double RACK_MAX_TRAVEL_METERS = Units.inchesToMeters(53.0 * Math.PI / 10.0);
    // Rack angle
    public static final double RACK_ANGLE_DEGREES = 22.022506;

    // Rack position constants (in rotations at mechanism/pinion)
    // Max travel = rack teeth / pinion teeth = 53 / 10 = 5.3 rotations
    public static final double kRackDeployedPosition = 2.5;
    public static final double kRackStowedPosition = 0.0;

    // Offset (in rotations) from deployed position when wiggling during feeding
    // Negative to retract 75% of the way back toward stowed
    public static final double kRackFeedingWiggleOffset = -0.75 * kRackDeployedPosition;

    // Rack PID constants (Slot 0)
    public static final double kRackKS = 0.395;
    public static final double kRackKG = 0.0;
    public static final double kRackKV = 0.0;
    public static final double kRackKA = 0.0;
    public static final double kRackKP = 64.0;
    public static final double kRackKI = 0.0;
    public static final double kRackKD = 0.0;
    public static final double kRackMotionMagicCruiseVelocity = 40.0; // rotations per second
    public static final double kRackMotionMagicAcceleration = 80.0; // rotations per second^2

    // Roller voltage for intaking
    public static final double kIntakeVoltage = -12.0;
    public static final double nonheldVoltage = 0.0;
    public static final double kEjectVoltage = 6.0;

    // Simulation constants (from CAD)
    // Mass of the moving rack carriage
    public static final double RACK_CARRIAGE_MASS_KG = Units.lbsToKilograms(10.224711);
    // Min/max positions for simulation (in meters)
    public static final double RACK_MIN_EXTENSION_METERS = 0.0;
    public static final double RACK_MAX_EXTENSION_METERS = RACK_MAX_TRAVEL_METERS;
    public static final double ROLLER_MOI = 0.001; // kg*m^2

    // Position tolerance for "at position" detection (rotations)
    public static final double kRackPositionToleranceRotations = 0.02;
  }

  public static class VisionConstants {
    public static final String kFrontCameraName = "UpperPortCamera";
    public static final Transform3d kFrontRobotToCam =
        new Transform3d(
            new Translation3d(0.131101, -0.0381, 0.718031),
            // new Translation3d(0, 0, 0),
            // new Rotation3d(0, 0, 0));
            new Rotation3d(0, Units.degreesToRadians(-15), 0));

    public static final String kBackCameraName = "BottomPortCamera";
    public static final Transform3d kBackRobotToCam =
        new Transform3d(
            new Translation3d(0.100942, -0.292100, 0.7180),
            // new Translation3d(0, 0, 0),
            // new Rotation3d(0, 0, Units.degreesToRadians(180)));
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(180)));
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class FuelConstants {
    private FuelConstants() {}

    // Shooter position offset from robot center (robot-relative)
    // X = forward, Y = left, Z = up
    // Negated X and Y for 180° front/back swap
    public static final Translation3d kShooterOffset =
        new Translation3d(
            Units.inchesToMeters(7.72),
            Units.inchesToMeters(-6.720000),
            Units.inchesToMeters(22.99));

    // Target height (hub opening height in meters)
    public static final double kTargetHeight = Units.feetToMeters(6.0);

    // Robot frame dimensions
    public static final double kHalfRobotLength = Units.inchesToMeters(13.25);

    // Hub geometry (front face to aim point center)
    public static final double kHubFrontToCenter = Units.inchesToMeters(23.51378);

    // Lookup table distance offset
    // Our table was measured from front of robot to front of hub,
    // so we add this to convert to shooter-to-goal-center distance
    public static final double kLookupTableDistanceOffset = kHalfRobotLength + kHubFrontToCenter;
  }
}
