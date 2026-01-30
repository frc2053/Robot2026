package frc.robot;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

  public static class FieldSpots {
    public static final Translation2d kBlueMiddleHub =
        new Translation2d(Units.inchesToMeters(181.907204), Units.inchesToMeters(158.843750));
  }

  public static class SwerveConstants {
    public static final double kDrivetrainWidth = Units.inchesToMeters(26.94);
    public static final double kDrivetrainLength = Units.inchesToMeters(26.94);
    public static final double kRobotWidth = Units.inchesToMeters(33.876000);
    public static final double kRobotLength = Units.inchesToMeters(33.876000);

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
  }

  public static class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 15;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 16;
    public static final int SHOOTER_MOTOR_TOP_ROLLER_ID = 17;

    public static final int SHOOTER_SUPPLY_LIMIT = 60;
    public static final int SHOOTER_STATOR_LIMIT = 80;

    // Simulation constants
    public static final double MAIN_SHOOTER_GEAR_RATIO = 1.0;
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double MAIN_SHOOTER_MOI = 0.004; // kg*m^2
    public static final double ROLLER_MOI = 0.001; // kg*m^2

    // Main shooter PID constants (Slot 0)
    public static final double kMainShooterKS = 0.1;
    public static final double kMainShooterKV = 0.12;
    public static final double kMainShooterKA = 0.01;
    public static final double kMainShooterKP = 0.11;
    public static final double kMainShooterKI = 0;
    public static final double kMainShooterKD = 0;
    public static final double kMainShooterMotionMagicAccel = 400;

    // Roller PID constants (Slot 0)
    public static final double kRollerKS = 0.1;
    public static final double kRollerKV = 0.12;
    public static final double kRollerKA = 0.01;
    public static final double kRollerKP = 0.15;
    public static final double kRollerKI = 0;
    public static final double kRollerKD = 0;
    public static final double kRollerMotionMagicAccel = 400;

    // Velocity tolerance for "at speed" detection (rotations per second)
    public static final double kVelocityToleranceRps = 2.0;

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

    // SOTF (Shooting On The Fly) constants
    // Total latency compensation in seconds (camera + motor lag + ball flight through shooter)
    // TODO: Tune this value - start at 0.1s, increase if shots land behind target
    public static final double kSOTFLatencyCompensation = 0.15;

    // Maximum horizontal velocity the shooter can achieve (m/s)
    // Used to determine if a shot is physically possible
    // TODO: Calculate from your max RPM and wheel radius
    public static final double kMaxHorizontalVelocity = 12.0;

    static {
      // Bottom shooter speeds (distance in meters -> speed in RPS)
      // TODO: Tune these values based on testing
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.inchesToMeters(39.37), 30.0); // ~1m
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.inchesToMeters(78.74), 40.0); // ~2m
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.inchesToMeters(118.11), 50.0); // ~3m
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.inchesToMeters(157.48), 60.0); // ~4m
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.inchesToMeters(196.85), 70.0); // ~5m

      // Top roller speeds (distance in meters -> speed in RPS)
      // TODO: Tune these values based on testing
      TOP_ROLLER_SPEED_MAP.put(Units.inchesToMeters(39.37), 25.0); // ~1m
      TOP_ROLLER_SPEED_MAP.put(Units.inchesToMeters(78.74), 35.0); // ~2m
      TOP_ROLLER_SPEED_MAP.put(Units.inchesToMeters(118.11), 45.0); // ~3m
      TOP_ROLLER_SPEED_MAP.put(Units.inchesToMeters(157.48), 55.0); // ~4m
      TOP_ROLLER_SPEED_MAP.put(Units.inchesToMeters(196.85), 65.0); // ~5m

      // Time of flight values (distance in meters -> flight time in seconds)
      // TODO: Measure these by recording shots at each distance with a phone camera
      // Count frames from ball leaving shooter to reaching goal, divide by framerate
      TIME_OF_FLIGHT_MAP.put(1.0, 0.40); // 1m
      TIME_OF_FLIGHT_MAP.put(2.0, 0.50); // 2m
      TIME_OF_FLIGHT_MAP.put(3.0, 0.62); // 3m
      TIME_OF_FLIGHT_MAP.put(4.0, 0.75); // 4m
      TIME_OF_FLIGHT_MAP.put(5.0, 0.90); // 5m
    }
  }

  public static class SpindexerConstants {
    public static final int SPINDEXER_MOTOR_ID = 18;

    public static final int SPINDEXER_SUPPLY_LIMIT = 40;
    public static final int SPINDEXER_STATOR_LIMIT = 60;

    // Spin voltage for feeding game pieces
    public static final double kSpinVoltage = 6.0;

    // Simulation constants
    public static final double SPINDEXER_GEAR_RATIO = 5.0;
    public static final double SPINDEXER_MOI = 0.002; // kg*m^2
  }

  public static class VisionConstants {
    public static final String kFrontCameraName = "FrontCamera";
    public static final String kSideCameraName = "SideCamera";
    public static final Transform3d kFrontRobotToCam =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-1.875000), Units.inchesToMeters(3.518814), Units.inchesToMeters(25.102211)),
            new Rotation3d(0, Units.degreesToRadians(-15), 0));
    public static final Transform3d kSideRobotToCam =
        new Transform3d(
            new Translation3d(-0.342138, 0.342138, 0.7112),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90)));
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
