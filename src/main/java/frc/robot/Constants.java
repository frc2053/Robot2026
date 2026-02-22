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

  public static final class FieldSpots {
    private FieldSpots() {}

    public static final double kFieldLength = Units.inchesToMeters(651.25); // 54 ft 3.25 in
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
    public static final double kMainShooterKS = 0.2;
    public static final double kMainShooterKV = 0.117;
    public static final double kMainShooterKA = 0.0;
    public static final double kMainShooterKP = 0.5;
    public static final double kMainShooterKI = 0;
    public static final double kMainShooterKD = 0;
    public static final double kMainShooterMotionMagicAccel = 400;

    // Roller PID constants (Slot 0)
    public static final double kRollerKS = 0.19;
    public static final double kRollerKV = 0.115;
    public static final double kRollerKA = 0.00;
    public static final double kRollerKP = 0.5;
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

    // Estimated velocity uncertainty in m/s (for first-order miss metric)
    // This is how uncertain we are about our robot's velocity from odometry
    // TODO: Tune based on odometry quality — lower is better
    public static final double kVelocityUncertainty = 0.3;

    static {
      // Bottom shooter speeds (distance in meters -> speed in RPM)
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(5), 4000.0); 
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(6), 4500.0); 
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(7), 4500.0); 
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(8), 4700.0);
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(9), 4700.0);
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(10), 5000.0);
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(11), 5100.0);
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(12), 5200.0);
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(13), 5200.0);
      BOTTOM_SHOOTER_SPEED_MAP.put(Units.feetToMeters(14), 5300.0);

      // Top roller speeds (distance in meters -> speed in RPM)
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(5), -3500.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(6), -3800.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(7), -3700.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(8), -3700.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(9), -3800.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(10), -3800.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(11), -3800.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(12), -3800.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(13), -3400.0);
      TOP_ROLLER_SPEED_MAP.put(Units.feetToMeters(14), -3500.0);

      // Time of flight values (distance in feet -> flight time in seconds)
      // Count frames from ball leaving shooter to reaching goal, divide by framerate
      // TODO: Tune these values based on testing with actual robot and camera
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(5), 0.40);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(6), 0.50);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(7), 0.62);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(8), 0.75);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(9), 0.90);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(10), 1.0);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(11), 1.1);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(12), 1.2);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(13), 1.3);
      TIME_OF_FLIGHT_MAP.put(Units.feetToMeters(14), 1.4);
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
    public static final int KICKER_STATOR_LIMIT = 60;

    // Spin voltage for feeding game pieces
    public static final double kSpinVoltage = 12.0;

    // Simulation constants
    public static final double KICKER_GEAR_RATIO = 5.0;
    public static final double KICKER_MOI = 0.002; // kg*m^2
  }

  public static class IntakeConstants {
    public static final int PIVOT_MOTOR_ID = 20;
    public static final int ROLLER_MOTOR_ID = 21;

    public static final int PIVOT_SUPPLY_LIMIT = 40;
    public static final int PIVOT_STATOR_LIMIT = 60;
    public static final int ROLLER_SUPPLY_LIMIT = 40;
    public static final int ROLLER_STATOR_LIMIT = 60;

    // Gear ratios
    public static final double PIVOT_GEAR_RATIO = 56.0;
    public static final double ROLLER_GEAR_RATIO = 1.0;

    // Pivot position constants (in rotations at mechanism)
    public static final double kPivotDeployedPosition = 0.0;
    public static final double kPivotStowedPosition = 0.25;

    // Pivot PID constants (Slot 0)
    public static final double kPivotKS = 0.0;
    public static final double kPivotKG = 0.0;
    public static final double kPivotKV = 0.0;
    public static final double kPivotKA = 0.0;
    public static final double kPivotKP = 0;
    public static final double kPivotKI = 0.0;
    public static final double kPivotKD = 0.0;
    public static final double kPivotMotionMagicCruiseVelocity = 40.0; // rotations per second
    public static final double kPivotMotionMagicAcceleration = 80.0; // rotations per second^2

    // Roller voltage for intaking
    public static final double kIntakeVoltage = 8.0;
    public static final double kEjectVoltage = -6.0;

    // Simulation constants (from CAD)
    // COM distance from pivot
    public static final double PIVOT_ARM_LENGTH_METERS = Units.inchesToMeters(11.549);
    public static final double PIVOT_ARM_MASS_KG = Units.lbsToKilograms(10.224711);
    // MOI conversion: in^2*lb to kg*m^2
    public static final double PIVOT_MOI =
        1585.915769 * Math.pow(Units.inchesToMeters(1), 2) * Units.lbsToKilograms(1);
    public static final double ROLLER_MOI = 0.001; // kg*m^2

    // Position tolerance for "at position" detection (rotations)
    public static final double kPivotPositionToleranceRotations = 0.02;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 22;

    public static final int CLIMBER_SUPPLY_LIMIT = 60;
    public static final int CLIMBER_STATOR_LIMIT = 120;

    // Gear ratio (81:1)
    public static final double CLIMBER_GEAR_RATIO = 81.0;

    // Total travel distance
    public static final double kMaxHeightMeters = Units.inchesToMeters(5.0);
    public static final double kMinHeightMeters = 0.0;

    // Drum radius for converting rotations to linear distance (0.75 inch diameter)
    public static final double kDrumRadiusMeters = Units.inchesToMeters(0.375);

    // Position constants (in meters)
    public static final double kRetractedPosition = 0.0;
    public static final double kExtendedPosition = Units.inchesToMeters(5);

    // Climber PID constants (Slot 0)
    public static final double kClimberKS = 0.0;
    public static final double kClimberKG = 0.0;
    public static final double kClimberKV = 0.0;
    public static final double kClimberKA = 0.0;
    public static final double kClimberKP = 0.0;
    public static final double kClimberKI = 0.0;
    public static final double kClimberKD = 0.0;
    public static final double kClimberMotionMagicCruiseVelocity = 10.0; // rotations per second
    public static final double kClimberMotionMagicAcceleration = 20.0; // rotations per second^2

    // Simulation constants
    // Light load when extending up (just the climber hook/carriage)
    public static final double CLIMBER_CARRIAGE_MASS_KG = Units.lbsToKilograms(2.0);
    // Heavy load when climbing (lifting the robot)
    public static final double CLIMBER_ROBOT_MASS_KG = Units.lbsToKilograms(200.0);

    // Position tolerance for "at position" detection (meters)
    public static final double kPositionToleranceMeters = 0.005;

    // Climb voltage for manual control
    public static final double kClimbVoltage = 12.0;
    public static final double kRetractVoltage = -12.0;
  }

  public static class VisionConstants {
    public static final String kFrontCameraName = "FrontCamera";
    public static final String kSideCameraName = "SideCamera";
    public static final Transform3d kFrontRobotToCam =
        new Transform3d(
            new Translation3d(-0.0887618, 0.037973, 0.6394273),
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
